"""Synthetic RGB-D guided Panda demonstration, entirely inside PyBullet.

This is an explicitly constraint-assisted grasp experiment.  A fixed attachment
is made only after approach and finger closure, preserving the current relative
pose.  It is not evidence of contact-only grasping or real-robot performance.
"""

from __future__ import annotations

from importlib.metadata import version
import math
import platform

import numpy as np
from PIL import Image
import pybullet as p
import pybullet_data

from .control import PID, dls_step
from .vision import detect_red


DT = 1.0 / 240.0
BALL_RADIUS = 0.025
IMAGE_SIZE = (400, 300)
FRAME_INTERVAL = 12
ARM = list(range(7))
FINGERS = [9, 10]
TIP = 11
MODE = "constraint-assisted; not contact-only"
CASES = ("nominal", "no-target", "out-of-workspace")


def _rotation_error(current, target):
    """World-frame rotation vector taking current orientation to target."""
    inverse = [-current[0], -current[1], -current[2], current[3]]
    _, difference = p.multiplyTransforms([0, 0, 0], target, [0, 0, 0], inverse)
    q = np.asarray(difference, dtype=float)
    if q[3] < 0:
        q *= -1
    length = float(np.linalg.norm(q[:3]))
    if length < 1e-10:
        return np.zeros(3)
    return q[:3] * (2 * math.atan2(length, max(0.0, q[3])) / length)


def _pose(client, robot):
    state = p.getLinkState(robot, TIP, computeForwardKinematics=True, physicsClientId=client)
    return np.asarray(state[4], dtype=float), state[5]


def _plan(client, robot, goal, orientation, lower, upper):
    """Use manually iterated pose DLS, restoring the live physical state.

    resetJointState is used only for temporary forward-kinematics evaluation;
    both positions and velocities are restored before any physics step.
    Actual movement is performed by the PID velocity controller in _Episode.
    """
    original = p.getJointStates(robot, ARM, physicsClientId=client)
    q = np.array([item[0] for item in original], dtype=float)
    converged = False
    position_error = float("inf")
    rotation_error = float("inf")
    iteration = 0
    try:
        for iteration in range(240):
            for joint, angle in zip(ARM, q):
                p.resetJointState(robot, joint, float(angle), physicsClientId=client)
            point, rotation = _pose(client, robot)
            delta = np.asarray(goal) - point
            angular = _rotation_error(rotation, orientation)
            position_error = float(np.linalg.norm(delta))
            rotation_error = float(np.linalg.norm(angular))
            if position_error < 0.002 and rotation_error < 0.025:
                converged = True
                break
            coordinates = list(q) + [
                p.getJointState(robot, index, physicsClientId=client)[0]
                for index in FINGERS
            ]
            linear, angular_jacobian = p.calculateJacobian(
                robot, TIP, [0, 0, 0], coordinates, [0.0] * 9, [0.0] * 9,
                physicsClientId=client,
            )
            jacobian = np.vstack((np.asarray(linear)[:, :7], 0.35 * np.asarray(angular_jacobian)[:, :7]))
            error = np.concatenate((delta, 0.35 * angular))
            q = np.clip(q + dls_step(jacobian, error, damping=0.025, max_step=0.08), lower, upper)
    finally:
        for joint, state in zip(ARM, original):
            p.resetJointState(robot, joint, state[0], state[1], physicsClientId=client)
    return q, converged, iteration + 1, position_error, rotation_error


class _Episode:
    def __init__(self, client, seed, case, record):
        self.client = client
        self.seed = seed
        self.case = case
        self.record = record
        self.steps = 0
        self.frames = []
        self.stages = []
        self.events = []
        self.trajectory = []
        self.stage = "initialize"
        self.attachment = None
        self.failure = ""
        self.estimate = None
        self.localization_error = None
        self.ik_iterations = 0
        self.constraint_snap = None
        self.success = False
        self.max_lift = 0.0
        rng = np.random.default_rng(seed)
        self.view = p.computeViewMatrix([0.50, 0.0, 1.05], [0.50, 0.0, 0.0], [0, 1, 0])
        self.projection = p.computeProjectionMatrixFOV(48.0, IMAGE_SIZE[0] / IMAGE_SIZE[1], 0.02, 2.0)
        self.overview_view = p.computeViewMatrix([1.25, -1.0, 0.85], [0.35, 0.0, 0.16], [0, 0, 1])
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.setTimeStep(DT, physicsClientId=client)
        p.setPhysicsEngineParameter(numSolverIterations=100, deterministicOverlappingPairs=1, physicsClientId=client)
        p.loadURDF("plane.urdf", [0, 0, -0.07], physicsClientId=client)
        table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.43, 0.30, 0.025], physicsClientId=client)
        table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.43, 0.30, 0.025], rgbaColor=[0.64, 0.71, 0.74, 1], physicsClientId=client)
        p.createMultiBody(0, table_collision, table_visual, [0.5, 0, -0.025], physicsClientId=client)
        self.robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True, physicsClientId=client)
        self.orientation = p.getQuaternionFromEuler([math.pi, 0, 0])
        for joint, angle in zip(ARM, [0.0, -0.65, 0.0, -2.25, 0.0, 1.65, 0.78]):
            p.resetJointState(self.robot, joint, angle, physicsClientId=client)
        initial = p.calculateInverseKinematics(
            self.robot, TIP, [0.39, -0.20, 0.34], self.orientation,
            maxNumIterations=300, residualThreshold=1e-6, physicsClientId=client,
        )
        for joint, angle in zip(ARM, initial[:7]):
            p.resetJointState(self.robot, joint, angle, physicsClientId=client)
        for finger in FINGERS:
            p.resetJointState(self.robot, finger, 0.04, physicsClientId=client)
        self.target_joints = np.array(initial[:7])
        self.pid = [PID(kp=9.0, ki=0.1, kd=0.025, dt=DT, limit=1.5) for _ in ARM]
        self.lower = np.array([p.getJointInfo(self.robot, j, physicsClientId=client)[8] for j in ARM])
        self.upper = np.array([p.getJointInfo(self.robot, j, physicsClientId=client)[9] for j in ARM])
        center = [float(rng.uniform(0.47, 0.54)), float(rng.uniform(-0.045, 0.045)), BALL_RADIUS]
        if case == "out-of-workspace":
            center = [0.82, 0.0, BALL_RADIUS]
        color = [0.05, 0.15, 0.85, 1] if case == "no-target" else [0.95, 0.015, 0.015, 1]
        shape = p.createCollisionShape(p.GEOM_SPHERE, radius=BALL_RADIUS, physicsClientId=client)
        visible = p.createVisualShape(p.GEOM_SPHERE, radius=BALL_RADIUS, rgbaColor=color, physicsClientId=client)
        self.ball = p.createMultiBody(0.025, shape, visible, center, physicsClientId=client)
        p.changeDynamics(self.ball, -1, lateralFriction=0.8, rollingFriction=0.01, physicsClientId=client)
        for _ in range(60):
            self.tick(False)
        self.initial_center = self.ball_position()
        self.change_stage("detect")

    def ball_position(self):
        return np.asarray(p.getBasePositionAndOrientation(self.ball, physicsClientId=self.client)[0], dtype=float)

    def render(self, overhead=False):
        width, height, rgba, depth, _ = p.getCameraImage(
            *IMAGE_SIZE, viewMatrix=self.view if overhead else self.overview_view,
            projectionMatrix=self.projection, renderer=p.ER_TINY_RENDERER,
            lightDirection=[-1, -1, 3], shadow=0, physicsClientId=self.client,
        )
        rgb = np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
        return rgb.copy(), np.asarray(depth).reshape(height, width)

    def capture(self):
        if self.record:
            self.frames.append(Image.fromarray(self.render()[0]))

    def change_stage(self, stage):
        self.stage = stage
        self.events.append({"stage": stage, "step": self.steps, "simulated_seconds": round(self.steps * DT, 6)})
        self.stages.append({"stage": stage, "frame_index": len(self.frames)})
        self.capture()
        for controller in self.pid:
            controller.reset()

    def tick(self, closed=False):
        actual = p.getJointStates(self.robot, ARM, physicsClientId=self.client)
        commands = [controller.compute(float(target - state[0])) for controller, target, state in zip(self.pid, self.target_joints, actual)]
        p.setJointMotorControlArray(
            self.robot, ARM, p.VELOCITY_CONTROL, targetVelocities=commands,
            forces=[87, 87, 87, 87, 12, 12, 12], physicsClientId=self.client,
        )
        p.setJointMotorControlArray(
            self.robot, FINGERS, p.POSITION_CONTROL, targetPositions=[0.0 if closed else 0.04] * 2,
            forces=[30, 30], physicsClientId=self.client,
        )
        p.stepSimulation(physicsClientId=self.client)
        self.steps += 1
        if hasattr(self, "initial_center"):
            self.max_lift = max(self.max_lift, float(self.ball_position()[2] - self.initial_center[2]))
        if self.steps % 4 == 0:
            tip, _ = _pose(self.client, self.robot)
            ball = self.ball_position()
            self.trajectory.append({
                "step": self.steps, "time_s": round(self.steps * DT, 6), "stage": self.stage,
                "tip_x": float(tip[0]), "tip_y": float(tip[1]), "tip_z": float(tip[2]),
                "ball_x": float(ball[0]), "ball_y": float(ball[1]), "ball_z": float(ball[2]),
            })
        if self.steps % FRAME_INTERVAL == 0:
            self.capture()

    def move(self, name, goal, closed=False):
        self.change_stage(name)
        result = _plan(self.client, self.robot, goal, self.orientation, self.lower, self.upper)
        self.target_joints, converged, iterations, _, _ = result
        self.ik_iterations += iterations
        if not converged:
            self.failure = "ik_not_converged:" + name
            return False
        for _ in range(240 * 5):
            self.tick(closed)
            point, rotation = _pose(self.client, self.robot)
            if np.linalg.norm(point - goal) < 0.009 and np.linalg.norm(_rotation_error(rotation, self.orientation)) < 0.12:
                return True
        self.failure = "stage_timeout:" + name
        return False

    def attach(self):
        """Preserve relative pose; no target reset or target teleportation."""
        point, orientation = _pose(self.client, self.robot)
        ball, ball_orientation = p.getBasePositionAndOrientation(self.ball, physicsClientId=self.client)
        if float(np.linalg.norm(point - np.array(ball))) > 0.065:
            self.failure = "gripper_not_near_target"
            return False
        inverse = p.invertTransform(point.tolist(), orientation)
        relative = p.multiplyTransforms(*inverse, ball, ball_orientation)
        before = self.ball_position()
        self.attachment = p.createConstraint(
            self.robot, TIP, self.ball, -1, p.JOINT_FIXED, [0, 0, 0], relative[0], [0, 0, 0],
            parentFrameOrientation=relative[1], childFrameOrientation=[0, 0, 0, 1], physicsClientId=self.client,
        )
        p.changeConstraint(self.attachment, maxForce=50, physicsClientId=self.client)
        self.constraint_snap = float(np.linalg.norm(self.ball_position() - before))
        return True

    def execute(self):
        rgb, depth = self.render(overhead=True)
        detection = detect_red(rgb, depth, self.view, self.projection)
        if detection is None:
            self.failure = "red_target_not_detected"
            return
        # The generic detector reports the whole component's median surface
        # depth. For this sphere-specific center prior, sample a small patch
        # at its detected centroid instead, so the visible cap is near its top.
        u, v = detection["pixel"]
        x, y = int(round(u)), int(round(v))
        patch = depth[max(0, y - 1):min(IMAGE_SIZE[1], y + 2), max(0, x - 1):min(IMAGE_SIZE[0], x + 2)]
        valid_depth = patch[(patch > 0) & (patch < 1)]
        if not valid_depth.size:
            self.failure = "centroid_depth_unavailable"
            return
        centroid_depth = float(np.median(valid_depth))
        projection = np.asarray(self.projection).reshape(4, 4, order="F")
        view = np.asarray(self.view).reshape(4, 4, order="F")
        clip = [2 * (u + 0.5) / IMAGE_SIZE[0] - 1, 1 - 2 * (v + 0.5) / IMAGE_SIZE[1], 2 * centroid_depth - 1, 1]
        homogeneous = np.linalg.solve(projection @ view, clip)
        surface = homogeneous[:3] / homogeneous[3]
        # The camera looks straight down onto a sphere of known radius. The
        # contour centroid's depth is a visible top-surface estimate. Radius is
        # subtracted along world Z; this limited geometric prior is NOT a
        # learned 6-D pose estimate and is not valid for arbitrary objects.
        self.estimate = surface - np.array([0, 0, BALL_RADIUS])
        self.localization_error = float(np.linalg.norm(self.estimate - self.initial_center))
        if not (0.33 <= self.estimate[0] <= 0.66 and abs(self.estimate[1]) <= 0.22 and -0.01 <= self.estimate[2] <= 0.09):
            self.failure = "estimated_target_out_of_workspace"
            return
        approach = self.estimate + [0, 0, 0.22]
        descend = self.estimate + [0, 0, 0.016]
        if not self.move("approach", approach):
            return
        if not self.move("descend", descend):
            return
        self.change_stage("grasp")
        for _ in range(120):
            self.tick(True)
        if not self.attach():
            return
        if not self.move("lift", self.estimate + [0, 0, 0.24], True):
            return
        self.change_stage("hold")
        hold_heights = []
        for _ in range(120):
            self.tick(True)
            hold_heights.append(float(self.ball_position()[2] - self.initial_center[2]))
        self.success = self.attachment is not None and min(hold_heights) > 0.12
        if not self.success:
            self.failure = "lift_not_held_above_threshold"

    def result(self):
        self.change_stage("done" if self.success else "failed")
        final_center = self.ball_position()
        metrics = {
            "schema_version": 1, "seed": self.seed, "case": self.case,
            "grasp_mode": MODE, "success": self.success, "failure_reason": self.failure,
            "dt": DT, "steps": self.steps, "simulated_seconds": round(self.steps * DT, 6),
            "estimated_center": None if self.estimate is None else self.estimate.tolist(),
            "true_center_initial": self.initial_center.tolist(),
            "true_center_final": final_center.tolist(), "localization_error_m": self.localization_error,
            "lift_height_m": float(final_center[2] - self.initial_center[2]),
            "maximum_lift_height_m": self.max_lift,
            "constraint_creation_displacement_m": self.constraint_snap,
            "ik_iterations": self.ik_iterations, "stage_events": self.events,
            "initial_pose": "PyBullet built-in IK initialization; subsequent targets use manual pose DLS",
            "center_estimation": "known-radius sphere, overhead camera; 3x3 centroid-depth patch unprojection minus radius along world Z",
            "software": {"python": platform.python_version(), "numpy": np.__version__, "pybullet": version("pybullet"), "Pillow": version("Pillow")},
        }
        return {"metrics": metrics, "frames": self.frames, "stages": self.stages, "trajectory": self.trajectory}


def run_episode(seed=7, case="nominal", record=True):
    """Run one fixed-seed synthetic simulation, never opening a GUI or files.

    Arm movement goals come from rendered RGB-D detection and the documented
    sphere-radius prior. Simulator object poses are used for evaluation and
    the explicitly assisted attachment's proximity check and relative frame.
    """
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**32:
        raise ValueError("seed must be an integer from 0 through 4294967295")
    if case not in CASES:
        raise ValueError("case must be nominal, no-target or out-of-workspace")
    if not isinstance(record, bool):
        raise ValueError("record must be a boolean")
    client = p.connect(p.DIRECT)
    if client < 0:
        raise RuntimeError("Could not create the local PyBullet simulation")
    try:
        episode = _Episode(client, seed, case, record)
        episode.execute()
        return episode.result()
    finally:
        if p.isConnected(client):
            p.disconnect(client)
