"""Integration checks run the actual DIRECT physics and RGB-D pipeline."""

import json
import unittest
from unittest.mock import patch

import numpy as np
import pybullet as p

from grasp_demo.simulation import run_episode


class SimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal = run_episode(seed=7, record=False)

    def test_nominal_lifts_and_holds_actual_simulated_ball(self):
        metrics = self.nominal["metrics"]
        self.assertTrue(metrics["success"], metrics["failure_reason"])
        self.assertGreater(metrics["lift_height_m"], 0.12)
        self.assertIn("constraint-assisted", metrics["grasp_mode"])
        self.assertEqual(metrics["constraint_creation_displacement_m"], 0.0)
        self.assertLess(metrics["localization_error_m"], 0.01)
        self.assertEqual([event["stage"] for event in metrics["stage_events"]],
                         ["detect", "approach", "descend", "grasp", "lift", "hold", "done"])
        self.assertGreater(metrics["ik_iterations"], 0)

    def test_metrics_are_finite_json_and_times_are_physics_ticks(self):
        metrics = self.nominal["metrics"]
        json.dumps(metrics, allow_nan=False)
        self.assertAlmostEqual(metrics["simulated_seconds"], metrics["steps"] * metrics["dt"], places=5)
        self.assertEqual(self.nominal["frames"], [])
        self.assertGreater(len(self.nominal["trajectory"]), 1)
        self.assertTrue(all(b["step"] > a["step"] for a, b in zip(self.nominal["trajectory"], self.nominal["trajectory"][1:])))
        initial = metrics["true_center_initial"]
        final = metrics["true_center_final"]
        self.assertAlmostEqual(metrics["lift_height_m"], final[2] - initial[2])

    def test_nominal_is_reproducible_for_same_seed(self):
        again = run_episode(seed=7, record=False)["metrics"]
        first = self.nominal["metrics"]
        self.assertEqual(again["success"], first["success"])
        self.assertEqual(again["steps"], first["steps"])
        np.testing.assert_allclose(again["true_center_initial"], first["true_center_initial"], atol=1e-10)
        np.testing.assert_allclose(again["true_center_final"], first["true_center_final"], atol=1e-10)

    def test_missing_red_target_fails_before_planning(self):
        result = run_episode(case="no-target", record=False)
        metrics = result["metrics"]
        self.assertFalse(metrics["success"])
        self.assertEqual(metrics["failure_reason"], "red_target_not_detected")
        self.assertIsNone(metrics["estimated_center"])
        self.assertIsNone(metrics["constraint_creation_displacement_m"])
        self.assertEqual(metrics["ik_iterations"], 0)

    def test_visible_target_outside_workspace_is_rejected(self):
        metrics = run_episode(case="out-of-workspace", record=False)["metrics"]
        self.assertFalse(metrics["success"])
        self.assertEqual(metrics["failure_reason"], "estimated_target_out_of_workspace")
        self.assertIsNotNone(metrics["estimated_center"])
        self.assertEqual(metrics["ik_iterations"], 0)
        self.assertIsNone(metrics["constraint_creation_displacement_m"])

    def test_recording_yields_only_new_synthetic_frames(self):
        result = run_episode(case="no-target", record=True)
        self.assertGreater(len(result["frames"]), 0)
        self.assertTrue(all(frame.mode == "RGB" and frame.size == (400, 300) for frame in result["frames"]))
        self.assertTrue(all(0 <= stage["frame_index"] < len(result["frames"]) for stage in result["stages"]))

    def test_simulation_disconnects_even_on_unexpected_detector_error(self):
        original = p.connect
        clients = []
        def connected(*args, **kwargs):
            client = original(*args, **kwargs)
            clients.append(client)
            return client
        with patch("grasp_demo.simulation.p.connect", side_effect=connected):
            with patch("grasp_demo.simulation.detect_red", side_effect=RuntimeError("synthetic failure")):
                with self.assertRaisesRegex(RuntimeError, "synthetic failure"):
                    run_episode(record=False)
        self.assertTrue(clients)
        self.assertFalse(p.isConnected(clients[0]))

    def test_invalid_parameters_never_create_physics_connection(self):
        with patch("grasp_demo.simulation.p.connect") as connect:
            for arguments in ({"seed": -1}, {"seed": True}, {"seed": 2**32}, {"seed": 1.5},
                              {"case": "real-robot"}, {"record": "yes"}):
                with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                    run_episode(**arguments)
            connect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
