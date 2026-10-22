"""Small, bounded controllers for the synthetic grasping demonstration."""

from __future__ import annotations

import math
from numbers import Real

import numpy as np


def _scalar(name: str, value: float, *, positive: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        qualifier = "positive and finite" if positive else "finite"
        raise ValueError(f"{name} must be {qualifier}")
    return result


def _array(name, values):
    try:
        original = np.asarray(values)
        if original.dtype.kind not in "iuf":
            raise ValueError("non-real numeric dtype")
        return np.array(original, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain finite real numbers") from exc


class PID:
    """Fixed-step PID with per-axis output limits and conditional integration.

    Scalar and array errors are supported. The shape must remain unchanged until
    ``reset``. The first sample has zero derivative, avoiding a startup kick.
    Integration stops only when it would push a saturated output further in the
    same direction; an opposite-sign error can unwind the accumulated integral.
    This is a numerical demo controller, not a physical robot safety controller.
    """

    def __init__(self, kp: float, ki: float, kd: float, dt: float, limit: float):
        self.kp = _scalar("kp", kp)
        self.ki = _scalar("ki", ki)
        self.kd = _scalar("kd", kd)
        if min(self.kp, self.ki, self.kd) < 0:
            raise ValueError("PID gains must be nonnegative")
        self.dt = _scalar("dt", dt, positive=True)
        self.limit = _scalar("limit", limit, positive=True)
        self.reset()

    def reset(self) -> None:
        """Clear accumulated error, derivative history, and shape binding."""
        self._integral = None
        self._previous = None

    def compute(self, error):
        current = _array("error", error)
        if current.size == 0 or not np.all(np.isfinite(current)):
            raise ValueError("error must contain finite real numbers")
        if self._previous is not None and current.shape != self._previous.shape:
            raise ValueError("error shape changed; reset the PID first")
        if self._previous is None:
            previous_integral = np.zeros_like(current)
            derivative = np.zeros_like(current)
        else:
            previous_integral = self._integral
            with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
                derivative = (current - self._previous) / self.dt

        with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
            candidate = previous_integral + current * self.dt
            # Also bound the stored integral itself, including during reversal.
            if self.ki > 0:
                integral_limit = self.limit / self.ki
                candidate = np.clip(candidate, -integral_limit, integral_limit)
            else:
                candidate = np.zeros_like(current)
            proposed = self.kp * current + self.ki * candidate + self.kd * derivative
            outward = ((proposed > self.limit) & (current > 0)) | (
                (proposed < -self.limit) & (current < 0)
            )
            integral = np.where(outward, previous_integral, candidate)
            command = self.kp * current + self.ki * integral + self.kd * derivative
        if not np.all(np.isfinite(command)) or not np.all(np.isfinite(integral)):
            raise ValueError("PID arithmetic exceeded finite numeric range")
        self._integral = np.array(integral, copy=True)
        self._previous = current
        command = np.clip(command, -self.limit, self.limit)
        return float(command) if command.ndim == 0 else command


def dls_step(jacobian, error, damping: float = 0.03, max_step: float = 0.08):
    """Return clipped joint increments using damped least squares.

    For an m-by-n Jacobian J and m-vector e, solves
    ``(J J.T + damping**2 I) y = e`` and returns ``clip(J.T y)``.
    This deliberately uses a linear solve instead of an explicit inverse.
    ``max_step`` is a per-joint increment bound, not a vector-norm bound.
    """
    damping = _scalar("damping", damping, positive=True)
    max_step = _scalar("max_step", max_step, positive=True)
    matrix = _array("jacobian", jacobian)
    residual = _array("error", error)
    if matrix.ndim != 2 or min(matrix.shape, default=0) == 0:
        raise ValueError("jacobian must be a nonempty m-by-n matrix")
    if residual.shape != (matrix.shape[0],):
        raise ValueError("error must be a vector matching jacobian rows")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(residual)):
        raise ValueError("jacobian and error must contain only finite values")
    with np.errstate(over="ignore", invalid="ignore", under="ignore"):
        normal = matrix @ matrix.T + damping * damping * np.eye(matrix.shape[0])
    if not np.all(np.isfinite(normal)):
        raise ValueError("DLS arithmetic exceeded finite numeric range")
    try:
        solved = np.linalg.solve(normal, residual)
        increments = matrix.T @ solved
    except np.linalg.LinAlgError as exc:
        raise ValueError("damping is insufficient for a stable linear solve") from exc
    if not np.all(np.isfinite(increments)):
        raise ValueError("DLS arithmetic exceeded finite numeric range")
    return np.clip(increments, -max_step, max_step)
