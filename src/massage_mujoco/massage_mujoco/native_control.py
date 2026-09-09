"""Small deterministic helpers for MuJoCo Viewer control inputs."""

import numpy as np


def limited_linear_velocity(current, target, gain, maximum_speed, tolerance):
    """Return a bounded world-frame velocity and whether the target is reached."""
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    parameters = np.asarray([gain, maximum_speed, tolerance], dtype=float)
    if current.shape != (3,) or target.shape != (3,):
        raise ValueError("TCP positions must contain three values")
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(target)):
        raise ValueError("TCP positions must be finite")
    if not np.all(np.isfinite(parameters)) or np.any(parameters <= 0.0):
        raise ValueError("TCP control parameters must be finite and positive")
    error = target - current
    distance = float(np.linalg.norm(error))
    if distance <= tolerance:
        return np.zeros(3), True
    speed = min(gain * distance, maximum_speed)
    return error * (speed / distance), False
