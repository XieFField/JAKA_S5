"""ROS trajectory validation and interpolation without node side effects."""

from typing import List, Sequence

import numpy as np

from massage_mujoco.model import JOINT_NAMES
from massage_mujoco.runtime import TrajectoryPoint


def duration_seconds(duration) -> float:
    """Convert a builtin_interfaces Duration-like value to seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def prepare_trajectory(trajectory) -> List[TrajectoryPoint]:
    """Validate and reorder a ROS JointTrajectory into canonical joint order."""
    if len(trajectory.joint_names) != len(JOINT_NAMES):
        raise ValueError("trajectory must name all six joints")
    if len(set(trajectory.joint_names)) != len(trajectory.joint_names):
        raise ValueError("trajectory joint names must be unique")
    if set(trajectory.joint_names) != set(JOINT_NAMES):
        raise ValueError("trajectory joint names do not match the JAKA S5")
    if not trajectory.points:
        raise ValueError("trajectory must contain at least one point")

    source_indices = [trajectory.joint_names.index(name) for name in JOINT_NAMES]
    prepared = []
    previous_time = -1.0
    for point in trajectory.points:
        if len(point.positions) != len(JOINT_NAMES):
            raise ValueError("each trajectory point must contain six positions")
        time_from_start = duration_seconds(point.time_from_start)
        if not np.isfinite(time_from_start) or time_from_start < 0.0:
            raise ValueError("trajectory times must be finite and nonnegative")
        positions = [float(point.positions[index]) for index in source_indices]
        if not np.all(np.isfinite(positions)):
            raise ValueError("trajectory positions must be finite")
        if time_from_start < previous_time:
            raise ValueError("trajectory times must be nondecreasing")
        if time_from_start == previous_time:
            if positions != prepared[-1].positions:
                raise ValueError(
                    "equal trajectory times must have identical positions"
                )
            continue
        prepared.append(TrajectoryPoint(time_from_start, positions))
        previous_time = time_from_start
    return prepared


def sample_trajectory(
    start_positions: Sequence[float],
    points: Sequence[TrajectoryPoint],
    elapsed: float,
) -> np.ndarray:
    """Linearly interpolate a prepared trajectory at an absolute elapsed time."""
    start = np.asarray(start_positions, dtype=float)
    if elapsed <= points[0].time_from_start:
        if points[0].time_from_start == 0.0:
            return np.asarray(points[0].positions, dtype=float)
        alpha = max(0.0, elapsed) / points[0].time_from_start
        return start + alpha * (np.asarray(points[0].positions) - start)

    for previous, current in zip(points, points[1:]):
        if elapsed <= current.time_from_start:
            segment_time = current.time_from_start - previous.time_from_start
            alpha = (elapsed - previous.time_from_start) / segment_time
            previous_positions = np.asarray(previous.positions, dtype=float)
            return previous_positions + alpha * (
                np.asarray(current.positions, dtype=float) - previous_positions
            )
    return np.asarray(points[-1].positions, dtype=float)
