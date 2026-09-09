"""Tests for ROS trajectory validation and interpolation."""

from builtin_interfaces.msg import Duration
import numpy as np
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from massage_mujoco.model import JOINT_NAMES
from massage_mujoco.runtime import TrajectoryPoint
from massage_mujoco.trajectory import prepare_trajectory, sample_trajectory


def _point(seconds, positions):
    return JointTrajectoryPoint(
        positions=positions,
        time_from_start=Duration(sec=seconds),
    )


def test_prepare_trajectory_reorders_joint_values():
    names = list(reversed(JOINT_NAMES))
    trajectory = JointTrajectory(
        joint_names=names,
        points=[_point(1, list(range(6)))],
    )
    prepared = prepare_trajectory(trajectory)
    assert prepared[0].positions == [5.0, 4.0, 3.0, 2.0, 1.0, 0.0]


def test_prepare_trajectory_rejects_invalid_names():
    with pytest.raises(ValueError, match="all six joints"):
        prepare_trajectory(JointTrajectory(joint_names=["joint_1"], points=[]))


def test_prepare_trajectory_folds_identical_points_at_equal_times():
    positions = np.arange(6, dtype=float)
    trajectory = JointTrajectory(
        joint_names=list(JOINT_NAMES),
        points=[_point(1, positions), _point(1, positions), _point(2, positions + 1)],
    )

    prepared = prepare_trajectory(trajectory)

    assert [point.time_from_start for point in prepared] == [1.0, 2.0]
    np.testing.assert_allclose(prepared[0].positions, positions)
    np.testing.assert_allclose(prepared[1].positions, positions + 1)


def test_prepare_trajectory_rejects_different_positions_at_equal_times():
    trajectory = JointTrajectory(
        joint_names=list(JOINT_NAMES),
        points=[_point(1, np.zeros(6)), _point(1, np.ones(6))],
    )
    with pytest.raises(ValueError, match="identical positions"):
        prepare_trajectory(trajectory)


def test_prepare_trajectory_rejects_decreasing_times():
    trajectory = JointTrajectory(
        joint_names=list(JOINT_NAMES),
        points=[_point(2, np.zeros(6)), _point(1, np.zeros(6))],
    )
    with pytest.raises(ValueError, match="nondecreasing"):
        prepare_trajectory(trajectory)


def test_sample_trajectory_interpolates_all_segments():
    start = np.zeros(6)
    points = [
        TrajectoryPoint(1.0, np.ones(6)),
        TrajectoryPoint(3.0, np.full(6, 3.0)),
    ]
    np.testing.assert_allclose(sample_trajectory(start, points, 0.5), 0.5)
    np.testing.assert_allclose(sample_trajectory(start, points, 2.0), 2.0)
    np.testing.assert_allclose(sample_trajectory(start, points, 4.0), 3.0)
