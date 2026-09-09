"""Tests for teaching, TCP diagnostics, and force-guided surface motion."""

import numpy as np
import pytest

from massage_mujoco.features import (
    SurfaceFollower,
    TeachingTrajectory,
    TcpDiagnosticTracker,
)
from massage_mujoco.model import JOINT_NAMES


def test_teaching_points_are_timed_by_largest_joint_change(tmp_path):
    teaching = TeachingTrajectory(6)
    teaching.record([0.0, 0.2, 0.0, 0.0, 0.0, 0.0])
    teaching.record([0.0, 0.6, 0.0, 0.0, 0.0, 0.0])

    points = teaching.timed_points(np.zeros(6), 0.4, 0.2)

    assert [point.time_from_start for point in points] == pytest.approx([0.5, 1.5])
    destination = tmp_path / "teach.yaml"
    teaching.save(destination, JOINT_NAMES)
    loaded = TeachingTrajectory(6)
    assert loaded.load(destination, JOINT_NAMES) == 2
    np.testing.assert_allclose(loaded.waypoints, teaching.waypoints)


def test_teaching_rejects_duplicate_and_wrong_joint_order(tmp_path):
    teaching = TeachingTrajectory(6)
    teaching.record(np.zeros(6))
    with pytest.raises(ValueError, match="same as"):
        teaching.record(np.zeros(6))
    destination = tmp_path / "teach.yaml"
    teaching.save(destination, JOINT_NAMES)
    with pytest.raises(ValueError, match="joint order"):
        TeachingTrajectory(6).load(destination, tuple(reversed(JOINT_NAMES)))


def test_tcp_diagnostic_velocity_uses_simulation_time():
    tracker = TcpDiagnosticTracker()
    first = tracker.update(1.0, [0.0, 0.0, 0.0], [0.1, 0.0, 0.0])
    second = tracker.update(1.02, [0.002, 0.0, 0.0], [0.1, 0.0, 0.0])

    np.testing.assert_array_equal(first.actual_velocity, 0.0)
    assert second.actual_speed == pytest.approx(0.1)
    assert second.target_error == pytest.approx(0.098)


def _surface_follower(**overrides):
    parameters = dict(
        approach_direction=[1.0, 0.0, 0.0],
        tangent_velocity=[0.0, 0.02, 0.0],
        target_force=2.0,
        force_gain=0.01,
        approach_speed=0.01,
        maximum_normal_speed=0.02,
        contact_force=0.05,
        maximum_force=20.0,
        duration=1.0,
        approach_timeout=2.0,
        force_tolerance=0.1,
    )
    parameters.update(overrides)
    return SurfaceFollower(**parameters)


def test_surface_follow_runs_approach_force_control_and_tangent_motion():
    follower = _surface_follower()
    follower.reset(10.0)

    approach = follower.update(10.1, 0.0)
    np.testing.assert_allclose(approach.velocity, [0.01, 0.0, 0.0])
    assert approach.state == "APPROACH"

    establish = follower.update(10.2, 1.0, [1.0, 0.0, 0.0])
    np.testing.assert_allclose(establish.velocity, [0.01, 0.0, 0.0])
    assert establish.state == "ESTABLISH_FORCE"

    following = follower.update(10.3, 1.95, [1.0, 0.0, 0.0])
    np.testing.assert_allclose(following.velocity, [0.0005, 0.02, 0.0])
    assert following.state == "FOLLOW"

    complete = follower.update(11.31, 2.0, [1.0, 0.0, 0.0])
    assert complete.finished and complete.state == "COMPLETE"
    np.testing.assert_array_equal(complete.velocity, 0.0)


def test_surface_follow_fails_when_contact_is_not_reached():
    follower = _surface_follower(approach_timeout=0.5)
    follower.reset(0.0)
    result = follower.update(0.5, 0.0)
    assert result.finished and result.state == "FAULT"
    assert "timed out" in result.fault


def test_surface_follow_pauses_tangent_motion_after_contact_loss():
    follower = _surface_follower()
    follower.reset(0.0)
    follower.update(0.1, 0.1, [1.0, 0.0, 0.0])
    following = follower.update(0.2, 2.0, [1.0, 0.0, 0.0])
    assert following.state == "FOLLOW"

    lost = follower.update(0.3, 0.0)

    assert lost.state == "ESTABLISH_FORCE"
    assert lost.velocity[1] == 0.0
    assert lost.velocity[0] > 0.0


def test_surface_follow_stops_at_maximum_force():
    follower = _surface_follower()
    result = follower.update(0.1, 20.0)
    assert result.finished and result.state == "FAULT"
    assert "maximum" in result.fault
