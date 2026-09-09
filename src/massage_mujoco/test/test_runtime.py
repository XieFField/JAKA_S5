"""Contract tests for the headless MuJoCo runtime."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from massage_mujoco.model import JOINT_NAMES
from massage_mujoco.runtime import ControlMode, MujocoRuntime, TrajectoryPoint


EXPECTED_RANGES = np.array([
    [-6.28, 6.28],
    [-1.48, 4.62],
    [-3.05, 3.05],
    [-1.48, 4.62],
    [-6.28, 6.28],
    [-6.28, 6.28],
])

PRONE_INITIAL_POSITIONS = (
    Path(__file__).parents[2]
    / "massage_bringup/config/prone_back_initial_positions.yaml"
)


def test_model_preserves_joint_names_order_and_limits():
    runtime = MujocoRuntime()
    assert runtime.model.nq == 7
    assert runtime.model.nv == 7
    assert runtime.model.nu == 12
    assert [runtime.model.joint(index).name for index in range(6)] == list(JOINT_NAMES)
    assert [runtime.model.actuator(index).name for index in range(6, 12)] == [
        f"{name}_torque_Nm" for name in JOINT_NAMES
    ]
    np.testing.assert_allclose(runtime.model.jnt_range[:6], EXPECTED_RANGES)


def test_reset_is_deterministic_and_returns_detached_state():
    runtime = MujocoRuntime()
    initial = [0.1, 1.2, -1.1, 1.0, 0.8, -0.2]
    first = runtime.reset(initial)
    runtime.step(20)
    second = runtime.reset(initial)
    np.testing.assert_array_equal(first.position, second.position)
    np.testing.assert_array_equal(first.velocity, second.velocity)
    np.testing.assert_array_equal(first.target_position, second.target_position)
    np.testing.assert_array_equal(first.commanded_torque, second.commanded_torque)
    np.testing.assert_array_equal(first.torque_controlled, second.torque_controlled)
    np.testing.assert_array_equal(first.actuator_effort, second.actuator_effort)
    np.testing.assert_array_equal(first.tool_position, second.tool_position)
    first.position[0] = 99.0
    assert runtime.state().position[0] != 99.0


def test_tcp_control_ball_starts_at_tool_tip_and_never_collides():
    runtime = MujocoRuntime()
    np.testing.assert_allclose(
        runtime.tcp_control_target(),
        runtime.state().tool_position,
    )
    body_id = runtime.tcp_control_body_id
    geom_ids = np.flatnonzero(runtime.model.geom_bodyid == body_id)
    assert len(geom_ids) == 4
    np.testing.assert_array_equal(runtime.model.geom_contype[geom_ids], 0)
    np.testing.assert_array_equal(runtime.model.geom_conaffinity[geom_ids], 0)


def test_contact_pad_distance_distinguishes_default_and_precontact_pose():
    runtime = MujocoRuntime()
    assert runtime.distance_to_contact_pad(runtime.state().tool_position) > 1.0
    runtime.reset([
        -0.0150542703,
        0.439740469,
        -0.667101116,
        0.227360592,
        -0.0150542703,
        0.0000009447,
    ])
    assert runtime.distance_to_contact_pad(runtime.state().tool_position) == (
        pytest.approx(0.005)
    )


def test_prone_mannequin_aligns_stable_back_below_downward_tool():
    runtime = MujocoRuntime(
        use_prone_mannequin=True,
        initial_positions_path=PRONE_INITIAL_POSITIONS,
    )
    state = runtime.state()

    assert runtime.model.nflex == 1
    assert runtime.contact_target_name == "prone_back_soft_tissue"
    np.testing.assert_allclose(state.tool_position, [0.65, -0.10, 0.50], atol=1e-8)
    np.testing.assert_allclose(state.tool_rotation[:, 2], [0.0, 0.0, -1.0], atol=1e-7)
    assert runtime.distance_to_contact_pad(state.tool_position) == pytest.approx(0.042)
    assert runtime.data.ncon == 0

    runtime.step(2000)
    settled = runtime.state()
    np.testing.assert_allclose(settled.tool_position, state.tool_position, atol=1e-8)
    assert runtime.distance_to_contact_pad(settled.tool_position) == pytest.approx(0.042)
    assert np.all(np.isfinite(runtime.data.qpos))
    assert np.all(np.isfinite(runtime.data.qvel))


def test_reset_returns_tcp_control_ball_to_tool_tip():
    runtime = MujocoRuntime()
    runtime.set_tcp_control_target([0.8, -0.4, 1.2])

    state = runtime.reset()

    np.testing.assert_allclose(runtime.tcp_control_target(), state.tool_position)


def test_state_reports_servo_target_and_finite_actuator_effort():
    runtime = MujocoRuntime()
    target = runtime.state().position + np.array([
        0.05,
        -0.05,
        0.05,
        -0.05,
        -0.05,
        0.05,
    ])
    runtime.set_target(target)
    state = runtime.step()

    np.testing.assert_allclose(state.target_position, target)
    assert state.actuator_effort.shape == (len(JOINT_NAMES),)
    assert np.all(np.isfinite(state.actuator_effort))


def test_default_reset_uses_project_simulation_initial_positions():
    runtime = MujocoRuntime()
    np.testing.assert_allclose(
        runtime.state().position,
        [
            -np.pi,
            np.pi / 2,
            -np.pi / 2,
            np.pi / 2,
            np.pi / 2,
            -5 * np.pi / 12,
        ],
    )


def test_native_mujoco_reset_restores_project_initial_pose_and_hold_target():
    runtime = MujocoRuntime()
    expected = runtime.state().position
    runtime.move_to(expected + [0.1, -0.1, 0.1, -0.1, -0.1, 0.1], 0.5)

    mujoco.mj_resetData(runtime.model, runtime.data)

    assert not np.allclose(runtime.state().position, expected)
    assert runtime.recover_native_reset()
    np.testing.assert_allclose(runtime.state().position, expected)
    np.testing.assert_allclose(runtime.data.ctrl[:6], expected)
    state = runtime.step(100)
    np.testing.assert_allclose(state.position, expected, atol=1e-10)


def test_rejects_invalid_joint_targets():
    runtime = MujocoRuntime()
    with pytest.raises(ValueError, match="six joint positions"):
        runtime.set_target([0.0])
    with pytest.raises(ValueError, match="outside its URDF limits"):
        runtime.set_target([0.0, 5.0, 0.0, 0.0, 0.0, 0.0])


def test_position_servo_reaches_smoke_target():
    runtime = MujocoRuntime()
    target = runtime.state().position + np.array([
        0.15,
        -0.12,
        0.12,
        -0.12,
        -0.12,
        0.15,
    ])
    result = runtime.move_to(target, duration=1.0, settle_duration=1.0)
    assert np.max(np.abs(result.position - target)) < 0.01


def test_servo_velocity_stream_integrates_past_feedback_relative_position_leads():
    runtime = MujocoRuntime()
    initial = runtime.state().position
    velocity = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])

    for _ in range(30):
        # MoveIt Servo sends a one-period position lead based on current
        # feedback. Treating this as an absolute actuator target caused the
        # target to remain about 0.01 rad ahead and throttled actual motion.
        feedback_relative_target = runtime.state().position + velocity * 0.01
        runtime.set_servo_target(
            feedback_relative_target,
            velocity,
            timeout=0.2,
        )
        runtime.step(10)

    state = runtime.state()
    assert state.target_position[1] - initial[1] == pytest.approx(0.30)
    assert state.velocity[1] > 0.8
    np.testing.assert_allclose(state.target_velocity, velocity)

    runtime.stop_servo()
    stopped = runtime.state()
    np.testing.assert_allclose(stopped.target_position, stopped.position)
    np.testing.assert_array_equal(stopped.target_velocity, 0.0)


def test_servo_velocity_stream_stops_on_simulation_time_timeout():
    runtime = MujocoRuntime()
    velocity = np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.0])
    runtime.set_servo_target(
        runtime.state().position + velocity * 0.01,
        velocity,
        timeout=0.02,
    )

    runtime.step(21)
    target_at_timeout = runtime.state().target_position
    runtime.step(5)

    state = runtime.state()
    np.testing.assert_array_equal(state.target_velocity, 0.0)
    np.testing.assert_array_equal(state.target_position, target_at_timeout)


def test_native_torque_control_releases_only_selected_joint_and_keeps_compensation():
    runtime = MujocoRuntime()
    runtime.set_native_torque_control_limit(50.0)
    initial = runtime.state().position
    changed = np.array([False, True, False, False, False, False])

    runtime.accept_native_torque_controls(
        [0.0, 20.0, 0.0, 0.0, 0.0, 0.0],
        changed,
    )
    moving = runtime.step(100)

    assert runtime.model.actuator_gainprm[1, 0] == 0.0
    assert np.all(runtime.model.actuator_gainprm[[0, 2, 3, 4, 5], 0] > 0.0)
    assert moving.position[1] > initial[1]
    assert moving.velocity[1] > 0.0
    np.testing.assert_array_equal(
        moving.commanded_torque,
        [0.0, 20.0, 0.0, 0.0, 0.0, 0.0],
    )
    np.testing.assert_array_equal(moving.torque_controlled, changed)

    runtime.accept_native_position_targets(moving.position, changed)
    restored = runtime.state()
    np.testing.assert_array_equal(restored.commanded_torque, 0.0)
    np.testing.assert_array_equal(restored.torque_controlled, False)
    assert np.all(runtime.model.actuator_gainprm[:6, 0] > 0.0)

    with pytest.raises(ValueError, match="native control limit"):
        runtime.accept_native_torque_controls(
            [0.0, 51.0, 0.0, 0.0, 0.0, 0.0],
            changed,
        )


def test_native_torque_control_accepts_independent_joint_limits():
    runtime = MujocoRuntime()
    limits = np.array([5.0, 13.37, 3.510, 0.1158, 0.02179, 0.0003014])

    runtime.set_native_torque_control_limits(limits)

    np.testing.assert_allclose(
        runtime.model.actuator_ctrlrange[runtime._torque_actuator_ids, 0],
        -limits,
    )
    np.testing.assert_allclose(
        runtime.model.actuator_ctrlrange[runtime._torque_actuator_ids, 1],
        limits,
    )
    with pytest.raises(ValueError, match="six positive values"):
        runtime.set_native_torque_control_limits([5.0] * 5)


def test_inertia_scaled_torque_limits_match_joint_1_position_response():
    runtime = MujocoRuntime()
    limits = np.array([5.0, 13.37, 3.510, 0.1158, 0.02179, 0.0003014])
    runtime.set_native_torque_control_limits(limits)
    initial = runtime.state().position.copy()

    for direction in (-1.0, 1.0):
        displacements = []
        for index, limit in enumerate(limits):
            runtime.reset(initial)
            torques = np.zeros(6)
            changed = np.zeros(6, dtype=bool)
            torques[index] = direction * limit
            changed[index] = True
            runtime.accept_native_torque_controls(torques, changed)
            state = runtime.step(250)
            displacements.append(state.position[index] - initial[index])

        np.testing.assert_allclose(
            displacements,
            np.full(6, displacements[0]),
            rtol=0.005,
            atol=1e-9,
        )


def test_zero_native_torque_joint_remains_stationary_with_bias_compensation():
    runtime = MujocoRuntime()
    initial = runtime.state().position
    changed = np.array([False, True, False, False, False, False])
    runtime.accept_native_torque_controls(np.zeros(6), changed)

    state = runtime.step(200)

    np.testing.assert_allclose(state.position, initial, atol=1e-12)
    np.testing.assert_allclose(state.velocity, 0.0, atol=1e-12)


def test_mouse_rotation_projects_to_selected_joint_and_holds_on_release():
    runtime = MujocoRuntime()
    joint_index = 1
    body_id = int(runtime._joint_body_ids[joint_index])
    runtime.begin_mouse_joint_control(joint_index)
    runtime.data.xfrc_applied[body_id, 3:] = (
        runtime.data.xaxis[joint_index] * 1e6
    )

    torque = runtime.update_mouse_joint_torque(body_id, maximum_torque=50.0)
    moving = runtime.step(20)

    assert torque == pytest.approx(50.0)
    np.testing.assert_array_equal(runtime.data.xfrc_applied[body_id], 0.0)
    assert moving.velocity[joint_index] > 0.0
    assert runtime.joint_index_for_body(runtime.model.body("massage_head_link").id) == 5

    runtime.end_mouse_joint_control()
    held = runtime.state()
    assert held.target_position[joint_index] == held.position[joint_index]
    assert not np.any(held.torque_controlled)


def test_hold_mode_captures_current_position_and_rejects_new_targets():
    runtime = MujocoRuntime()
    runtime.set_target(runtime.state().position + [
        0.2,
        -0.1,
        0.1,
        -0.1,
        -0.1,
        0.2,
    ])
    moving = runtime.step(50)
    held = runtime.set_control_mode(ControlMode.HOLD)

    assert runtime.control_mode is ControlMode.HOLD
    np.testing.assert_allclose(held.target_position, moving.position)
    with pytest.raises(RuntimeError, match="POSITION"):
        runtime.set_target(np.zeros(6))


def test_gravity_mode_disables_and_position_mode_restores_servos():
    runtime = MujocoRuntime()
    nominal_gain = runtime.model.actuator_gainprm[:6].copy()
    nominal_bias = runtime.model.actuator_biasprm[:6].copy()

    runtime.set_control_mode(ControlMode.GRAVITY)
    assert runtime.control_mode is ControlMode.GRAVITY
    np.testing.assert_array_equal(runtime.model.actuator_gainprm[:6], 0.0)
    np.testing.assert_array_equal(runtime.model.actuator_biasprm[:6], 0.0)
    gravity_state = runtime.step()
    assert np.all(np.isfinite(gravity_state.position))

    position_state = runtime.set_control_mode(ControlMode.POSITION)
    assert runtime.control_mode is ControlMode.POSITION
    np.testing.assert_array_equal(runtime.model.actuator_gainprm[:6], nominal_gain)
    np.testing.assert_array_equal(runtime.model.actuator_biasprm[:6], nominal_bias)
    np.testing.assert_allclose(
        position_state.target_position,
        position_state.position,
    )


def test_bias_compensation_is_refreshed_before_every_physics_step(monkeypatch):
    runtime = MujocoRuntime()
    recorded = []
    original_step = mujoco.mj_step

    def record_step(model, data):
        recorded.append((data.qfrc_applied[:6].copy(), data.qfrc_bias[:6].copy()))
        original_step(model, data)

    monkeypatch.setattr(mujoco, "mj_step", record_step)
    for mode in (ControlMode.POSITION, ControlMode.HOLD, ControlMode.GRAVITY):
        runtime.set_control_mode(mode)
        runtime.step(3)

    assert len(recorded) == 9
    for applied, bias in recorded:
        np.testing.assert_allclose(applied, bias)


def test_executes_multiple_time_ordered_trajectory_points():
    runtime = MujocoRuntime()
    points = [
        TrajectoryPoint(0.5, [0.1, 0.7, -0.8, 0.6, 0.5, -0.1]),
        TrajectoryPoint(1.0, [0.2, 1.0, -1.1, 0.8, 0.7, -0.2]),
    ]
    result = runtime.execute_trajectory(points, settle_duration=1.0)
    np.testing.assert_allclose(result.position, points[-1].positions, atol=0.01)


def test_rejects_empty_or_non_monotonic_trajectory():
    runtime = MujocoRuntime()
    with pytest.raises(ValueError, match="at least one point"):
        runtime.execute_trajectory([])
    with pytest.raises(ValueError, match="strictly increasing"):
        runtime.execute_trajectory([
            TrajectoryPoint(0.5, np.zeros(6)),
            TrajectoryPoint(0.5, np.zeros(6)),
        ])
