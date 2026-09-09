"""Integration tests for the MuJoCo FollowJointTrajectory action server."""

from dataclasses import dataclass
import math
import threading
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import TwistStamped, WrenchStamped
import mujoco
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from massage_mujoco.model import JOINT_NAMES
from massage_mujoco.ros_node import (
    MujocoNode,
    _joint_mouse_perturbation_active,
)
from massage_msgs.msg import ContactState


@dataclass
class _ActionHarness:
    server: MujocoNode
    client: ActionClient
    wrench_messages: list
    wrench_received: threading.Event
    joint_state_messages: list
    joint_state_received: threading.Event
    controller_state_messages: list
    controller_state_received: threading.Event
    mode_messages: list
    mode_received: threading.Event
    mode_clients: dict
    servo_publisher: object
    owner_messages: list
    owner_received: threading.Event
    contact_messages: list
    tcp_commands: list
    tcp_command_received: threading.Event
    clock_messages: list
    clock_received: threading.Event

    def send_goal(self, positions, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        seconds = int(duration)
        nanoseconds = round((duration - seconds) * 1e9)
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[float(value) for value in positions],
                time_from_start=Duration(sec=seconds, nanosec=nanoseconds),
            )
        ]
        return _wait_for_future(self.client.send_goal_async(goal))


def _wait_for_future(future, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            raise TimeoutError("ROS future did not complete before the deadline")
        time.sleep(0.01)
    exception = future.exception()
    if exception is not None:
        raise exception
    return future.result()


@pytest.mark.parametrize(
    "perturbation",
    [
        mujoco.mjtPertBit.mjPERT_ROTATE,
        mujoco.mjtPertBit.mjPERT_TRANSLATE,
        int(mujoco.mjtPertBit.mjPERT_ROTATE)
        | int(mujoco.mjtPertBit.mjPERT_TRANSLATE),
    ],
)
def test_joint_mouse_control_accepts_rotate_and_right_drag(perturbation):
    assert _joint_mouse_perturbation_active(perturbation)


def test_joint_mouse_control_ignores_inactive_perturbation():
    assert not _joint_mouse_perturbation_active(0)


@pytest.fixture(scope="module")
def action_harness():
    rclpy.init()
    server = MujocoNode(parameter_overrides=[
        Parameter("realtime_factor", value=10.0),
    ])
    client_node = Node("massage_mujoco_action_test")
    client = ActionClient(
        client_node,
        FollowJointTrajectory,
        "/jaka_s5_controller/follow_joint_trajectory",
    )
    wrench_messages = []
    wrench_received = threading.Event()
    joint_state_messages = []
    joint_state_received = threading.Event()
    controller_state_messages = []
    controller_state_received = threading.Event()
    mode_messages = []
    mode_received = threading.Event()
    owner_messages = []
    owner_received = threading.Event()
    contact_messages = []
    tcp_commands = []
    tcp_command_received = threading.Event()
    clock_messages = []
    clock_received = threading.Event()
    contact_subscription = client_node.create_subscription(
        ContactState, '/massage_mujoco/contact_state', contact_messages.append, 10,
    )
    tcp_command_subscription = client_node.create_subscription(
        TwistStamped,
        "/massage/cartesian_jog/command",
        lambda message: (
            tcp_commands.append(message),
            tcp_command_received.set(),
        ),
        10,
    )
    clock_subscription = client_node.create_subscription(
        Clock,
        "/clock",
        lambda message: (
            clock_messages.append(message),
            clock_received.set(),
        ),
        10,
    )

    def receive_wrench(message):
        wrench_messages.append(message)
        wrench_received.set()

    wrench_subscription = client_node.create_subscription(
        WrenchStamped,
        "/massage_ft_broadcaster/wrench",
        receive_wrench,
        10,
    )

    def receive_joint_state(message):
        joint_state_messages.append(message)
        joint_state_received.set()

    def receive_controller_state(message):
        controller_state_messages.append(message)
        controller_state_received.set()

    joint_state_subscription = client_node.create_subscription(
        JointState,
        "/joint_states",
        receive_joint_state,
        10,
    )
    controller_state_subscription = client_node.create_subscription(
        JointTrajectoryControllerState,
        "/jaka_s5_controller/controller_state",
        receive_controller_state,
        10,
    )
    state_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    mode_subscription = client_node.create_subscription(
        String,
        "/massage_mujoco/control_mode",
        lambda message: (mode_messages.append(message), mode_received.set()),
        state_qos,
    )
    owner_subscription = client_node.create_subscription(
        String,
        "/massage_mujoco/control_owner",
        lambda message: (owner_messages.append(message), owner_received.set()),
        state_qos,
    )
    servo_publisher = client_node.create_publisher(
        JointTrajectory,
        "/jaka_s5_controller/joint_trajectory",
        10,
    )
    mode_clients = {
        mode: client_node.create_client(
            Trigger,
            "/massage_mujoco/set_%s_mode" % mode,
        )
        for mode in ("position", "hold", "gravity")
    }
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    assert client.wait_for_server(timeout_sec=10.0)
    assert all(
        mode_client.wait_for_service(timeout_sec=10.0)
        for mode_client in mode_clients.values()
    )

    yield _ActionHarness(
        server,
        client,
        wrench_messages,
        wrench_received,
        joint_state_messages,
        joint_state_received,
        controller_state_messages,
        controller_state_received,
        mode_messages,
        mode_received,
        mode_clients,
        servo_publisher,
        owner_messages,
        owner_received,
        contact_messages,
        tcp_commands,
        tcp_command_received,
        clock_messages,
        clock_received,
    )

    executor.shutdown(timeout_sec=5.0)
    executor_thread.join(timeout=5.0)
    client.destroy()
    client_node.destroy_subscription(wrench_subscription)
    client_node.destroy_subscription(joint_state_subscription)
    client_node.destroy_subscription(controller_state_subscription)
    client_node.destroy_subscription(mode_subscription)
    client_node.destroy_subscription(owner_subscription)
    client_node.destroy_subscription(contact_subscription)
    client_node.destroy_subscription(tcp_command_subscription)
    client_node.destroy_subscription(clock_subscription)
    client_node.destroy_publisher(servo_publisher)
    client_node.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


def _cancel_and_wait(goal_handle):
    cancel_response = _wait_for_future(goal_handle.cancel_goal_async())
    assert cancel_response.goals_canceling
    return _wait_for_future(goal_handle.get_result_async())


def _set_mode(action_harness, mode):
    action_harness.mode_received.clear()
    response = _wait_for_future(
        action_harness.mode_clients[mode].call_async(Trigger.Request())
    )
    assert response.success
    assert response.message == "control mode set to %s" % mode.upper()
    assert action_harness.mode_received.wait(timeout=2.0)
    assert action_harness.mode_messages[-1].data == mode.upper()


def _wait_for_reference(action_harness, expected, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        action_harness.controller_state_received.clear()
        if not action_harness.controller_state_received.wait(timeout=0.2):
            continue
        reference = np.asarray(
            action_harness.controller_state_messages[-1].reference.positions
        )
        if np.allclose(reference, expected):
            return reference
    raise TimeoutError("controller reference did not reach the expected value")


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def test_wrench_topic_uses_sensor_frame_and_six_finite_values(action_harness):
    assert action_harness.wrench_received.wait(timeout=2.0)
    message = action_harness.wrench_messages[-1]
    values = [
        message.wrench.force.x,
        message.wrench.force.y,
        message.wrench.force.z,
        message.wrench.torque.x,
        message.wrench.torque.y,
        message.wrench.torque.z,
    ]
    assert message.header.frame_id == "massage_head_link"
    assert all(math.isfinite(value) for value in values)


def test_joint_and_controller_state_publish_standard_telemetry(action_harness):
    assert action_harness.joint_state_received.wait(timeout=2.0)
    assert action_harness.controller_state_received.wait(timeout=2.0)
    joint_state = action_harness.joint_state_messages[-1]
    controller_state = action_harness.controller_state_messages[-1]

    assert joint_state.name == list(JOINT_NAMES)
    assert len(joint_state.effort) == len(JOINT_NAMES)
    assert all(math.isfinite(value) for value in joint_state.effort)
    assert controller_state.joint_names == list(JOINT_NAMES)
    np.testing.assert_allclose(
        controller_state.reference.positions,
        controller_state.desired.positions,
    )
    np.testing.assert_allclose(
        controller_state.feedback.positions,
        controller_state.actual.positions,
    )
    np.testing.assert_allclose(
        controller_state.error.positions,
        np.asarray(controller_state.reference.positions)
        - np.asarray(controller_state.feedback.positions),
    )
    np.testing.assert_allclose(
        controller_state.output.effort,
        controller_state.feedback.effort,
    )


def test_contact_state_publishes_world_frame_and_zero_free_space_force(action_harness):
    deadline = time.monotonic() + 2.0
    while not action_harness.contact_messages and time.monotonic() < deadline:
        time.sleep(0.01)
    message = action_harness.contact_messages[-1]
    assert message.header.frame_id == 'world'
    assert message.header.stamp.sec > 0 or message.header.stamp.nanosec > 0
    assert not message.in_contact
    assert not message.unexpected_contact
    assert message.normal_force == 0.0


def test_controller_reference_changes_during_trajectory(action_harness):
    initial_reference = np.asarray(
        action_harness.controller_state_messages[-1].reference.positions
    )
    target = initial_reference + np.array([
        0.2,
        -0.15,
        0.15,
        -0.15,
        -0.15,
        0.2,
    ])
    action_harness.controller_state_received.clear()
    goal_handle = action_harness.send_goal(target, duration=1.0)
    assert goal_handle.accepted

    deadline = time.monotonic() + 2.0
    changed = False
    while time.monotonic() < deadline:
        assert action_harness.controller_state_received.wait(timeout=0.2)
        reference = np.asarray(
            action_harness.controller_state_messages[-1].reference.positions
        )
        if not np.allclose(reference, initial_reference):
            changed = True
            break
        action_harness.controller_state_received.clear()

    assert changed
    wrapped_result = _wait_for_future(goal_handle.get_result_async())
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED


def test_control_modes_interrupt_and_gate_trajectories(action_harness):
    long_goal = action_harness.send_goal(
        [-2.8, 1.3, -1.3, 1.3, 1.3, -1.0],
        duration=5.0,
    )
    assert long_goal.accepted

    _set_mode(action_harness, "hold")
    interrupted = _wait_for_future(long_goal.get_result_async())
    assert interrupted.status == GoalStatus.STATUS_ABORTED
    assert (
        "switch to HOLD" in interrupted.result.error_string
        or "interrupted before execution by control mode switch"
        in interrupted.result.error_string
    )
    rejected = action_harness.send_goal(
        [-3.0, 1.4, -1.4, 1.4, 1.4, -1.1],
    )
    assert not rejected.accepted

    _set_mode(action_harness, "gravity")
    rejected = action_harness.send_goal(
        [-3.0, 1.4, -1.4, 1.4, 1.4, -1.1],
    )
    assert not rejected.accepted

    _set_mode(action_harness, "position")
    target = action_harness.controller_state_messages[-1].feedback.positions
    accepted = action_harness.send_goal(target, duration=0.2)
    assert accepted.accepted
    completed = _wait_for_future(accepted.get_result_async())
    assert completed.status == GoalStatus.STATUS_SUCCEEDED


def test_servo_stream_owns_controller_until_command_timeout(action_harness):
    _set_mode(action_harness, "position")
    initial = np.asarray(
        action_harness.controller_state_messages[-1].feedback.positions
    )
    target = initial + np.array([0.02, -0.02, 0.02, -0.02, -0.02, 0.02])
    command = JointTrajectory(
        joint_names=list(JOINT_NAMES),
        points=[JointTrajectoryPoint(positions=target.tolist())],
    )
    action_harness.owner_received.clear()
    action_harness.controller_state_received.clear()
    action_harness.servo_publisher.publish(command)

    assert action_harness.owner_received.wait(timeout=2.0)
    assert action_harness.owner_messages[-1].data == "MOVEIT_SERVO"
    _wait_for_reference(action_harness, target)
    rejected = action_harness.send_goal(target, duration=0.2)
    assert not rejected.accepted

    action_harness.owner_received.clear()
    assert action_harness.owner_received.wait(timeout=2.0)
    assert action_harness.owner_messages[-1].data == "IDLE"
    current = action_harness.controller_state_messages[-1].feedback.positions
    accepted = action_harness.send_goal(current, duration=0.2)
    assert accepted.accepted
    completed = _wait_for_future(accepted.get_result_async())
    assert completed.status == GoalStatus.STATUS_SUCCEEDED


def test_servo_stream_is_ignored_outside_position_mode(action_harness):
    _set_mode(action_harness, "hold")
    # The latched mode update can arrive before the first controller state
    # carrying HOLD's newly captured reference.
    time.sleep(0.05)
    action_harness.controller_state_received.clear()
    assert action_harness.controller_state_received.wait(timeout=2.0)
    before = np.asarray(
        action_harness.controller_state_messages[-1].reference.positions
    )
    command = JointTrajectory(
        joint_names=list(JOINT_NAMES),
        points=[JointTrajectoryPoint(positions=(before + 0.01).tolist())],
    )
    action_harness.servo_publisher.publish(command)
    time.sleep(0.1)
    after = np.asarray(
        action_harness.controller_state_messages[-1].reference.positions
    )
    np.testing.assert_allclose(after, before)
    _set_mode(action_harness, "position")


def test_native_joint_slider_takes_control(action_harness):
    _set_mode(action_harness, "position")
    server = action_harness.server
    with server._lock:
        target = server._runtime.target_positions()
        target[0] += 0.5
        server._runtime.set_target(target)

    start = len(action_harness.owner_messages)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        owners = [message.data for message in action_harness.owner_messages[start:]]
        if "MUJOCO_JOINT_CONTROL" in owners:
            break
        time.sleep(0.01)
    assert "MUJOCO_JOINT_CONTROL" in owners
    _set_mode(action_harness, "hold")
    _set_mode(action_harness, "position")


def test_native_torque_slider_releases_joint_until_position_slider_moves(
    action_harness,
):
    _set_mode(action_harness, "position")
    server = action_harness.server
    np.testing.assert_allclose(
        server._runtime.model.actuator_ctrlrange[
            server._runtime._torque_actuator_ids
        ],
        [
            [-5.0, 5.0],
            [-13.37, 13.37],
            [-3.510, 3.510],
            [-0.1158, 0.1158],
            [-0.02179, 0.02179],
            [-0.0003014, 0.0003014],
        ],
    )
    assert server._joint_mouse_control_maximum_torque == 50.0
    with server._lock:
        server._runtime.data.ctrl[server._runtime._torque_actuator_ids[1]] = 2.0

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if (
            server._runtime.torque_controlled[1]
            and action_harness.controller_state_messages[-1].reference.effort[1]
            == 2.0
        ):
            break
        time.sleep(0.01)
    assert server._runtime.torque_controlled[1]
    assert action_harness.owner_messages[-1].data == "MUJOCO_JOINT_CONTROL"
    assert action_harness.controller_state_messages[-1].reference.effort[1] == 2.0

    with server._lock:
        target = server._runtime.target_positions()
        target[1] = server._runtime.state().position[1] + 0.01
        server._runtime.data.ctrl[server._runtime._position_actuator_ids[1]] = target[1]
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if not server._runtime.torque_controlled[1]:
            break
        time.sleep(0.01)
    assert not server._runtime.torque_controlled[1]
    assert server._runtime.torque_controls()[1] == 0.0
    _set_mode(action_harness, "hold")
    _set_mode(action_harness, "position")


def test_tcp_control_ball_publishes_bounded_world_velocity(action_harness):
    _set_mode(action_harness, "position")
    server = action_harness.server
    with server._lock:
        current = server._runtime.state().tool_position
        server._runtime.set_tcp_control_target(current + [0.2, 0.0, 0.0])
    action_harness.tcp_command_received.clear()

    assert action_harness.tcp_command_received.wait(timeout=2.0)
    command = action_harness.tcp_commands[-1]
    assert command.header.frame_id == "world"
    assert command.twist.linear.x == pytest.approx(0.50)
    assert command.twist.linear.y == pytest.approx(0.0)
    assert command.twist.linear.z == pytest.approx(0.0)
    assert command.twist.angular.x == 0.0
    assert command.twist.angular.y == 0.0
    assert command.twist.angular.z == 0.0
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if any(
            message.data == "MUJOCO_TCP_CONTROL"
            for message in action_harness.owner_messages
        ):
            break
        time.sleep(0.01)
    assert any(
        message.data == "MUJOCO_TCP_CONTROL"
        for message in action_harness.owner_messages
    )
    _set_mode(action_harness, "hold")
    _set_mode(action_harness, "position")


def test_native_reset_keeps_clock_monotonic_and_drains_servo_output(
    action_harness,
):
    _set_mode(action_harness, "position")
    assert action_harness.clock_received.wait(timeout=2.0)
    before = _stamp_seconds(action_harness.clock_messages[-1].clock)
    server = action_harness.server
    with server._lock:
        mujoco.mj_resetData(server._runtime.model, server._runtime.data)

    deadline = time.monotonic() + 2.0
    after = before
    while time.monotonic() < deadline:
        after = _stamp_seconds(action_harness.clock_messages[-1].clock)
        if after > before:
            break
        time.sleep(0.01)
    assert after > before
    expected = np.asarray([
        server._runtime.contract.initial_positions[name]
        for name in JOINT_NAMES
    ])
    _wait_for_reference(action_harness, expected)

    stale_target = expected.copy()
    stale_target[0] += 0.2
    command = JointTrajectory(
        joint_names=list(JOINT_NAMES),
        points=[JointTrajectoryPoint(positions=stale_target.tolist())],
    )
    action_harness.servo_publisher.publish(command)
    time.sleep(0.05)
    reference = np.asarray(
        action_harness.controller_state_messages[-1].reference.positions
    )
    np.testing.assert_allclose(reference, expected)
    time.sleep(0.2)


def test_successful_trajectory(action_harness):
    _set_mode(action_harness, "position")
    goal_handle = action_harness.send_goal(
        [0.05, 1.5, -1.5, 1.5, 1.5, -0.05],
        duration=0.5,
    )
    assert goal_handle.accepted
    wrapped_result = _wait_for_future(goal_handle.get_result_async())
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
    assert (
        wrapped_result.result.error_code
        == FollowJointTrajectory.Result.SUCCESSFUL
    )


def test_invalid_target_is_rejected(action_harness):
    goal_handle = action_harness.send_goal(
        [0.0, 5.0, 0.0, 0.0, 0.0, 0.0],
    )
    assert not goal_handle.accepted


def test_active_trajectory_can_be_canceled(action_harness):
    _set_mode(action_harness, "position")
    goal_handle = action_harness.send_goal(
        [-0.5, 1.0, -1.0, 1.0, 1.0, 0.5],
        duration=5.0,
    )
    assert goal_handle.accepted
    wrapped_result = _cancel_and_wait(goal_handle)
    assert wrapped_result.status == GoalStatus.STATUS_CANCELED
    assert wrapped_result.result.error_string == "trajectory canceled"


def test_second_goal_is_rejected_while_controller_is_busy(action_harness):
    _set_mode(action_harness, "position")
    first_goal = action_harness.send_goal(
        [0.5, 1.2, -1.2, 1.2, 1.2, -0.5],
        duration=5.0,
    )
    assert first_goal.accepted

    second_goal = action_harness.send_goal(
        [0.0, 1.4, -1.4, 1.4, 1.4, 0.0],
        duration=1.0,
    )
    assert not second_goal.accepted

    wrapped_result = _cancel_and_wait(first_goal)
    assert wrapped_result.status == GoalStatus.STATUS_CANCELED
