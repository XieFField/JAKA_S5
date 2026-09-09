"""Check actual pad contacts survive ROS serialization and event publication."""

import time

from massage_msgs.msg import ContactState
from massage_mujoco.ros_node import MujocoNode
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def test_contact_points_and_start_event_reach_ros_subscriber():
    context = Context()
    rclpy.init(context=context)
    server = MujocoNode(context=context)
    client = Node('contact_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client)
    messages = []
    subscription = client.create_subscription(
        ContactState, '/massage_mujoco/contact_events', messages.append, 10)
    try:
        deadline = time.monotonic() + 3
        while server._contact_event_publisher.get_subscription_count() < 1:
            assert time.monotonic() < deadline
            time.sleep(0.01)
        runtime = server._runtime
        runtime.reset([-0.0224084984, 0.442422977, -0.672689985,
                       0.230266926, -0.0224084984, 0.0000009178])
        runtime.step(3000)
        server._publish_state()
        deadline = time.monotonic() + 3
        while not messages:
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.1)
        message = messages[-1]
        assert 'CONTACT_STARTED' in message.events
        assert message.in_contact and message.normal_force > 0.05
        assert message.contacts and all(point.massage_pair for point in message.contacts)
        assert message.header.frame_id == 'world'
        assert abs(sum(p.normal_force for p in message.contacts) - message.normal_force) < 1e-9
    finally:
        executor.shutdown()
        client.destroy_subscription(subscription)
        client.destroy_node()
        server.destroy_node()
        rclpy.shutdown(context=context)
