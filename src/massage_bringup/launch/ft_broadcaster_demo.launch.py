import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")
    controller_config = os.path.join(
        bringup_share,
        "config",
        "compliance_controllers.yaml",
    )

    ft_sensor_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "ft_sensor_sim.launch.py")
        ),
        launch_arguments={"use_massage_head": "true"}.items(),
    )

    # 等待 Gazebo 创建 ros2_control controller_manager 后加载 FT broadcaster。
    ft_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "massage_ft_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    (
                        "force_torque_sensor_broadcaster/"
                        "ForceTorqueSensorBroadcaster"
                    ),
                    "--param-file",
                    controller_config,
                    "--controller-manager-timeout",
                    "10",
                ],
            )
        ],
    )

    return LaunchDescription([
        ft_sensor_sim,
        ft_broadcaster,
    ])
