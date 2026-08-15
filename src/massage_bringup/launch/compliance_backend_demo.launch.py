import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")
    test_mode = LaunchConfiguration("test_mode")

    controller_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                "launch",
                "admittance_controller_load_demo.launch.py",
            )
        )
    )

    demo = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="massage_motion",
                executable="compliance_backend_demo",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"test_mode": test_mode},
                    {"run_duration": 1.0},
                    {"request_timeout": 2.0},
                    {"force_limit": 5.0},
                ],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "test_mode",
            default_value="normal",
            description="normal, timeout, or force_limit",
        ),
        controller_system,
        demo,
    ])
