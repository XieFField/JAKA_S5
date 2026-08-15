import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")

    ft_sensor_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "ft_sensor_sim.launch.py")
        ),
        launch_arguments={"use_massage_head": "true"}.items(),
    )

    # 等待 Gazebo 生成机器人和 FT 话题后再开始固定姿态基线标定。
    force_admittance_demo = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="massage_motion",
                executable="force_admittance_demo",
                name="force_admittance_demo",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        ft_sensor_sim,
        force_admittance_demo,
    ])
