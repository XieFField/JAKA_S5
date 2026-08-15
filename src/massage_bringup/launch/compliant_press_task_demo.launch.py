import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")

    controller_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                "launch",
                "admittance_controller_load_demo.launch.py",
            )
        )
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "move_group_sim.launch.py")
        )
    )

    demo_node = Node(
        package="massage_task",
        executable="compliant_press_task_demo",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "approach_distance": LaunchConfiguration("approach_distance"),
            "precontact_distance": LaunchConfiguration("precontact_distance"),
            "contact_threshold": LaunchConfiguration("contact_threshold"),
            "force_limit": LaunchConfiguration("force_limit"),
            "hold_duration": LaunchConfiguration("hold_duration"),
            "expect_limit_exceeded": LaunchConfiguration(
                "expect_limit_exceeded"
            ),
        }],
    )
    demo = TimerAction(
        period=10.0,
        actions=[demo_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument("approach_distance", default_value="0.04"),
        DeclareLaunchArgument("precontact_distance", default_value="0.0055"),
        DeclareLaunchArgument("contact_threshold", default_value="0.1"),
        DeclareLaunchArgument("force_limit", default_value="1.0"),
        DeclareLaunchArgument("hold_duration", default_value="2.0"),
        DeclareLaunchArgument("expect_limit_exceeded", default_value="false"),
        SetParameter(name="use_sim_time", value=True),
        controller_system,
        move_group,
        demo,
        # demo 结束后保留仿真基础设施，便于核查 TF、碰撞场景和控制器；
        # 使用 Ctrl-C 统一关闭，避免一并强杀 move_group/Gazebo 掩盖子进程错误。
    ])
