import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")
    moveit_share = get_package_share_directory("jaka_s5_moveit_config")

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
            os.path.join(moveit_share, "launch", "move_group_sim.launch.py")
        )
    )

    force_limit = LaunchConfiguration("force_limit")
    expect_limit_exceeded = LaunchConfiguration("expect_limit_exceeded")

    contact_demo = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="massage_task",
                executable="compliance_contact_demo",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "force_limit": force_limit,
                    "expect_limit_exceeded": expect_limit_exceeded,
                }],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("force_limit", default_value="5.0"),
        DeclareLaunchArgument(
            "expect_limit_exceeded",
            default_value="false",
        ),
        SetParameter(name="use_sim_time", value=True),
        controller_system,
        move_group,
        contact_demo,
    ])
