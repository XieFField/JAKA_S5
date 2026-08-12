from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    run_demo = LaunchConfiguration("run_demo")
    test_mode = LaunchConfiguration("test_mode")
    joint_error_tolerance = LaunchConfiguration("joint_error_tolerance")

    jaka_share = get_package_share_directory(
        "jaka_s5_moveit_config"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                jaka_share,
                "launch",
                "s5_gazebo_control.launch.py",
            )
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                jaka_share,
                "launch",
                "move_group_sim.launch.py",
            )
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                jaka_share,
                "launch",
                "moveit_rviz.launch.py",
            )
        ),
        condition=IfCondition(use_rviz),
    )

    # Gazebo 中的机器人和控制器需要先完成创建。这里使用固定延时建立最小可用
    # 的统一入口；后续系统级 bringup 再改为基于控制器状态的就绪检查。
    demo_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("massage_bringup"),
                        "launch",
                        "minimal_task_demo.launch.py",
                    )
                ),
                launch_arguments={
                    "test_mode": test_mode,
                    "joint_error_tolerance": joint_error_tolerance,
                }.items(),
            )
        ],
        condition=IfCondition(run_demo),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start MoveIt RViz",
        ),
        DeclareLaunchArgument(
            "run_demo",
            default_value="false",
            description="Whether to run the minimal planning/execution demo",
        ),
        DeclareLaunchArgument(
            "test_mode",
            default_value="normal",
            description="Execution test mode: normal, cancel, or timeout",
        ),
        DeclareLaunchArgument(
            "joint_error_tolerance",
            default_value="0.01",
            description="Maximum normal-mode endpoint joint error in radians",
        ),
        gazebo_launch,
        move_group_launch,
        rviz_launch,
        demo_launch,
    ])
