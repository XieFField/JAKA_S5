from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    joint_error_tolerance = LaunchConfiguration("joint_error_tolerance")
    test_mode = LaunchConfiguration("test_mode")
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_s5",
            package_name="jaka_s5_moveit_config",
        )
        .robot_description(
            mappings={
                "use_rviz_sim": "false",
                "use_gazebo": "true",
            }
        )
        .joint_limits(
            file_path="config/joint_limits_sim.yaml",
        )
        .to_moveit_configs()
    )

    minimal_task_demo = Node(
        package="massage_task",
        executable="minimal_task_demo",
        name="minimal_task_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "joint_error_tolerance": joint_error_tolerance,
                "test_mode": test_mode,
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "joint_error_tolerance",
            default_value="0.01",
            description="Maximum allowed endpoint joint error in radians",

        ),
        DeclareLaunchArgument(
            "test_mode",
            default_value="normal",
            description="Execution test mode: normal, cancel, or timeout",
        ),
        minimal_task_demo,
    ])
