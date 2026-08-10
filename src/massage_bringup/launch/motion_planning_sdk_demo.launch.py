from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
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

    motion_planning_sdk_demo = Node(
        package="massage_motion",
        executable="motion_planning_sdk_demo",
        name="motion_planning_sdk_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )
    return LaunchDescription([motion_planning_sdk_demo])