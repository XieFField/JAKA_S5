import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    robot_xacro = os.path.join(
        get_package_share_directory("massage_description"),
        "urdf",
        "jaka_s5_massage.urdf.xacro",
    )
    semantic_file = os.path.join(
        get_package_share_directory("massage_description"),
        "config",
        "jaka_s5_massage.srdf",
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_s5", package_name="jaka_s5_moveit_config")
        .robot_description(
            file_path=robot_xacro,
            mappings={
                "use_gazebo": "true",
                "use_rviz_sim": "false",
                "use_massage_head": "true",
                "initial_positions_file": initial_positions_file,
            },
        )
        .robot_description_semantic(file_path=semantic_file)
        .joint_limits(file_path="config/joint_limits_sim.yaml")
        .to_moveit_configs()
    )
    generated_launch = generate_moveit_rviz_launch(moveit_config)
    return LaunchDescription([
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=os.path.join(
                get_package_share_directory("jaka_s5_moveit_config"),
                "config",
                "initial_positions.yaml",
            ),
            description="Initial joint positions embedded in robot_description",
        ),
        *generated_launch.entities,
    ])
