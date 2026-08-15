import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
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
        MoveItConfigsBuilder(
            "jaka_s5",
            package_name="jaka_s5_moveit_config",
        )
        .robot_description(
            file_path=robot_xacro,
            mappings={
                "use_gazebo": "false",
                "use_rviz_sim": "false",
                "use_massage_head": "true",
            },
        )
        .robot_description_semantic(file_path=semantic_file)
        .joint_limits(file_path="config/joint_limits_real.yaml")
        .to_moveit_configs()
    )
    generated_launch = generate_move_group_launch(moveit_config)
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=False),
        *generated_launch.entities,
    ])
