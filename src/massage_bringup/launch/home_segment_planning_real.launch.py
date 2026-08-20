import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


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
        MoveItConfigsBuilder("jaka_s5", package_name="jaka_s5_moveit_config")
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

    node = Node(
        package="massage_bringup",
        executable="home_segment_planning_real",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "segment_ratio": float_parameter("segment_ratio"),
                "planning_attempts": ParameterValue(
                    LaunchConfiguration("planning_attempts"), value_type=int
                ),
                "velocity_scale": float_parameter("velocity_scale"),
                "acceleration_scale": float_parameter("acceleration_scale"),
                "planning_timeout": float_parameter("planning_timeout"),
                "joint_state_timeout": float_parameter("joint_state_timeout"),
                "maximum_joint_travel": float_parameter(
                    "maximum_joint_travel"
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("segment_ratio", default_value="0.25"),
            DeclareLaunchArgument("planning_attempts", default_value="3"),
            DeclareLaunchArgument("velocity_scale", default_value="0.02"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
            DeclareLaunchArgument("planning_timeout", default_value="5.0"),
            DeclareLaunchArgument("joint_state_timeout", default_value="3.0"),
            DeclareLaunchArgument("maximum_joint_travel", default_value="0.15"),
            node,
        ]
    )
