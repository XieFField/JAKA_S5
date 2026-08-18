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
        executable="return_home_real",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                ),
                "parameters_confirmed": ParameterValue(
                    LaunchConfiguration("parameters_confirmed"), value_type=bool
                ),
                "planning_attempts": ParameterValue(
                    LaunchConfiguration("planning_attempts"), value_type=int
                ),
                "velocity_scale": float_parameter("velocity_scale"),
                "acceleration_scale": float_parameter("acceleration_scale"),
                "planning_timeout": float_parameter("planning_timeout"),
                "execution_timeout_margin": float_parameter(
                    "execution_timeout_margin"
                ),
                "state_timeout": float_parameter("state_timeout"),
                "readiness_timeout": float_parameter("readiness_timeout"),
                "feedback_timeout": float_parameter("feedback_timeout"),
                "maximum_joint_travel": float_parameter("maximum_joint_travel"),
                "endpoint_tolerance": float_parameter("endpoint_tolerance"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("parameters_confirmed", default_value="false"),
            DeclareLaunchArgument("planning_attempts", default_value="3"),
            DeclareLaunchArgument("velocity_scale", default_value="0.02"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
            DeclareLaunchArgument("planning_timeout", default_value="5.0"),
            DeclareLaunchArgument(
                "execution_timeout_margin", default_value="15.0"
            ),
            DeclareLaunchArgument("state_timeout", default_value="3.0"),
            DeclareLaunchArgument("readiness_timeout", default_value="60.0"),
            DeclareLaunchArgument("feedback_timeout", default_value="1.0"),
            DeclareLaunchArgument("maximum_joint_travel", default_value="3.5"),
            DeclareLaunchArgument("endpoint_tolerance", default_value="0.01"),
            node,
        ]
    )
