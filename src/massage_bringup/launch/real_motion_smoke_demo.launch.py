import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


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
        package="massage_jaka",
        executable="real_motion_smoke_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                ),
                "joint_name": LaunchConfiguration("joint_name"),
                "joint_delta": ParameterValue(
                    LaunchConfiguration("joint_delta"), value_type=float
                ),
                "maximum_joint_delta": ParameterValue(
                    LaunchConfiguration("maximum_joint_delta"),
                    value_type=float,
                ),
                "velocity_scale": ParameterValue(
                    LaunchConfiguration("velocity_scale"), value_type=float
                ),
                "acceleration_scale": ParameterValue(
                    LaunchConfiguration("acceleration_scale"),
                    value_type=float,
                ),
                "planning_timeout": ParameterValue(
                    LaunchConfiguration("planning_timeout"), value_type=float
                ),
                "execution_timeout": ParameterValue(
                    LaunchConfiguration("execution_timeout"), value_type=float
                ),
                "joint_state_timeout": ParameterValue(
                    LaunchConfiguration("joint_state_timeout"),
                    value_type=float,
                ),
                "endpoint_tolerance": ParameterValue(
                    LaunchConfiguration("endpoint_tolerance"),
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("joint_name", default_value="joint_1"),
        DeclareLaunchArgument("joint_delta", default_value="0.01"),
        DeclareLaunchArgument("maximum_joint_delta", default_value="0.05"),
        DeclareLaunchArgument("velocity_scale", default_value="0.02"),
        DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
        DeclareLaunchArgument("planning_timeout", default_value="5.0"),
        DeclareLaunchArgument("execution_timeout", default_value="20.0"),
        DeclareLaunchArgument("joint_state_timeout", default_value="3.0"),
        DeclareLaunchArgument("endpoint_tolerance", default_value="0.01"),
        node,
    ])
