import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    description = get_package_share_directory("massage_description")
    robot_xacro = os.path.join(
        description, "urdf", "jaka_s5_massage.urdf.xacro"
    )
    semantic = os.path.join(
        description, "config", "jaka_s5_massage.srdf"
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
        .robot_description_semantic(file_path=semantic)
        .joint_limits(file_path="config/joint_limits_real.yaml")
        .to_moveit_configs()
    )
    node = Node(
        package="massage_bringup",
        executable="technique_trajectory_demo",
        name="technique_trajectory_real_plan_only",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "technique": LaunchConfiguration("technique"),
                "execution_environment": "real",
                "parameters_confirmed": typed("parameters_confirmed", bool),
                "execute": False,
                "velocity_scale": typed("velocity_scale", float),
                "acceleration_scale": typed("acceleration_scale", float),
                "planning_timeout": typed("planning_timeout", float),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("technique", default_value="press"),
            DeclareLaunchArgument("parameters_confirmed", default_value="false"),
            DeclareLaunchArgument("velocity_scale", default_value="0.02"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
            DeclareLaunchArgument("planning_timeout", default_value="10.0"),
            node,
        ]
    )
