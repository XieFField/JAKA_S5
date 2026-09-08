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
    semantic_file = os.path.join(
        description, "config", "jaka_s5_massage.srdf"
    )
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_s5", package_name="jaka_s5_moveit_config"
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
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits_real.yaml")
        .to_moveit_configs()
    )

    node = Node(
        package="massage_bringup",
        executable="r10_precontact_real",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "use_sim_time": False,
                "execute": typed("execute", bool),
                "parameters_confirmed": typed(
                    "parameters_confirmed", bool
                ),
                "planning_attempts": typed("planning_attempts", int),
                "maximum_ik_attempts": typed("maximum_ik_attempts", int),
                "maximum_unique_ik_candidates": typed(
                    "maximum_unique_ik_candidates", int
                ),
                "contact_x": typed("contact_x", float),
                "contact_y": typed("contact_y", float),
                "contact_z": typed("contact_z", float),
                "precontact_clearance": typed(
                    "precontact_clearance", float
                ),
                "work_ready_clearance": typed(
                    "work_ready_clearance", float
                ),
                "velocity_scale": typed("velocity_scale", float),
                "acceleration_scale": typed("acceleration_scale", float),
                "planning_timeout": typed("planning_timeout", float),
                "execution_timeout_margin": typed(
                    "execution_timeout_margin", float
                ),
                "maximum_joint_travel": typed(
                    "maximum_joint_travel", float
                ),
                "endpoint_tolerance": typed("endpoint_tolerance", float),
                "maximum_axis_error_degrees": typed(
                    "maximum_axis_error_degrees", float
                ),
                "state_timeout": typed("state_timeout", float),
                "feedback_timeout": typed("feedback_timeout", float),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument(
                "parameters_confirmed", default_value="false"
            ),
            DeclareLaunchArgument("planning_attempts", default_value="3"),
            DeclareLaunchArgument("maximum_ik_attempts", default_value="32"),
            DeclareLaunchArgument(
                "maximum_unique_ik_candidates", default_value="8"
            ),
            DeclareLaunchArgument(
                "contact_x", default_value="-0.471238630147741"
            ),
            DeclareLaunchArgument(
                "contact_y", default_value="0.156275068796867"
            ),
            DeclareLaunchArgument(
                "contact_z", default_value="0.281381721182422"
            ),
            DeclareLaunchArgument(
                "precontact_clearance", default_value="0.002"
            ),
            DeclareLaunchArgument(
                "work_ready_clearance", default_value="0.050"
            ),
            DeclareLaunchArgument("velocity_scale", default_value="0.05"),
            DeclareLaunchArgument(
                "acceleration_scale", default_value="0.05"
            ),
            DeclareLaunchArgument("planning_timeout", default_value="30.0"),
            DeclareLaunchArgument(
                "execution_timeout_margin", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "maximum_joint_travel", default_value="3.5"
            ),
            DeclareLaunchArgument(
                "endpoint_tolerance", default_value="0.002"
            ),
            DeclareLaunchArgument(
                "maximum_axis_error_degrees", default_value="3.0"
            ),
            DeclareLaunchArgument("state_timeout", default_value="5.0"),
            DeclareLaunchArgument("feedback_timeout", default_value="0.5"),
            node,
        ]
    )
