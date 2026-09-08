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
    bringup_share = get_package_share_directory("massage_bringup")
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
        executable="push_trajectory_real_plan_only",
        name="push_trajectory_real_plan_only",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "execution_environment": "real_plan_only",
                "parameters_confirmed": ParameterValue(
                    LaunchConfiguration("parameters_confirmed"),
                    value_type=bool,
                ),
                "config_file": LaunchConfiguration("config_file"),
                "joint_state_timeout": float_parameter(
                    "joint_state_timeout"
                ),
                "maximum_ik_attempts": ParameterValue(
                    LaunchConfiguration("maximum_ik_attempts"),
                    value_type=int,
                ),
                "maximum_unique_ik_candidates": ParameterValue(
                    LaunchConfiguration("maximum_unique_ik_candidates"),
                    value_type=int,
                ),
                "ptp_planning_attempts_per_candidate": ParameterValue(
                    LaunchConfiguration("ptp_planning_attempts_per_candidate"),
                    value_type=int,
                ),
                "lin_planning_attempts": ParameterValue(
                    LaunchConfiguration("lin_planning_attempts"),
                    value_type=int,
                ),
                "ik_random_seed": ParameterValue(
                    LaunchConfiguration("ik_random_seed"), value_type=int
                ),
                "ik_base_timeout": float_parameter("ik_base_timeout"),
                "ik_timeout_per_meter": float_parameter(
                    "ik_timeout_per_meter"
                ),
                "ik_timeout_per_radian": float_parameter(
                    "ik_timeout_per_radian"
                ),
                "ik_minimum_timeout": float_parameter("ik_minimum_timeout"),
                "ik_maximum_timeout": float_parameter("ik_maximum_timeout"),
                "ik_failure_backoff_factor": float_parameter(
                    "ik_failure_backoff_factor"
                ),
                "ik_duplicate_tolerance": float_parameter(
                    "ik_duplicate_tolerance"
                ),
                "velocity_scale": float_parameter("velocity_scale"),
                "acceleration_scale": float_parameter("acceleration_scale"),
                "planning_timeout": float_parameter("planning_timeout"),
                "maximum_joint_travel": float_parameter(
                    "maximum_joint_travel"
                ),
                "link_height_gate_enabled": True,
                "diagnostic_link_name": LaunchConfiguration(
                    "diagnostic_link_name"
                ),
                "minimum_link_height": float_parameter(
                    "minimum_link_height"
                ),
                "maximum_link_drop_from_standby": float_parameter(
                    "maximum_link_drop_from_standby"
                ),
                "elbow_posture_diagnostics_enabled": ParameterValue(
                    LaunchConfiguration("elbow_posture_diagnostics_enabled"),
                    value_type=bool,
                ),
                "shoulder_link_name": LaunchConfiguration(
                    "shoulder_link_name"
                ),
                "elbow_link_name": LaunchConfiguration("elbow_link_name"),
                "wrist_link_name": LaunchConfiguration("wrist_link_name"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_confirmed", default_value="false"
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(
                    bringup_share,
                    "config",
                    "push_contact_sequence_stage_b.yaml",
                ),
            ),
            DeclareLaunchArgument("joint_state_timeout", default_value="30.0"),
            DeclareLaunchArgument("maximum_ik_attempts", default_value="24"),
            DeclareLaunchArgument(
                "maximum_unique_ik_candidates", default_value="8"
            ),
            DeclareLaunchArgument(
                "ptp_planning_attempts_per_candidate", default_value="2"
            ),
            DeclareLaunchArgument("lin_planning_attempts", default_value="1"),
            DeclareLaunchArgument("ik_random_seed", default_value="684"),
            DeclareLaunchArgument("ik_base_timeout", default_value="0.02"),
            DeclareLaunchArgument(
                "ik_timeout_per_meter", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "ik_timeout_per_radian", default_value="0.05"
            ),
            DeclareLaunchArgument("ik_minimum_timeout", default_value="0.02"),
            DeclareLaunchArgument("ik_maximum_timeout", default_value="0.50"),
            DeclareLaunchArgument(
                "ik_failure_backoff_factor", default_value="1.35"
            ),
            DeclareLaunchArgument(
                "ik_duplicate_tolerance", default_value="0.0001"
            ),
            DeclareLaunchArgument("velocity_scale", default_value="0.02"),
            DeclareLaunchArgument(
                "acceleration_scale", default_value="0.02"
            ),
            DeclareLaunchArgument("planning_timeout", default_value="5.0"),
            DeclareLaunchArgument(
                "maximum_joint_travel", default_value="3.5"
            ),
            DeclareLaunchArgument(
                "diagnostic_link_name", default_value="Link_03"
            ),
            DeclareLaunchArgument(
                "minimum_link_height", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "maximum_link_drop_from_standby", default_value="0.20"
            ),
            DeclareLaunchArgument(
                "elbow_posture_diagnostics_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "shoulder_link_name", default_value="Link_02"
            ),
            DeclareLaunchArgument(
                "elbow_link_name", default_value="Link_03"
            ),
            DeclareLaunchArgument(
                "wrist_link_name", default_value="Link_04"
            ),
            node,
        ]
    )
