import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _validate_arguments(context):
    execute = LaunchConfiguration("execute").perform(context).lower() == "true"
    confirmed = (
        LaunchConfiguration("parameters_confirmed").perform(context).lower()
        == "true"
    )
    if not execute or not confirmed:
        raise RuntimeError(
            "真机分段往返要求 execute:=true 和 parameters_confirmed:=true"
        )

    ratio = float(LaunchConfiguration("segment_ratio").perform(context))
    attempts = int(LaunchConfiguration("planning_attempts").perform(context))
    maximum_travel = float(
        LaunchConfiguration("maximum_joint_travel").perform(context)
    )
    endpoint_tolerance = float(
        LaunchConfiguration("endpoint_tolerance").perform(context)
    )
    if not math.isfinite(ratio) or not 0.0 < ratio <= 1.0:
        raise RuntimeError("segment_ratio 必须在 (0, 1] 内")
    if not 1 <= attempts <= 10:
        raise RuntimeError("planning_attempts 必须在 [1, 10] 内")
    if not math.isfinite(maximum_travel) or maximum_travel <= 0.0:
        raise RuntimeError("maximum_joint_travel 必须为有限正数")
    if (
        not math.isfinite(endpoint_tolerance)
        or endpoint_tolerance <= 0.0
        or endpoint_tolerance >= maximum_travel
    ):
        raise RuntimeError(
            "endpoint_tolerance 必须为正数且小于 maximum_joint_travel"
        )
    return []


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
        executable="home_segment_round_trip_real",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                ),
                "parameters_confirmed": ParameterValue(
                    LaunchConfiguration("parameters_confirmed"),
                    value_type=bool,
                ),
                "segment_ratio": _float_parameter("segment_ratio"),
                "planning_attempts": ParameterValue(
                    LaunchConfiguration("planning_attempts"), value_type=int
                ),
                "velocity_scale": _float_parameter("velocity_scale"),
                "acceleration_scale": _float_parameter(
                    "acceleration_scale"
                ),
                "planning_timeout": _float_parameter("planning_timeout"),
                "execution_timeout_margin": _float_parameter(
                    "execution_timeout_margin"
                ),
                "state_timeout": _float_parameter("state_timeout"),
                "readiness_timeout": _float_parameter("readiness_timeout"),
                "feedback_timeout": _float_parameter("feedback_timeout"),
                "maximum_joint_travel": _float_parameter(
                    "maximum_joint_travel"
                ),
                "start_state_tolerance": _float_parameter(
                    "start_state_tolerance"
                ),
                "endpoint_tolerance": _float_parameter(
                    "endpoint_tolerance"
                ),
                "settling_duration": _float_parameter("settling_duration"),
                "inter_leg_delay": _float_parameter("inter_leg_delay"),
                "output_csv": LaunchConfiguration("output_csv"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("parameters_confirmed", default_value="false"),
            DeclareLaunchArgument("segment_ratio", default_value="0.25"),
            DeclareLaunchArgument("planning_attempts", default_value="3"),
            DeclareLaunchArgument("velocity_scale", default_value="0.02"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
            DeclareLaunchArgument("planning_timeout", default_value="5.0"),
            DeclareLaunchArgument(
                "execution_timeout_margin", default_value="10.0"
            ),
            DeclareLaunchArgument("state_timeout", default_value="3.0"),
            DeclareLaunchArgument("readiness_timeout", default_value="30.0"),
            DeclareLaunchArgument("feedback_timeout", default_value="1.0"),
            DeclareLaunchArgument("maximum_joint_travel", default_value="0.15"),
            DeclareLaunchArgument("start_state_tolerance", default_value="0.01"),
            DeclareLaunchArgument("endpoint_tolerance", default_value="0.002"),
            DeclareLaunchArgument("settling_duration", default_value="0.3"),
            DeclareLaunchArgument("inter_leg_delay", default_value="0.5"),
            DeclareLaunchArgument("output_csv", default_value=""),
            OpaqueFunction(function=_validate_arguments),
            node,
        ]
    )
