import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _validate_arguments(context):
    execute = LaunchConfiguration("execute").perform(context).lower() == "true"
    confirmed = (
        LaunchConfiguration("parameters_confirmed").perform(context).lower()
        == "true"
    )
    if execute and not confirmed:
        raise RuntimeError(
            "真机重复性执行要求 execute:=true 和 parameters_confirmed:=true"
        )

    cycles = int(LaunchConfiguration("cycles").perform(context))
    joint_delta = float(LaunchConfiguration("joint_delta").perform(context))
    maximum_joint_delta = float(
        LaunchConfiguration("maximum_joint_delta").perform(context)
    )
    endpoint_tolerance = float(
        LaunchConfiguration("endpoint_tolerance").perform(context)
    )
    minimum_completion = float(
        LaunchConfiguration("minimum_completion_ratio").perform(context)
    )
    maximum_range = float(
        LaunchConfiguration("maximum_position_range").perform(context)
    )
    if not 1 <= cycles <= 10:
        raise RuntimeError("cycles 必须在 [1, 10] 内")
    if (
        not math.isfinite(joint_delta)
        or joint_delta <= 0.0
        or not math.isfinite(maximum_joint_delta)
        or maximum_joint_delta <= 0.0
        or joint_delta > maximum_joint_delta
    ):
        raise RuntimeError("joint_delta 或 maximum_joint_delta 无效")
    if (
        not math.isfinite(endpoint_tolerance)
        or endpoint_tolerance <= 0.0
        or endpoint_tolerance >= joint_delta
    ):
        raise RuntimeError("endpoint_tolerance 必须在 (0, joint_delta) 内")
    if (
        not math.isfinite(minimum_completion)
        or not 0.0 < minimum_completion <= 1.0
    ):
        raise RuntimeError("minimum_completion_ratio 必须在 (0, 1] 内")
    if not math.isfinite(maximum_range) or maximum_range < 0.0:
        raise RuntimeError("maximum_position_range 不能为负数")
    return []


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
        package="massage_jaka",
        executable="real_motion_repeatability_demo",
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
                "joint_name": LaunchConfiguration("joint_name"),
                "cycles": ParameterValue(
                    LaunchConfiguration("cycles"), value_type=int
                ),
                "joint_delta": float_parameter("joint_delta"),
                "maximum_joint_delta": float_parameter(
                    "maximum_joint_delta"
                ),
                "velocity_scale": float_parameter("velocity_scale"),
                "acceleration_scale": float_parameter(
                    "acceleration_scale"
                ),
                "planning_timeout": float_parameter("planning_timeout"),
                "execution_timeout_margin": float_parameter(
                    "execution_timeout_margin"
                ),
                "state_timeout": float_parameter("state_timeout"),
                "readiness_timeout": float_parameter("readiness_timeout"),
                "feedback_timeout": float_parameter("feedback_timeout"),
                "endpoint_tolerance": float_parameter(
                    "endpoint_tolerance"
                ),
                "minimum_completion_ratio": float_parameter(
                    "minimum_completion_ratio"
                ),
                "maximum_position_range": float_parameter(
                    "maximum_position_range"
                ),
                "settling_duration": float_parameter("settling_duration"),
                "inter_trial_delay": float_parameter("inter_trial_delay"),
                "output_csv": LaunchConfiguration("output_csv"),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("parameters_confirmed", default_value="false"),
        DeclareLaunchArgument("joint_name", default_value="joint_1"),
        DeclareLaunchArgument("cycles", default_value="3"),
        DeclareLaunchArgument("joint_delta", default_value="0.01"),
        DeclareLaunchArgument("maximum_joint_delta", default_value="0.02"),
        DeclareLaunchArgument("velocity_scale", default_value="0.02"),
        DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
        DeclareLaunchArgument("planning_timeout", default_value="5.0"),
        DeclareLaunchArgument("execution_timeout_margin", default_value="10.0"),
        DeclareLaunchArgument("state_timeout", default_value="3.0"),
        DeclareLaunchArgument("readiness_timeout", default_value="30.0"),
        DeclareLaunchArgument("feedback_timeout", default_value="1.0"),
        DeclareLaunchArgument("endpoint_tolerance", default_value="0.002"),
        DeclareLaunchArgument("minimum_completion_ratio", default_value="0.90"),
        DeclareLaunchArgument("maximum_position_range", default_value="0.001"),
        DeclareLaunchArgument("settling_duration", default_value="0.3"),
        DeclareLaunchArgument("inter_trial_delay", default_value="0.5"),
        DeclareLaunchArgument("output_csv", default_value=""),
        OpaqueFunction(function=_validate_arguments),
        node,
    ])
