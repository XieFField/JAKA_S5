from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _parameter(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    node = Node(
        package="massage_jaka",
        executable="real_tool_drive_smoke_demo",
        output="screen",
        parameters=[{
            "activate": _parameter("activate", bool),
            "parameters_confirmed": _parameter("parameters_confirmed", bool),
            "axis": _parameter("axis", int),
            "frame": _parameter("frame", int),
            "rebound": _parameter("rebound", float),
            "rigidity": _parameter("rigidity", float),
            "preserve_existing_axis_parameters": _parameter(
                "preserve_existing_axis_parameters", bool
            ),
            "preserve_existing_sensitivity": _parameter(
                "preserve_existing_sensitivity", bool
            ),
            "preserve_existing_warning_range": _parameter(
                "preserve_existing_warning_range", bool
            ),
            "sensitivity_level": _parameter("sensitivity_level", int),
            "warning_range": _parameter("warning_range", int),
            "baseline_duration": _parameter("baseline_duration", float),
            "settle_duration": _parameter("settle_duration", float),
            "load_duration": _parameter("load_duration", float),
            "recovery_duration": _parameter("recovery_duration", float),
            "readiness_timeout": _parameter("readiness_timeout", float),
            "feedback_timeout": _parameter("feedback_timeout", float),
            "state_timeout": _parameter("state_timeout", float),
            "log_period": _parameter("log_period", float),
            "minimum_force_delta": _parameter(
                "minimum_force_delta", float
            ),
            "minimum_axis_displacement": _parameter(
                "minimum_axis_displacement", float
            ),
            "maximum_force": _parameter("maximum_force", float),
            "maximum_torque": _parameter("maximum_torque", float),
            "maximum_linear_displacement": _parameter(
                "maximum_linear_displacement", float
            ),
            "maximum_transverse_displacement": _parameter(
                "maximum_transverse_displacement", float
            ),
            "maximum_joint_displacement": _parameter(
                "maximum_joint_displacement", float
            ),
            "wrench_frame": LaunchConfiguration("wrench_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "tool_frame": LaunchConfiguration("tool_frame"),
            "csv_path": LaunchConfiguration("csv_path"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("activate", default_value="false"),
        DeclareLaunchArgument("parameters_confirmed", default_value="false"),
        DeclareLaunchArgument("axis", default_value="2"),
        DeclareLaunchArgument("frame", default_value="0"),
        DeclareLaunchArgument("rebound", default_value="0.0"),
        DeclareLaunchArgument("rigidity", default_value="0.0"),
        DeclareLaunchArgument(
            "preserve_existing_axis_parameters", default_value="true"
        ),
        DeclareLaunchArgument(
            "preserve_existing_sensitivity", default_value="true"
        ),
        DeclareLaunchArgument(
            "preserve_existing_warning_range", default_value="true"
        ),
        DeclareLaunchArgument("sensitivity_level", default_value="1"),
        DeclareLaunchArgument("warning_range", default_value="1"),
        DeclareLaunchArgument("baseline_duration", default_value="2.0"),
        DeclareLaunchArgument("settle_duration", default_value="1.0"),
        DeclareLaunchArgument("load_duration", default_value="4.0"),
        DeclareLaunchArgument("recovery_duration", default_value="2.0"),
        DeclareLaunchArgument("readiness_timeout", default_value="5.0"),
        DeclareLaunchArgument("feedback_timeout", default_value="0.5"),
        DeclareLaunchArgument("state_timeout", default_value="0.5"),
        DeclareLaunchArgument("log_period", default_value="0.5"),
        DeclareLaunchArgument("minimum_force_delta", default_value="0.5"),
        DeclareLaunchArgument(
            "minimum_axis_displacement", default_value="0.0002"
        ),
        DeclareLaunchArgument("maximum_force", default_value="5.0"),
        DeclareLaunchArgument("maximum_torque", default_value="1.0"),
        DeclareLaunchArgument(
            "maximum_linear_displacement", default_value="0.002"
        ),
        DeclareLaunchArgument(
            "maximum_transverse_displacement", default_value="0.0005"
        ),
        DeclareLaunchArgument(
            "maximum_joint_displacement", default_value="0.005"
        ),
        DeclareLaunchArgument("wrench_frame", default_value="Link_06"),
        DeclareLaunchArgument("base_frame", default_value="world"),
        DeclareLaunchArgument("tool_frame", default_value="massage_tool_tip"),
        DeclareLaunchArgument("csv_path", default_value=""),
        node,
    ])
