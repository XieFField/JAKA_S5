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
        executable="real_compliance_smoke_demo",
        output="screen",
        parameters=[{
            "activate": _parameter("activate", bool),
            "parameters_confirmed": _parameter("parameters_confirmed", bool),
            "axis": _parameter("axis", int),
            "target_wrench": _parameter("target_wrench", float),
            "maximum_force": _parameter("maximum_force", float),
            "maximum_torque": _parameter("maximum_torque", float),
            "constant": _parameter("constant", float),
            "rebound": _parameter("rebound", float),
            "run_duration": _parameter("run_duration", float),
            "guard_timeout": _parameter("guard_timeout", float),
            "readiness_timeout": _parameter("readiness_timeout", float),
            "feedback_timeout": _parameter("feedback_timeout", float),
            "state_timeout": _parameter("state_timeout", float),
            "maximum_joint_displacement": _parameter(
                "maximum_joint_displacement", float
            ),
            "maximum_linear_displacement": _parameter(
                "maximum_linear_displacement", float
            ),
            "wrench_frame": LaunchConfiguration("wrench_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "tool_frame": LaunchConfiguration("tool_frame"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("activate", default_value="false"),
        DeclareLaunchArgument("parameters_confirmed", default_value="false"),
        DeclareLaunchArgument("axis", default_value="2"),
        DeclareLaunchArgument("target_wrench", default_value="0.0"),
        DeclareLaunchArgument("maximum_force", default_value="5.0"),
        DeclareLaunchArgument("maximum_torque", default_value="1.0"),
        DeclareLaunchArgument("constant", default_value="0.0"),
        DeclareLaunchArgument("rebound", default_value="0.0"),
        DeclareLaunchArgument("run_duration", default_value="2.0"),
        DeclareLaunchArgument("guard_timeout", default_value="3.0"),
        DeclareLaunchArgument("readiness_timeout", default_value="5.0"),
        DeclareLaunchArgument("feedback_timeout", default_value="0.5"),
        DeclareLaunchArgument("state_timeout", default_value="0.5"),
        DeclareLaunchArgument(
            "maximum_joint_displacement", default_value="0.02"
        ),
        DeclareLaunchArgument(
            "maximum_linear_displacement", default_value="0.01"
        ),
        DeclareLaunchArgument("wrench_frame", default_value="Link_06"),
        DeclareLaunchArgument("base_frame", default_value="world"),
        DeclareLaunchArgument("tool_frame", default_value="massage_tool_tip"),
        node,
    ])
