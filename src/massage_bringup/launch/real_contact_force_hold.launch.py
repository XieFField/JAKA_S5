from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    transformer = Node(
        package="massage_motion",
        executable="wrench_frame_transformer",
        name="r10_wrench_frame_transformer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("activate")),
        parameters=[{
            "input_topic": "/jaka_driver/wrench",
            "output_topic": "/massage/ft_sensor/wrench_world",
            "source_frame_override": "Link_06",
            "expression_frame": "world",
            "reference_point_frame": "massage_tool_tip",
            "output_frame": "massage_tool_tip_world_aligned",
            "validation_sample_count": 0,
        }],
    )
    test = Node(
        package="massage_jaka",
        executable="real_contact_force_hold_demo",
        output="screen",
        parameters=[{
            "activate": typed("activate", bool),
            "parameters_confirmed": typed("parameters_confirmed", bool),
            "execute_contact_search": typed("execute_contact_search", bool),
            "target_wrench_z": typed("target_wrench_z", float),
            "contact_threshold": typed("contact_threshold", float),
            "target_tolerance": typed("target_tolerance", float),
            "maximum_force": typed("maximum_force", float),
            "maximum_torque": typed("maximum_torque", float),
            "maximum_speed_wrench": typed("maximum_speed_wrench", float),
            "rebound_wrench": typed("rebound_wrench", float),
            "sensor_compensation": typed("sensor_compensation", int),
            "compliance_type": typed("compliance_type", int),
            "compliance_linear_speed_limit_mm_s": typed(
                "compliance_linear_speed_limit_mm_s", float
            ),
            "compliance_angular_speed_limit_rad_s": typed(
                "compliance_angular_speed_limit_rad_s", float
            ),
            "approach_linear_speed_limit_mm_s": typed(
                "approach_linear_speed_limit_mm_s", float
            ),
            "approach_angular_speed_limit_rad_s": typed(
                "approach_angular_speed_limit_rad_s", float
            ),
            "baseline_duration": typed("baseline_duration", float),
            "maximum_baseline_force_bias": typed(
                "maximum_baseline_force_bias", float
            ),
            "maximum_baseline_torque_bias": typed(
                "maximum_baseline_torque_bias", float
            ),
            "contact_timeout": typed("contact_timeout", float),
            "hold_duration": typed("hold_duration", float),
            "minimum_hold_in_tolerance_ratio": typed(
                "minimum_hold_in_tolerance_ratio", float
            ),
            "maximum_hold_standard_deviation": typed(
                "maximum_hold_standard_deviation", float
            ),
            "maximum_hold_sample_gap": typed(
                "maximum_hold_sample_gap", float
            ),
            "maximum_hold_mean_absolute_error": typed(
                "maximum_hold_mean_absolute_error", float
            ),
            "maximum_hold_terminal_error": typed(
                "maximum_hold_terminal_error", float
            ),
            "minimum_hold_samples": typed("minimum_hold_samples", int),
            "contact_confirmation_samples": typed(
                "contact_confirmation_samples", int
            ),
            "target_confirmation_samples": typed(
                "target_confirmation_samples", int
            ),
            "maximum_joint_displacement": typed(
                "maximum_joint_displacement", float
            ),
            "maximum_linear_displacement": typed(
                "maximum_linear_displacement", float
            ),
            "maximum_transverse_displacement": typed(
                "maximum_transverse_displacement", float
            ),
            "maximum_wrong_direction_displacement": typed(
                "maximum_wrong_direction_displacement", float
            ),
            "expected_motion_direction_z": typed(
                "expected_motion_direction_z", float
            ),
            "expected_measured_force_sign_z": typed(
                "expected_measured_force_sign_z", float
            ),
            "maximum_tool_axis_error_degrees": typed(
                "maximum_tool_axis_error_degrees", float
            ),
            "contact_x": typed("contact_x", float),
            "contact_y": typed("contact_y", float),
            "contact_z": typed("contact_z", float),
            "precontact_clearance": typed("precontact_clearance", float),
            "maximum_precontact_position_error": typed(
                "maximum_precontact_position_error", float
            ),
            "readiness_timeout": typed("readiness_timeout", float),
            "service_timeout": typed("service_timeout", float),
            "feedback_timeout": typed("feedback_timeout", float),
            "state_timeout": typed("state_timeout", float),
            "csv_path": LaunchConfiguration("csv_path"),
        }],
    )
    delayed_test = TimerAction(
        period=LaunchConfiguration("startup_delay"), actions=[test]
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=test,
            on_exit=[EmitEvent(event=Shutdown(reason="R10 test completed"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("activate", default_value="false"),
        DeclareLaunchArgument("parameters_confirmed", default_value="false"),
        DeclareLaunchArgument("execute_contact_search", default_value="false"),
        DeclareLaunchArgument("startup_delay", default_value="1.0"),
        DeclareLaunchArgument("target_wrench_z", default_value="-1.0"),
        DeclareLaunchArgument("contact_threshold", default_value="0.5"),
        DeclareLaunchArgument("target_tolerance", default_value="0.3"),
        DeclareLaunchArgument("maximum_force", default_value="5.0"),
        DeclareLaunchArgument("maximum_torque", default_value="1.0"),
        DeclareLaunchArgument("maximum_speed_wrench", default_value="5.0"),
        DeclareLaunchArgument("rebound_wrench", default_value="0.0"),
        DeclareLaunchArgument("sensor_compensation", default_value="1"),
        DeclareLaunchArgument("compliance_type", default_value="1"),
        DeclareLaunchArgument(
            "compliance_linear_speed_limit_mm_s", default_value="2.0"
        ),
        DeclareLaunchArgument(
            "compliance_angular_speed_limit_rad_s", default_value="0.05"
        ),
        DeclareLaunchArgument(
            "approach_linear_speed_limit_mm_s", default_value="2.0"
        ),
        DeclareLaunchArgument(
            "approach_angular_speed_limit_rad_s", default_value="0.05"
        ),
        DeclareLaunchArgument("baseline_duration", default_value="2.0"),
        DeclareLaunchArgument(
            "maximum_baseline_force_bias", default_value="0.5"
        ),
        DeclareLaunchArgument(
            "maximum_baseline_torque_bias", default_value="0.1"
        ),
        DeclareLaunchArgument("contact_timeout", default_value="5.0"),
        DeclareLaunchArgument("hold_duration", default_value="3.0"),
        DeclareLaunchArgument(
            "minimum_hold_in_tolerance_ratio", default_value="0.8"
        ),
        DeclareLaunchArgument(
            "maximum_hold_standard_deviation", default_value="0.25"
        ),
        DeclareLaunchArgument("maximum_hold_sample_gap", default_value="0.2"),
        DeclareLaunchArgument(
            "maximum_hold_mean_absolute_error", default_value="0.25"
        ),
        DeclareLaunchArgument(
            "maximum_hold_terminal_error", default_value="0.3"
        ),
        DeclareLaunchArgument("minimum_hold_samples", default_value="20"),
        DeclareLaunchArgument(
            "contact_confirmation_samples", default_value="3"
        ),
        DeclareLaunchArgument(
            "target_confirmation_samples", default_value="5"
        ),
        DeclareLaunchArgument(
            "maximum_joint_displacement", default_value="0.05"
        ),
        DeclareLaunchArgument(
            "maximum_linear_displacement", default_value="0.012"
        ),
        DeclareLaunchArgument(
            "maximum_transverse_displacement", default_value="0.002"
        ),
        DeclareLaunchArgument(
            "maximum_wrong_direction_displacement", default_value="0.001"
        ),
        DeclareLaunchArgument(
            "expected_motion_direction_z", default_value="-1.0"
        ),
        DeclareLaunchArgument(
            "expected_measured_force_sign_z", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "maximum_tool_axis_error_degrees", default_value="3.0"
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
        DeclareLaunchArgument("precontact_clearance", default_value="0.002"),
        DeclareLaunchArgument(
            "maximum_precontact_position_error", default_value="0.003"
        ),
        DeclareLaunchArgument("readiness_timeout", default_value="8.0"),
        DeclareLaunchArgument("service_timeout", default_value="3.0"),
        DeclareLaunchArgument("feedback_timeout", default_value="0.5"),
        DeclareLaunchArgument("state_timeout", default_value="0.5"),
        DeclareLaunchArgument("csv_path", default_value=""),
        transformer,
        delayed_test,
        shutdown,
    ])
