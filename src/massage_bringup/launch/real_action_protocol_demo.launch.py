from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    node = Node(
        package="massage_jaka",
        executable="real_action_protocol_demo",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "run_protocol_test": ParameterValue(
                    LaunchConfiguration("run_protocol_test"), value_type=bool
                ),
                "test_external_stop": ParameterValue(
                    LaunchConfiguration("test_external_stop"), value_type=bool
                ),
                "joint_state_timeout": ParameterValue(
                    LaunchConfiguration("joint_state_timeout"), value_type=float
                ),
                "action_timeout": ParameterValue(
                    LaunchConfiguration("action_timeout"), value_type=float
                ),
                "hold_duration": ParameterValue(
                    LaunchConfiguration("hold_duration"), value_type=float
                ),
                "maximum_stationary_delta": ParameterValue(
                    LaunchConfiguration("maximum_stationary_delta"),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_protocol_test", default_value="false"),
            DeclareLaunchArgument("test_external_stop", default_value="false"),
            DeclareLaunchArgument("joint_state_timeout", default_value="3.0"),
            DeclareLaunchArgument("action_timeout", default_value="5.0"),
            DeclareLaunchArgument("hold_duration", default_value="3.0"),
            DeclareLaunchArgument(
                "maximum_stationary_delta", default_value="0.005"
            ),
            node,
        ]
    )
