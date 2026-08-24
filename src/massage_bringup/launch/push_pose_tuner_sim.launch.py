import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
                "use_gazebo": "true",
                "use_rviz_sim": "false",
                "use_massage_head": "true",
            },
        )
        .robot_description_semantic(file_path=semantic_file)
        .joint_limits(file_path="config/joint_limits_sim.yaml")
        .to_moveit_configs()
    )

    simulation = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "sim.launch.py")
                ),
                launch_arguments={
                    "use_rviz": "false",
                    "run_demo": "false",
                    "allow_trajectory_execution": "false",
                }.items(),
            )
        ],
    )

    tuner = TimerAction(
        period=LaunchConfiguration("startup_delay"),
        actions=[
            Node(
                package="massage_motion",
                executable="push_pose_tuner",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "use_sim_time": True,
                        "reference_frame": LaunchConfiguration("reference_frame"),
                        "end_effector_link": LaunchConfiguration(
                            "end_effector_link"
                        ),
                        "direction_x": float_parameter("direction_x"),
                        "direction_y": float_parameter("direction_y"),
                        "push_length": float_parameter("push_length"),
                        "push_speed": float_parameter("push_speed"),
                        "sample_period": float_parameter("sample_period"),
                        "maximum_speed": float_parameter("maximum_speed"),
                        "planning_timeout": float_parameter("planning_timeout"),
                        "velocity_scale": float_parameter("velocity_scale"),
                        "acceleration_scale": float_parameter(
                            "acceleration_scale"
                        ),
                        "maximum_translation_nudge": float_parameter(
                            "maximum_translation_nudge"
                        ),
                        "maximum_rotation_nudge_deg": float_parameter(
                            "maximum_rotation_nudge_deg"
                        ),
                        "marker_scale": float_parameter("marker_scale"),
                        "output_yaml": LaunchConfiguration("output_yaml"),
                    },
                ],
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="push_pose_tuner_rviz",
        output="screen",
        arguments=[
            "-d",
            os.path.join(bringup_share, "config", "push_pose_tuner.rviz"),
        ],
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("startup_delay", default_value="7.0"),
            DeclareLaunchArgument("reference_frame", default_value="world"),
            DeclareLaunchArgument(
                "end_effector_link", default_value="massage_tool_tip"
            ),
            DeclareLaunchArgument("direction_x", default_value="0.0"),
            DeclareLaunchArgument("direction_y", default_value="1.0"),
            DeclareLaunchArgument("push_length", default_value="0.05"),
            DeclareLaunchArgument("push_speed", default_value="0.01"),
            DeclareLaunchArgument("sample_period", default_value="0.05"),
            DeclareLaunchArgument("maximum_speed", default_value="0.02"),
            DeclareLaunchArgument("planning_timeout", default_value="5.0"),
            DeclareLaunchArgument("velocity_scale", default_value="0.05"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.05"),
            DeclareLaunchArgument(
                "maximum_translation_nudge", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "maximum_rotation_nudge_deg", default_value="10.0"
            ),
            DeclareLaunchArgument("marker_scale", default_value="0.15"),
            DeclareLaunchArgument(
                "output_yaml", default_value="/tmp/massage_push_pose.yaml"
            ),
            simulation,
            rviz,
            tuner,
        ]
    )
