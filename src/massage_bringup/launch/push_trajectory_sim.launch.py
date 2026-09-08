import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")
    description_share = get_package_share_directory("massage_description")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
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
                "initial_positions_file": initial_positions_file,
            },
        )
        .robot_description_semantic(file_path=semantic_file)
        .joint_limits(file_path="config/joint_limits_sim.yaml")
        .to_moveit_configs()
    )

    simulation = GroupAction(
        condition=IfCondition(LaunchConfiguration("start_simulation")),
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "sim.launch.py")
                ),
                launch_arguments={
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "world_file": LaunchConfiguration("world_file"),
                    "run_demo": "false",
                    "gazebo_extra_args": LaunchConfiguration(
                        "gazebo_extra_args"
                    ),
                    "allow_trajectory_execution": LaunchConfiguration(
                        "execute"
                    ),
                    "initial_positions_file": initial_positions_file,
                }.items(),
            )
        ],
    )

    push_node = Node(
        package="massage_bringup",
        executable="push_trajectory_sim",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "config_file": LaunchConfiguration("config_file"),
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
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
                "execution_timeout_margin": float_parameter(
                    "execution_timeout_margin"
                ),
                "joint_state_timeout": float_parameter("joint_state_timeout"),
                "maximum_joint_travel": float_parameter(
                    "maximum_joint_travel"
                ),
                "endpoint_tolerance": float_parameter("endpoint_tolerance"),
                "link_height_gate_enabled": ParameterValue(
                    LaunchConfiguration("link_height_gate_enabled"),
                    value_type=bool,
                ),
                "diagnostic_link_name": LaunchConfiguration(
                    "diagnostic_link_name"
                ),
                "minimum_link_height": float_parameter("minimum_link_height"),
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
                "workflow_mode": LaunchConfiguration("workflow_mode"),
                "test_mode": LaunchConfiguration("test_mode"),
                "progressive_checkpoint_fractions": ParameterValue(
                    LaunchConfiguration("progressive_checkpoint_fractions"),
                    value_type=str,
                ),
                "progressive_maximum_checkpoint_count": ParameterValue(
                    LaunchConfiguration(
                        "progressive_maximum_checkpoint_count"
                    ),
                    value_type=int,
                ),
                "progressive_maximum_segment_joint_travel": float_parameter(
                    "progressive_maximum_segment_joint_travel"
                ),
                "progressive_joint_continuity_tolerance": float_parameter(
                    "progressive_joint_continuity_tolerance"
                ),
                "progressive_minimum_direction_observability": float_parameter(
                    "progressive_minimum_direction_observability"
                ),
                "cartesian_position_tolerance": float_parameter(
                    "cartesian_position_tolerance"
                ),
                "cartesian_orientation_tolerance": float_parameter(
                    "cartesian_orientation_tolerance"
                ),
                "maximum_lin_transverse_error": float_parameter(
                    "maximum_lin_transverse_error"
                ),
                "maximum_lin_height_error": float_parameter(
                    "maximum_lin_height_error"
                ),
                "maximum_lin_orientation_error": float_parameter(
                    "maximum_lin_orientation_error"
                ),
                "maximum_lin_longitudinal_overshoot": float_parameter(
                    "maximum_lin_longitudinal_overshoot"
                ),
            },
        ],
    )

    stage_b_node = TimerAction(
        period=LaunchConfiguration("startup_delay"),
        actions=[push_node],
    )

    shutdown_on_completion = RegisterEventHandler(
        OnProcessExit(
            target_action=push_node,
            on_exit=[
                LogInfo(
                    msg=(
                        "Stage C-B functional node has exited; subsequent "
                        "MoveGroup/Ignition messages belong to infrastructure "
                        "cleanup."
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="push trajectory simulation test completed"
                    )
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration("shutdown_on_completion")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("start_simulation", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_extra_args", default_value=""),
            DeclareLaunchArgument(
                "shutdown_on_completion", default_value="true"
            ),
            DeclareLaunchArgument("startup_delay", default_value="7.0"),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=os.path.join(
                    description_share,
                    "config",
                    "initial_positions_massage_standby.yaml",
                ),
                description=(
                    "Push-test simulation initial state; defaults to the "
                    "business standby joint target"
                ),
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(
                    get_package_share_directory("massage_description"),
                    "worlds",
                    "free_space.sdf",
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(
                    bringup_share,
                    "config",
                    "push_contact_sequence_stage_b.yaml",
                ),
            ),
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
            DeclareLaunchArgument("velocity_scale", default_value="0.05"),
            DeclareLaunchArgument(
                "acceleration_scale", default_value="0.05"
            ),
            DeclareLaunchArgument("planning_timeout", default_value="5.0"),
            DeclareLaunchArgument(
                "execution_timeout_margin", default_value="5.0"
            ),
            DeclareLaunchArgument("joint_state_timeout", default_value="3.0"),
            DeclareLaunchArgument(
                "maximum_joint_travel", default_value="3.5"
            ),
            DeclareLaunchArgument(
                "endpoint_tolerance", default_value="0.005"
            ),
            DeclareLaunchArgument(
                "link_height_gate_enabled", default_value="true"
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
            DeclareLaunchArgument(
                "workflow_mode",
                default_value="full_push",
                description="full_push or progressive_ptp",
            ),
            DeclareLaunchArgument("test_mode", default_value="normal"),
            DeclareLaunchArgument(
                "progressive_checkpoint_fractions",
                default_value="[0.10, 0.25, 0.50, 1.00]",
            ),
            DeclareLaunchArgument(
                "progressive_maximum_checkpoint_count", default_value="64"
            ),
            DeclareLaunchArgument(
                "progressive_maximum_segment_joint_travel",
                default_value="0.30",
            ),
            DeclareLaunchArgument(
                "progressive_joint_continuity_tolerance",
                default_value="0.000001",
            ),
            DeclareLaunchArgument(
                "progressive_minimum_direction_observability",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "cartesian_position_tolerance", default_value="0.005"
            ),
            DeclareLaunchArgument(
                "cartesian_orientation_tolerance", default_value="0.03"
            ),
            DeclareLaunchArgument(
                "maximum_lin_transverse_error", default_value="0.005"
            ),
            DeclareLaunchArgument(
                "maximum_lin_height_error", default_value="0.003"
            ),
            DeclareLaunchArgument(
                "maximum_lin_orientation_error", default_value="0.03"
            ),
            DeclareLaunchArgument(
                "maximum_lin_longitudinal_overshoot",
                default_value="0.005",
            ),
            simulation,
            stage_b_node,
            shutdown_on_completion,
        ]
    )
