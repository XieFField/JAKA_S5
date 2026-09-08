import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    bringup = get_package_share_directory("massage_bringup")
    description = get_package_share_directory("massage_description")
    initial_positions = os.path.join(
        description, "config", "initial_positions_massage_standby.yaml"
    )
    robot_xacro = os.path.join(
        description, "urdf", "jaka_s5_massage.urdf.xacro"
    )
    semantic = os.path.join(
        description, "config", "jaka_s5_massage.srdf"
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_s5", package_name="jaka_s5_moveit_config")
        .robot_description(
            file_path=robot_xacro,
            mappings={
                "use_gazebo": "true",
                "use_rviz_sim": "false",
                "use_massage_head": "true",
                "initial_positions_file": initial_positions,
            },
        )
        .robot_description_semantic(file_path=semantic)
        .joint_limits(file_path="config/joint_limits_sim.yaml")
        .to_moveit_configs()
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "run_demo": "false",
            "allow_trajectory_execution": LaunchConfiguration("execute"),
            "initial_positions_file": initial_positions,
            "world_file": os.path.join(description, "worlds", "free_space.sdf"),
            "gazebo_extra_args": LaunchConfiguration("gazebo_extra_args"),
        }.items(),
    )
    node = Node(
        package="massage_bringup",
        executable="technique_trajectory_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "technique": LaunchConfiguration("technique"),
                "execution_environment": "simulation",
                "execute": typed("execute", bool),
                "velocity_scale": typed("velocity_scale", float),
                "acceleration_scale": typed("acceleration_scale", float),
                "planning_timeout": typed("planning_timeout", float),
                "execution_timeout_margin": typed(
                    "execution_timeout_margin", float
                ),
                "endpoint_joint_tolerance": typed(
                    "endpoint_joint_tolerance", float
                ),
                "endpoint_position_tolerance": typed(
                    "endpoint_position_tolerance", float
                ),
                "endpoint_orientation_tolerance": typed(
                    "endpoint_orientation_tolerance", float
                ),
                "press_stroke": typed("press_stroke", float),
                "press_cycle_duration": typed("press_cycle_duration", float),
                "press_cycles": typed("press_cycles", int),
                "knead_radius": typed("knead_radius", float),
                "knead_cycle_duration": typed("knead_cycle_duration", float),
                "knead_cycles": typed("knead_cycles", int),
            },
        ],
    )
    delayed_node = TimerAction(
        period=LaunchConfiguration("startup_delay"), actions=[node]
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason="technique test completed"))],
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("technique", default_value="press"),
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("gazebo_extra_args", default_value="-s"),
            DeclareLaunchArgument("startup_delay", default_value="7.0"),
            DeclareLaunchArgument("velocity_scale", default_value="0.10"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.05"),
            DeclareLaunchArgument("planning_timeout", default_value="8.0"),
            DeclareLaunchArgument(
                "execution_timeout_margin", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "endpoint_joint_tolerance", default_value="0.006"
            ),
            DeclareLaunchArgument(
                "endpoint_position_tolerance", default_value="0.006"
            ),
            DeclareLaunchArgument(
                "endpoint_orientation_tolerance", default_value="0.04"
            ),
            DeclareLaunchArgument("press_stroke", default_value="0.004"),
            DeclareLaunchArgument(
                "press_cycle_duration", default_value="2.0"
            ),
            DeclareLaunchArgument("press_cycles", default_value="2"),
            DeclareLaunchArgument("knead_radius", default_value="0.010"),
            DeclareLaunchArgument(
                "knead_cycle_duration", default_value="4.0"
            ),
            DeclareLaunchArgument("knead_cycles", default_value="2"),
            simulation,
            delayed_node,
            shutdown,
        ]
    )
