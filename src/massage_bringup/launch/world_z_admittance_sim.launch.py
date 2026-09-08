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


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    bringup = get_package_share_directory("massage_bringup")
    description = get_package_share_directory("massage_description")
    controller_config = os.path.join(
        bringup, "config", "compliance_controllers_world_z.yaml"
    )
    initial_positions = os.path.join(
        description, "config", "initial_positions_massage_standby.yaml"
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "run_demo": "false",
            "allow_trajectory_execution": "true",
            "initial_positions_file": initial_positions,
            "world_file": os.path.join(
                description, "worlds", "massage_bed_contact_test.sdf"
            ),
            "gazebo_extra_args": LaunchConfiguration("gazebo_extra_args"),
        }.items(),
    )

    ft_broadcaster = TimerAction(
        period=5.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "massage_ft_broadcaster",
                "--controller-manager", "/controller_manager",
                "--controller-type",
                "force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster",
                "--param-file", controller_config,
                "--controller-manager-timeout", "10",
            ],
        )],
    )
    admittance = TimerAction(
        period=6.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "massage_admittance_controller",
                "--controller-manager", "/controller_manager",
                "--controller-type", "admittance_controller/AdmittanceController",
                "--param-file", controller_config,
                "--inactive",
                "--controller-manager-timeout", "10",
            ],
        )],
    )
    transformer = TimerAction(
        period=5.0,
        actions=[Node(
            package="massage_motion",
            executable="wrench_frame_transformer",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "input_topic": "/massage/ft_sensor/wrench_raw",
                "output_topic": "/massage/ft_sensor/wrench_world",
                "source_frame_override": "massage_head_link",
                "expression_frame": "world",
                "reference_point_frame": "massage_tool_tip",
                "output_frame": "massage_tool_tip_world_aligned",
                "validation_sample_count": 0,
            }],
        )],
    )
    simulated_wrench_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="simulated_contact_wrench_bridge",
        output="screen",
        arguments=[
            "/world/massage_bed_contact_test/wrench/persistent"
            "@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
            "/world/massage_bed_contact_test/wrench/clear"
            "@ros_gz_interfaces/msg/Entity]gz.msgs.Entity",
        ],
    )
    demo_node = Node(
        package="massage_task",
        executable="world_z_admittance_demo",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "precontact_z": typed("precontact_z", float),
            "search_depth": typed("search_depth", float),
            "contact_threshold": typed("contact_threshold", float),
            "force_limit": typed("force_limit", float),
            "hold_duration": typed("hold_duration", float),
            "skip_approach": False,
            "enable_simulated_contact_wrench": True,
            "simulated_contact_trigger_fraction": 0.35,
            "simulated_contact_force_z": -1.0,
            "simulated_contact_entity": "jaka_s5::massage_head_link",
        }],
    )
    delayed_demo = TimerAction(
        period=LaunchConfiguration("startup_delay"), actions=[demo_node]
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=demo_node,
            on_exit=[EmitEvent(event=Shutdown(reason="admittance test completed"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gazebo_extra_args", default_value="-s"),
        DeclareLaunchArgument("startup_delay", default_value="9.0"),
        DeclareLaunchArgument("precontact_z", default_value="0.281381721182422"),
        DeclareLaunchArgument("search_depth", default_value="0.020"),
        DeclareLaunchArgument("contact_threshold", default_value="0.15"),
        DeclareLaunchArgument("force_limit", default_value="5.0"),
        DeclareLaunchArgument("hold_duration", default_value="1.0"),
        simulation,
        ft_broadcaster,
        admittance,
        transformer,
        simulated_wrench_bridge,
        delayed_demo,
        shutdown,
    ])
