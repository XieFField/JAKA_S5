import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
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
    controller_config = os.path.join(
        bringup, "config", "compliance_controllers_world_z.yaml"
    )
    initial_positions = os.path.join(
        description, "config", "initial_positions_massage_standby.yaml"
    )
    robot_xacro = os.path.join(
        description, "urdf", "jaka_s5_massage.urdf.xacro"
    )
    semantic_file = os.path.join(
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
        .robot_description_semantic(file_path=semantic_file)
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
            "allow_trajectory_execution": "true",
            "initial_positions_file": initial_positions,
            "world_file": os.path.join(
                description, "worlds", "massage_task_test.sdf"
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
    demo = Node(
        package="massage_task",
        executable="massage_task_sim_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "execution_mode": LaunchConfiguration("execution_mode"),
                "technique": LaunchConfiguration("technique"),
                "planning_attempts": typed("planning_attempts", int),
                "simulated_contact_force_z": typed(
                    "simulated_contact_force_z", float
                ),
                "simulated_contact_ramp_duration": typed(
                    "simulated_contact_ramp_duration", float
                ),
                "target_normal_force": typed("target_normal_force", float),
                "maximum_joint_displacement": typed(
                    "maximum_joint_displacement", float
                ),
                "maximum_linear_displacement": typed(
                    "maximum_linear_displacement", float
                ),
                "work_ready_clearance": typed("work_ready_clearance", float),
                "maximum_tool_axis_error_degrees": typed(
                    "maximum_tool_axis_error_degrees", float
                ),
                "push_length": typed("push_length", float),
                "push_repetitions": typed("push_repetitions", int),
                "press_cycles": typed("press_cycles", int),
                "knead_cycles": typed("knead_cycles", int),
            },
        ],
    )
    delayed_demo = TimerAction(
        period=LaunchConfiguration("startup_delay"), actions=[demo]
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=demo,
            on_exit=[EmitEvent(event=Shutdown(reason="massage task test completed"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("simulation_domain_id", default_value="71"),
        DeclareLaunchArgument("execution_mode", default_value="compliant_contact"),
        DeclareLaunchArgument("technique", default_value="push"),
        DeclareLaunchArgument("planning_attempts", default_value="5"),
        DeclareLaunchArgument("simulated_contact_force_z", default_value="-0.25"),
        DeclareLaunchArgument("simulated_contact_ramp_duration", default_value="1.0"),
        DeclareLaunchArgument("target_normal_force", default_value="0.20"),
        DeclareLaunchArgument("maximum_joint_displacement", default_value="0.75"),
        DeclareLaunchArgument("maximum_linear_displacement", default_value="0.12"),
        DeclareLaunchArgument("work_ready_clearance", default_value="0.05"),
        DeclareLaunchArgument(
            "maximum_tool_axis_error_degrees", default_value="3.0"
        ),
        DeclareLaunchArgument("push_length", default_value="0.08"),
        DeclareLaunchArgument("push_repetitions", default_value="3"),
        DeclareLaunchArgument("press_cycles", default_value="3"),
        DeclareLaunchArgument("knead_cycles", default_value="3"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("gazebo_extra_args", default_value="-s"),
        DeclareLaunchArgument("startup_delay", default_value="9.0"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("simulation_domain_id")
        ),
        simulation,
        ft_broadcaster,
        admittance,
        transformer,
        delayed_demo,
        shutdown,
    ])
