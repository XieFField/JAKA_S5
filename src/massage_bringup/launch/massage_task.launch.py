import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def validate_environment(context):
    value = LaunchConfiguration("execution_environment").perform(context)
    if value not in ("simulation", "real"):
        raise RuntimeError(
            "execution_environment must be 'simulation' or 'real'"
        )
    return []


def generate_launch_description():
    bringup = get_package_share_directory("massage_bringup")
    common_arguments = {
        "execution_mode": LaunchConfiguration("execution_mode"),
        "technique": LaunchConfiguration("technique"),
        "planning_attempts": LaunchConfiguration("planning_attempts"),
        "push_length": LaunchConfiguration("push_length"),
        "push_repetitions": LaunchConfiguration("push_repetitions"),
        "press_cycles": LaunchConfiguration("press_cycles"),
        "knead_cycles": LaunchConfiguration("knead_cycles"),
    }
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "massage_task_sim.launch.py")
        ),
        condition=LaunchConfigurationEquals(
            "execution_environment", "simulation"
        ),
        launch_arguments={
            **common_arguments,
            "use_rviz": LaunchConfiguration("use_rviz"),
            "gazebo_extra_args": LaunchConfiguration("gazebo_extra_args"),
            "startup_delay": LaunchConfiguration("startup_delay"),
            "simulation_domain_id": LaunchConfiguration("simulation_domain_id"),
        }.items(),
    )
    real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "massage_task_real.launch.py")
        ),
        condition=LaunchConfigurationEquals("execution_environment", "real"),
        launch_arguments={
            **common_arguments,
            "execute": LaunchConfiguration("execute"),
            "parameters_confirmed": LaunchConfiguration(
                "parameters_confirmed"
            ),
            "contact_x": LaunchConfiguration("contact_x"),
            "contact_y": LaunchConfiguration("contact_y"),
            "contact_z": LaunchConfiguration("contact_z"),
            "free_space_velocity_scale": LaunchConfiguration(
                "free_space_velocity_scale"
            ),
            "technique_velocity_scale": LaunchConfiguration(
                "technique_velocity_scale"
            ),
            "technique_acceleration_scale": LaunchConfiguration(
                "technique_acceleration_scale"
            ),
            "planning_timeout": LaunchConfiguration("planning_timeout"),
            "execution_timeout_margin": LaunchConfiguration(
                "execution_timeout_margin"
            ),
            "robot_state_timeout": LaunchConfiguration(
                "robot_state_timeout"
            ),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("execution_environment", default_value="simulation"),
        DeclareLaunchArgument("execution_mode", default_value="plan_only"),
        DeclareLaunchArgument("technique", default_value="push"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("parameters_confirmed", default_value="false"),
        DeclareLaunchArgument("planning_attempts", default_value="5"),
        DeclareLaunchArgument("push_length", default_value="0.08"),
        DeclareLaunchArgument("push_repetitions", default_value="3"),
        DeclareLaunchArgument("press_cycles", default_value="3"),
        DeclareLaunchArgument("knead_cycles", default_value="3"),
        DeclareLaunchArgument("contact_x", default_value="-0.471238630147741"),
        DeclareLaunchArgument("contact_y", default_value="0.156275068796867"),
        DeclareLaunchArgument("contact_z", default_value="0.281381721182422"),
        DeclareLaunchArgument("free_space_velocity_scale", default_value="0.05"),
        DeclareLaunchArgument("technique_velocity_scale", default_value="0.02"),
        DeclareLaunchArgument("technique_acceleration_scale", default_value="0.02"),
        DeclareLaunchArgument("planning_timeout", default_value="60.0"),
        DeclareLaunchArgument("execution_timeout_margin", default_value="15.0"),
        DeclareLaunchArgument("robot_state_timeout", default_value="1.0"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("gazebo_extra_args", default_value="-s"),
        DeclareLaunchArgument("startup_delay", default_value="9.0"),
        DeclareLaunchArgument("simulation_domain_id", default_value="71"),
        OpaqueFunction(function=validate_environment),
        simulation,
        real,
    ])
