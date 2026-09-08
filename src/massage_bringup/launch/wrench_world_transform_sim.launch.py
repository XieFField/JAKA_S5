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

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "ft_sensor_sim.launch.py")
        ),
        launch_arguments={
            "use_massage_head": "true",
            "initial_positions_file": os.path.join(
                description,
                "config",
                "initial_positions_massage_standby.yaml",
            ),
            "world_file": os.path.join(
                description, "worlds", "free_space.sdf"
            ),
            "gazebo_extra_args": LaunchConfiguration("gazebo_extra_args"),
        }.items(),
    )

    transformer = Node(
        package="massage_motion",
        executable="wrench_frame_transformer",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/massage/ft_sensor/wrench_raw",
            "output_topic": "/massage/ft_sensor/wrench_world",
            # Gazebo 使用 scoped sensor 名作为消息 frame_id；其测量原点
            # 与 ft_sensor_joint 的 child link 重合，因此映射到真实 TF link。
            "source_frame_override": "massage_head_link",
            "expression_frame": "world",
            "reference_point_frame": "massage_tool_tip",
            "output_frame": "massage_tool_tip_world_aligned",
            "validation_sample_count": typed(
                "validation_sample_count", int
            ),
            "validation_timeout": typed("validation_timeout", float),
        }],
    )
    delayed_transformer = TimerAction(
        period=LaunchConfiguration("startup_delay"), actions=[transformer]
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=transformer,
            on_exit=[EmitEvent(event=Shutdown(reason="wrench test completed"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gazebo_extra_args", default_value="-s"),
        DeclareLaunchArgument("startup_delay", default_value="5.0"),
        DeclareLaunchArgument("validation_sample_count", default_value="50"),
        DeclareLaunchArgument("validation_timeout", default_value="10.0"),
        simulation,
        delayed_transformer,
        shutdown,
    ])
