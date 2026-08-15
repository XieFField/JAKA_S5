import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_arguments(context):
    robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
    if not robot_ip:
        raise RuntimeError(
            "robot_ip 必须显式提供，例如 robot_ip:=192.168.x.x"
        )
    return []


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")
    description_share = get_package_share_directory("massage_description")
    robot_ip = LaunchConfiguration("robot_ip")
    connect = LaunchConfiguration("connect")
    start_move_group = LaunchConfiguration("start_move_group")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                description_share,
                "urdf",
                "jaka_s5_massage.urdf.xacro",
            ),
            " use_gazebo:=false use_rviz_sim:=false use_massage_head:=true",
        ]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    driver = Node(
        package="jaka_driver",
        executable="jaka_driver",
        name="jaka_driver",
        output="screen",
        parameters=[{
            "ip": robot_ip,
            "ft_frame_id": LaunchConfiguration("ft_frame_id"),
            "ft_data_type": LaunchConfiguration("ft_data_type"),
        }],
        condition=IfCondition(connect),
    )
    diagnostics = Node(
        package="massage_jaka",
        executable="hardware_readiness_node",
        output="screen",
        parameters=[{
            "world_frame": "world",
            "tool_frame": "massage_tool_tip",
            "trajectory_action":
                "/jaka_s5_controller/follow_joint_trajectory",
        }],
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "move_group_real.launch.py")
        ),
        condition=IfCondition(start_move_group),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "moveit_rviz_real.launch.py")
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="JAKA controller IP; no repository default is allowed",
        ),
        DeclareLaunchArgument("connect", default_value="false"),
        DeclareLaunchArgument("start_move_group", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("ft_frame_id", default_value="Link_06"),
        DeclareLaunchArgument("ft_data_type", default_value="3"),
        OpaqueFunction(function=_validate_arguments),
        LogInfo(
            msg=[
                "JAKA real bringup: connect=", connect,
                ". 本 launch 不会自动登录、上电、使能或运动。",
            ]
        ),
        robot_state_publisher,
        driver,
        diagnostics,
        move_group,
        rviz,
    ])
