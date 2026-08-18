import math
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
    try:
        servo_step_num = int(
            LaunchConfiguration("trajectory_servo_step_num").perform(context)
        )
        maximum_lateness = float(
            LaunchConfiguration("trajectory_maximum_lateness").perform(context)
        )
        maximum_consecutive_overruns = int(
            LaunchConfiguration(
                "trajectory_maximum_consecutive_overruns"
            ).perform(context)
        )
    except ValueError as error:
        raise RuntimeError("JAKA servo 调度参数必须是数值") from error
    if not 1 <= servo_step_num <= 50:
        raise RuntimeError("trajectory_servo_step_num 必须在 [1, 50] 内")
    if not math.isfinite(maximum_lateness) or maximum_lateness < 0.0:
        raise RuntimeError("trajectory_maximum_lateness 不能为负数")
    if maximum_consecutive_overruns < 0:
        raise RuntimeError(
            "trajectory_maximum_consecutive_overruns 不能为负数"
        )
    auto_home = LaunchConfiguration("auto_home").perform(context).lower() == "true"
    if auto_home:
        connect = LaunchConfiguration("connect").perform(context).lower() == "true"
        start_move_group = (
            LaunchConfiguration("start_move_group").perform(context).lower()
            == "true"
        )
        confirmed = (
            LaunchConfiguration("auto_home_confirmed").perform(context).lower()
            == "true"
        )
        if not connect or not start_move_group or not confirmed:
            raise RuntimeError(
                "auto_home:=true 要求 connect:=true、start_move_group:=true "
                "和 auto_home_confirmed:=true"
            )
    return []


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")
    description_share = get_package_share_directory("massage_description")
    robot_ip = LaunchConfiguration("robot_ip")
    connect = LaunchConfiguration("connect")
    start_move_group = LaunchConfiguration("start_move_group")
    use_rviz = LaunchConfiguration("use_rviz")
    auto_home = LaunchConfiguration("auto_home")

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
            "trajectory_goal_tolerance": ParameterValue(
                LaunchConfiguration("trajectory_goal_tolerance"), value_type=float
            ),
            "trajectory_goal_timeout": ParameterValue(
                LaunchConfiguration("trajectory_goal_timeout"), value_type=float
            ),
            "trajectory_servo_step_num": ParameterValue(
                LaunchConfiguration("trajectory_servo_step_num"), value_type=int
            ),
            "maximum_servo_samples": ParameterValue(
                LaunchConfiguration("maximum_servo_samples"), value_type=int
            ),
            "trajectory_feedback_period": ParameterValue(
                LaunchConfiguration("trajectory_feedback_period"), value_type=float
            ),
            "trajectory_maximum_lateness": ParameterValue(
                LaunchConfiguration("trajectory_maximum_lateness"),
                value_type=float,
            ),
            "trajectory_maximum_consecutive_overruns": ParameterValue(
                LaunchConfiguration("trajectory_maximum_consecutive_overruns"),
                value_type=int,
            ),
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
    home_task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "return_home_real.launch.py")
        ),
        launch_arguments={
            "execute": "true",
            "parameters_confirmed": LaunchConfiguration("auto_home_confirmed"),
            "planning_attempts": LaunchConfiguration("home_planning_attempts"),
            "state_timeout": LaunchConfiguration("home_state_timeout"),
            "readiness_timeout": LaunchConfiguration("home_readiness_timeout"),
            "maximum_joint_travel": LaunchConfiguration(
                "home_maximum_joint_travel"
            ),
        }.items(),
        condition=IfCondition(auto_home),
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
        DeclareLaunchArgument("auto_home", default_value="false"),
        DeclareLaunchArgument("auto_home_confirmed", default_value="false"),
        DeclareLaunchArgument("home_planning_attempts", default_value="3"),
        DeclareLaunchArgument("home_state_timeout", default_value="30.0"),
        DeclareLaunchArgument("home_readiness_timeout", default_value="60.0"),
        DeclareLaunchArgument("home_maximum_joint_travel", default_value="3.5"),
        DeclareLaunchArgument("ft_frame_id", default_value="Link_06"),
        DeclareLaunchArgument("ft_data_type", default_value="3"),
        DeclareLaunchArgument("trajectory_goal_tolerance", default_value="0.01"),
        DeclareLaunchArgument("trajectory_goal_timeout", default_value="2.0"),
        DeclareLaunchArgument("trajectory_servo_step_num", default_value="4"),
        DeclareLaunchArgument("maximum_servo_samples", default_value="50000"),
        DeclareLaunchArgument("trajectory_feedback_period", default_value="0.1"),
        DeclareLaunchArgument(
            "trajectory_maximum_lateness", default_value="0.008"
        ),
        DeclareLaunchArgument(
            "trajectory_maximum_consecutive_overruns", default_value="1"
        ),
        OpaqueFunction(function=_validate_arguments),
        LogInfo(
            msg=[
                "JAKA real bringup: connect=", connect,
                ", auto_home=", auto_home,
                ". 本 launch 不会自动登录、上电或使能。",
            ]
        ),
        robot_state_publisher,
        driver,
        diagnostics,
        move_group,
        rviz,
        home_task,
    ])
