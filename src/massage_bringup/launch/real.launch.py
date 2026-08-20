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
        maximum_servo_step_num = int(
            LaunchConfiguration(
                "trajectory_maximum_servo_step_num"
            ).perform(context)
        )
        maximum_servo_samples = int(
            LaunchConfiguration("maximum_servo_samples").perform(context)
        )
        trajectory_goal_tolerance = float(
            LaunchConfiguration("trajectory_goal_tolerance").perform(context)
        )
        trajectory_goal_timeout = float(
            LaunchConfiguration("trajectory_goal_timeout").perform(context)
        )
        trajectory_feedback_period = float(
            LaunchConfiguration("trajectory_feedback_period").perform(context)
        )
        servo_filter_cutoff = float(
            LaunchConfiguration(
                "trajectory_servo_filter_cutoff_hz"
            ).perform(context)
        )
        maximum_queue_starvation = float(
            LaunchConfiguration(
                "trajectory_maximum_queue_starvation"
            ).perform(context)
        )
        maximum_consecutive_starvations = int(
            LaunchConfiguration(
                "trajectory_maximum_consecutive_starvations"
            ).perform(context)
        )
    except ValueError as error:
        raise RuntimeError("JAKA servo 调度参数必须是数值") from error
    if not 1 <= maximum_servo_step_num <= 50:
        raise RuntimeError(
            "trajectory_maximum_servo_step_num 必须在 [1, 50] 内"
        )
    if maximum_servo_samples <= 0:
        raise RuntimeError("maximum_servo_samples 必须大于零")
    if (
        not math.isfinite(trajectory_goal_tolerance)
        or trajectory_goal_tolerance <= 0.0
    ):
        raise RuntimeError("trajectory_goal_tolerance 必须为有限正数")
    if (
        not math.isfinite(trajectory_goal_timeout)
        or trajectory_goal_timeout <= 0.0
    ):
        raise RuntimeError("trajectory_goal_timeout 必须为有限正数")
    if (
        not math.isfinite(trajectory_feedback_period)
        or trajectory_feedback_period <= 0.0
    ):
        raise RuntimeError("trajectory_feedback_period 必须为有限正数")
    if not math.isfinite(servo_filter_cutoff) or servo_filter_cutoff < 0.0:
        raise RuntimeError("trajectory_servo_filter_cutoff_hz 不能为负数")
    if (
        not math.isfinite(maximum_queue_starvation)
        or maximum_queue_starvation < 0.0
    ):
        raise RuntimeError("trajectory_maximum_queue_starvation 不能为负数")
    if maximum_consecutive_starvations < 0:
        raise RuntimeError(
            "trajectory_maximum_consecutive_starvations 不能为负数"
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
        try:
            home_planning_attempts = int(
                LaunchConfiguration("home_planning_attempts").perform(context)
            )
            home_velocity_scale = float(
                LaunchConfiguration("home_velocity_scale").perform(context)
            )
            home_acceleration_scale = float(
                LaunchConfiguration("home_acceleration_scale").perform(context)
            )
            home_planning_timeout = float(
                LaunchConfiguration("home_planning_timeout").perform(context)
            )
            home_execution_timeout_margin = float(
                LaunchConfiguration(
                    "home_execution_timeout_margin"
                ).perform(context)
            )
            home_state_timeout = float(
                LaunchConfiguration("home_state_timeout").perform(context)
            )
            home_readiness_timeout = float(
                LaunchConfiguration("home_readiness_timeout").perform(context)
            )
            home_feedback_timeout = float(
                LaunchConfiguration("home_feedback_timeout").perform(context)
            )
            home_maximum_joint_travel = float(
                LaunchConfiguration(
                    "home_maximum_joint_travel"
                ).perform(context)
            )
            home_endpoint_tolerance = float(
                LaunchConfiguration("home_endpoint_tolerance").perform(context)
            )
        except ValueError as error:
            raise RuntimeError("自动回待机参数必须是数值") from error

        if not 1 <= home_planning_attempts <= 10:
            raise RuntimeError("home_planning_attempts 必须在 [1, 10] 内")
        if (
            not math.isfinite(home_velocity_scale)
            or not 0.0 < home_velocity_scale <= 1.0
        ):
            raise RuntimeError("home_velocity_scale 必须在 (0, 1] 内")
        if (
            not math.isfinite(home_acceleration_scale)
            or not 0.0 < home_acceleration_scale <= 1.0
        ):
            raise RuntimeError("home_acceleration_scale 必须在 (0, 1] 内")
        if (
            not math.isfinite(home_planning_timeout)
            or home_planning_timeout <= 0.0
        ):
            raise RuntimeError("home_planning_timeout 必须为有限正数")
        if (
            not math.isfinite(home_execution_timeout_margin)
            or home_execution_timeout_margin < 0.0
        ):
            raise RuntimeError("home_execution_timeout_margin 不能为负数")
        for name, value in (
            ("home_state_timeout", home_state_timeout),
            ("home_readiness_timeout", home_readiness_timeout),
            ("home_feedback_timeout", home_feedback_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise RuntimeError(f"{name} 必须为有限正数")
        if (
            not math.isfinite(home_maximum_joint_travel)
            or home_maximum_joint_travel <= 0.0
        ):
            raise RuntimeError("home_maximum_joint_travel 必须为有限正数")
        if (
            not math.isfinite(home_endpoint_tolerance)
            or home_endpoint_tolerance <= 0.0
            or home_endpoint_tolerance >= home_maximum_joint_travel
        ):
            raise RuntimeError(
                "home_endpoint_tolerance 必须为正数且小于最大关节行程"
            )
        if trajectory_goal_tolerance > home_endpoint_tolerance:
            raise RuntimeError(
                "trajectory_goal_tolerance 不能大于 "
                "home_endpoint_tolerance"
            )
        if trajectory_goal_timeout < home_execution_timeout_margin:
            raise RuntimeError(
                "trajectory_goal_timeout 不能小于 "
                "home_execution_timeout_margin，避免驱动内层提前中止"
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
            "trajectory_maximum_servo_step_num": ParameterValue(
                LaunchConfiguration("trajectory_maximum_servo_step_num"),
                value_type=int,
            ),
            "maximum_servo_samples": ParameterValue(
                LaunchConfiguration("maximum_servo_samples"), value_type=int
            ),
            "trajectory_feedback_period": ParameterValue(
                LaunchConfiguration("trajectory_feedback_period"), value_type=float
            ),
            "trajectory_servo_filter_cutoff_hz": ParameterValue(
                LaunchConfiguration("trajectory_servo_filter_cutoff_hz"),
                value_type=float,
            ),
            "trajectory_maximum_queue_starvation": ParameterValue(
                LaunchConfiguration("trajectory_maximum_queue_starvation"),
                value_type=float,
            ),
            "trajectory_maximum_consecutive_starvations": ParameterValue(
                LaunchConfiguration(
                    "trajectory_maximum_consecutive_starvations"
                ),
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
            "velocity_scale": LaunchConfiguration("home_velocity_scale"),
            "acceleration_scale": LaunchConfiguration(
                "home_acceleration_scale"
            ),
            "planning_timeout": LaunchConfiguration("home_planning_timeout"),
            "execution_timeout_margin": LaunchConfiguration(
                "home_execution_timeout_margin"
            ),
            "state_timeout": LaunchConfiguration("home_state_timeout"),
            "readiness_timeout": LaunchConfiguration("home_readiness_timeout"),
            "feedback_timeout": LaunchConfiguration("home_feedback_timeout"),
            "maximum_joint_travel": LaunchConfiguration(
                "home_maximum_joint_travel"
            ),
            "endpoint_tolerance": LaunchConfiguration(
                "home_endpoint_tolerance"
            ),
            "output_csv": LaunchConfiguration("home_output_csv"),
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
        DeclareLaunchArgument("home_velocity_scale", default_value="0.02"),
        DeclareLaunchArgument("home_acceleration_scale", default_value="0.02"),
        DeclareLaunchArgument("home_planning_timeout", default_value="5.0"),
        DeclareLaunchArgument(
            "home_execution_timeout_margin", default_value="15.0"
        ),
        DeclareLaunchArgument("home_state_timeout", default_value="30.0"),
        DeclareLaunchArgument("home_readiness_timeout", default_value="60.0"),
        DeclareLaunchArgument("home_feedback_timeout", default_value="1.0"),
        DeclareLaunchArgument("home_maximum_joint_travel", default_value="0.55"),
        DeclareLaunchArgument("home_endpoint_tolerance", default_value="0.002"),
        DeclareLaunchArgument("home_output_csv", default_value=""),
        DeclareLaunchArgument("ft_frame_id", default_value="Link_06"),
        DeclareLaunchArgument("ft_data_type", default_value="3"),
        DeclareLaunchArgument("trajectory_goal_tolerance", default_value="0.002"),
        DeclareLaunchArgument("trajectory_goal_timeout", default_value="15.0"),
        DeclareLaunchArgument(
            "trajectory_maximum_servo_step_num", default_value="50"
        ),
        DeclareLaunchArgument("maximum_servo_samples", default_value="50000"),
        DeclareLaunchArgument("trajectory_feedback_period", default_value="0.1"),
        DeclareLaunchArgument(
            "trajectory_servo_filter_cutoff_hz", default_value="0.5"
        ),
        DeclareLaunchArgument(
            "trajectory_maximum_queue_starvation", default_value="0.008"
        ),
        DeclareLaunchArgument(
            "trajectory_maximum_consecutive_starvations", default_value="1"
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
