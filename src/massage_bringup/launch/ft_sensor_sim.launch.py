import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_massage_head = LaunchConfiguration("use_massage_head")

    description_share = get_package_share_directory("massage_description")
    robot_xacro = os.path.join(
        description_share,
        "urdf",
        "jaka_s5_massage.urdf.xacro",
    )
    world_path = os.path.join(
        description_share,
        "worlds",
        "ft_contact_test.sdf",
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            robot_xacro,
            " use_gazebo:=true",
            " use_rviz_sim:=false",
            " use_massage_head:=",
            use_massage_head,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": world_path + " -r",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # massage_head.xacro 完成后，Gazebo sensor 会向固定话题发布 gz Wrench。
    # 使用 WrenchStamped 是为了保留时间戳和传感器坐标系。
    wrench_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="massage_ft_bridge",
        output="screen",
        arguments=[
            "/massage/ft_sensor/wrench"
            "@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench"
        ],
        remappings=[
            (
                "/massage/ft_sensor/wrench",
                "/massage/ft_sensor/wrench_raw",
            )
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "jaka_s5",
            "-topic",
            "/robot_description",
            "-z",
            "0.06",
        ],
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "jaka_s5_controller",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_massage_head",
            default_value="false",
            description=(
                "Enable the exercise massage head and force-torque sensor"
            ),
        ),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        wrench_bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),
        spawn_controllers,
    ])
