import os
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def stop_partitioned_gazebo(event, context):
    del event
    partition = LaunchConfiguration("gazebo_partition").perform(context)
    stopped = []
    proc_root = "/proc"
    for entry in os.scandir(proc_root):
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            with open(
                os.path.join(proc_root, entry.name, "environ"), "rb"
            ) as stream:
                environment = stream.read().split(b"\0")
            expected = {
                f"IGN_PARTITION={partition}".encode(),
                f"GZ_PARTITION={partition}".encode(),
            }
            if expected.isdisjoint(environment):
                continue
            with open(
                os.path.join(proc_root, entry.name, "cmdline"), "rb"
            ) as stream:
                command = stream.read().replace(b"\0", b" ")
            if b"ign gazebo" not in command and b"gz sim" not in command:
                continue
            os.kill(pid, signal.SIGTERM)
            stopped.append(pid)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return [LogInfo(msg=(
        f"Gazebo partition cleanup: partition={partition}, pids={stopped}"
    ))]


def generate_launch_description():
    use_massage_head = LaunchConfiguration("use_massage_head")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    world_file = LaunchConfiguration("world_file")
    gazebo_extra_args = LaunchConfiguration("gazebo_extra_args")
    gazebo_partition = LaunchConfiguration("gazebo_partition")

    description_share = get_package_share_directory("massage_description")
    robot_xacro = os.path.join(
        description_share,
        "urdf",
        "jaka_s5_massage.urdf.xacro",
    )
    robot_description = ParameterValue(
        Command([
            "xacro ",
            robot_xacro,
            " use_gazebo:=true",
            " use_rviz_sim:=false",
            " use_massage_head:=",
            use_massage_head,
            " initial_positions_file:=",
            initial_positions_file,
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
            "gz_args": [world_file, " -r ", gazebo_extra_args],
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
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=os.path.join(
                get_package_share_directory("jaka_s5_moveit_config"),
                "config",
                "initial_positions.yaml",
            ),
            description="Joint positions used when Gazebo creates the robot",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=os.path.join(
                description_share,
                "worlds",
                "ft_contact_test.sdf",
            ),
            description="Gazebo world file; FT contact demo keeps its contact pad",
        ),
        DeclareLaunchArgument(
            "gazebo_extra_args",
            default_value="",
            description="Additional Ignition Gazebo arguments, such as -s",
        ),
        DeclareLaunchArgument(
            "gazebo_partition",
            default_value=f"massage_sim_{os.getpid()}",
            description="Per-launch Gazebo transport partition",
        ),
        SetEnvironmentVariable("IGN_PARTITION", gazebo_partition),
        SetEnvironmentVariable("GZ_PARTITION", gazebo_partition),
        RegisterEventHandler(OnShutdown(on_shutdown=stop_partitioned_gazebo)),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        wrench_bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),
        spawn_controllers,
    ])
