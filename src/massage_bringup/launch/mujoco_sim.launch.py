import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_moveit = LaunchConfiguration("use_moveit")
    use_servo = LaunchConfiguration("use_servo")
    use_keyboard = LaunchConfiguration("use_keyboard")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mujoco_viewer = LaunchConfiguration("use_mujoco_viewer")
    show_mujoco_left_ui = LaunchConfiguration("show_mujoco_left_ui")
    show_mujoco_right_ui = LaunchConfiguration("show_mujoco_right_ui")
    use_tcp_control_ball = LaunchConfiguration("use_tcp_control_ball")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    physics_steps_per_update = LaunchConfiguration("physics_steps_per_update")
    realtime_factor = LaunchConfiguration("realtime_factor")
    wrench_frame_id = LaunchConfiguration("wrench_frame_id")
    description_share = get_package_share_directory("massage_description")
    bringup_share = get_package_share_directory("massage_bringup")
    robot_xacro = os.path.join(
        description_share,
        "urdf",
        "jaka_s5_massage.urdf.xacro",
    )
    semantic_file = os.path.join(
        description_share,
        "config",
        "jaka_s5_massage.srdf",
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_s5", package_name="jaka_s5_moveit_config")
        .robot_description(
            file_path=robot_xacro,
            mappings={
                "use_gazebo": "false",
                "use_rviz_sim": "false",
                "use_massage_head": "true",
                "initial_positions_file": initial_positions_file,
            },
        )
        .robot_description_semantic(file_path=semantic_file)
        .joint_limits(file_path="config/joint_limits_sim.yaml")
        .to_moveit_configs()
    )
    servo_config_path = os.path.join(
        bringup_share,
        "config",
        "mujoco_servo.yaml",
    )
    with open(servo_config_path, "r", encoding="utf-8") as stream:
        servo_config = {"moveit_servo": yaml.safe_load(stream)}

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "move_group_mujoco.launch.py")
        ),
        condition=IfCondition(use_moveit),
        launch_arguments={
            "initial_positions_file": initial_positions_file,
            "use_gazebo": "false",
        }.items(),
    )
    prone_planning_scene = Node(
        package="massage_bringup",
        executable="prone_planning_scene",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("use_prone_mannequin"),
            "'.lower() == 'true' and '", use_moveit,
            "'.lower() == 'true'",
        ])),
        parameters=[{
            "config_path": ParameterValue(
                LaunchConfiguration("config_path"), value_type=str,
            ),
        }],
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "moveit_rviz_mujoco.launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "initial_positions_file": initial_positions_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("use_servo", default_value="true"),
        DeclareLaunchArgument("use_keyboard", default_value="true"),
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("use_prone_mannequin", default_value="false"),
        DeclareLaunchArgument("use_monitor", default_value="false"),
        DeclareLaunchArgument("plot_group", default_value="joint"),
        DeclareLaunchArgument("plot_joint", default_value="1"),
        DeclareLaunchArgument("show_contact_points", default_value="false"),
        DeclareLaunchArgument("show_contact_forces", default_value="false"),
        DeclareLaunchArgument("show_collision_proxies", default_value="false"),
        DeclareLaunchArgument("show_site_frames", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_mujoco_viewer", default_value="true"),
        DeclareLaunchArgument("show_mujoco_left_ui", default_value="true"),
        DeclareLaunchArgument("show_mujoco_right_ui", default_value="true"),
        DeclareLaunchArgument("use_tcp_control_ball", default_value="true"),
        DeclareLaunchArgument("tcp_control_linear_gain", default_value="6.0"),
        DeclareLaunchArgument("tcp_control_maximum_speed", default_value="0.50"),
        DeclareLaunchArgument("tcp_control_position_tolerance", default_value="0.001"),
        DeclareLaunchArgument("tcp_control_maximum_distance", default_value="0.30"),
        DeclareLaunchArgument("joint_control_maximum_torque", default_value="5.0"),
        DeclareLaunchArgument(
            "joint_2_control_maximum_torque", default_value="13.37",
        ),
        DeclareLaunchArgument(
            "joint_3_control_maximum_torque", default_value="3.510",
        ),
        DeclareLaunchArgument(
            "joint_4_control_maximum_torque", default_value="0.1158",
        ),
        DeclareLaunchArgument(
            "joint_5_control_maximum_torque", default_value="0.02179",
        ),
        DeclareLaunchArgument(
            "joint_6_control_maximum_torque", default_value="0.0003014",
        ),
        DeclareLaunchArgument(
            "joint_mouse_control_maximum_torque", default_value="50.0",
        ),
        DeclareLaunchArgument(
            "teach_trajectory_file",
            default_value="log/mujoco/teach_trajectory.yaml",
        ),
        DeclareLaunchArgument(
            "teach_replay_maximum_joint_speed", default_value="0.4",
        ),
        DeclareLaunchArgument(
            "teach_replay_minimum_segment_duration", default_value="0.2",
        ),
        DeclareLaunchArgument("tcp_diagnostic_trace_length", default_value="500"),
        DeclareLaunchArgument("tcp_diagnostic_publish_period", default_value="0.05"),
        DeclareLaunchArgument("surface_follow_approach_x", default_value="0.0"),
        DeclareLaunchArgument("surface_follow_approach_y", default_value="-1.0"),
        DeclareLaunchArgument("surface_follow_approach_z", default_value="0.0"),
        DeclareLaunchArgument("surface_follow_tangent_x", default_value="0.02"),
        DeclareLaunchArgument("surface_follow_tangent_y", default_value="0.0"),
        DeclareLaunchArgument("surface_follow_tangent_z", default_value="0.0"),
        DeclareLaunchArgument("surface_follow_target_force", default_value="2.0"),
        DeclareLaunchArgument("surface_follow_force_gain", default_value="0.01"),
        DeclareLaunchArgument("surface_follow_approach_speed", default_value="0.01"),
        DeclareLaunchArgument(
            "surface_follow_maximum_normal_speed", default_value="0.02",
        ),
        DeclareLaunchArgument("surface_follow_duration", default_value="3.0"),
        DeclareLaunchArgument(
            "surface_follow_approach_timeout", default_value="10.0",
        ),
        DeclareLaunchArgument(
            "surface_follow_maximum_approach_distance", default_value="0.05",
        ),
        DeclareLaunchArgument(
            "surface_follow_force_tolerance", default_value="0.2",
        ),
        DeclareLaunchArgument("physics_steps_per_update", default_value="10"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        DeclareLaunchArgument(
            "wrench_frame_id",
            default_value="massage_head_link",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=os.path.join(
                get_package_share_directory("massage_mujoco"),
                "config",
                "initial_positions.yaml",
            ),
        ),
        Node(
            package="massage_mujoco",
            executable="mujoco_node",
            output="screen",
            parameters=[{
                **{
                    name: ParameterValue(LaunchConfiguration(name), value_type=bool)
                    for name in (
                        "use_prone_mannequin",
                        "show_contact_points", "show_contact_forces",
                        "show_collision_proxies", "show_site_frames",
                    )
                },
                "config_path": ParameterValue(
                    LaunchConfiguration("config_path"), value_type=str,
                ),
                "initial_positions_file": ParameterValue(
                    initial_positions_file, value_type=str,
                ),
                "teach_trajectory_file": ParameterValue(
                    LaunchConfiguration("teach_trajectory_file"), value_type=str,
                ),
                "tcp_diagnostic_trace_length": ParameterValue(
                    LaunchConfiguration("tcp_diagnostic_trace_length"),
                    value_type=int,
                ),
                "physics_steps_per_update": ParameterValue(
                    physics_steps_per_update,
                    value_type=int,
                ),
                "realtime_factor": ParameterValue(
                    realtime_factor,
                    value_type=float,
                ),
                "wrench_frame_id": wrench_frame_id,
                "use_mujoco_viewer": ParameterValue(
                    use_mujoco_viewer,
                    value_type=bool,
                ),
                "show_mujoco_left_ui": ParameterValue(
                    show_mujoco_left_ui,
                    value_type=bool,
                ),
                "show_mujoco_right_ui": ParameterValue(
                    show_mujoco_right_ui,
                    value_type=bool,
                ),
                "use_tcp_control_ball": ParameterValue(
                    use_tcp_control_ball,
                    value_type=bool,
                ),
                **{
                    name: ParameterValue(LaunchConfiguration(name), value_type=float)
                    for name in (
                        "tcp_control_linear_gain",
                        "tcp_control_maximum_speed",
                        "tcp_control_position_tolerance",
                        "tcp_control_maximum_distance",
                        "joint_control_maximum_torque",
                        "joint_2_control_maximum_torque",
                        "joint_3_control_maximum_torque",
                        "joint_4_control_maximum_torque",
                        "joint_5_control_maximum_torque",
                        "joint_6_control_maximum_torque",
                        "joint_mouse_control_maximum_torque",
                        "teach_replay_maximum_joint_speed",
                        "teach_replay_minimum_segment_duration",
                        "tcp_diagnostic_publish_period",
                        "surface_follow_approach_x",
                        "surface_follow_approach_y",
                        "surface_follow_approach_z",
                        "surface_follow_tangent_x",
                        "surface_follow_tangent_y",
                        "surface_follow_tangent_z",
                        "surface_follow_target_force",
                        "surface_follow_force_gain",
                        "surface_follow_approach_speed",
                        "surface_follow_maximum_normal_speed",
                        "surface_follow_duration",
                        "surface_follow_approach_timeout",
                        "surface_follow_maximum_approach_distance",
                        "surface_follow_force_tolerance",
                    )
                },
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {"use_sim_time": True},
            ],
        ),
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            output="screen",
            condition=IfCondition(use_servo),
            parameters=[
                servo_config,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": True},
            ],
        ),
        Node(
            package="massage_motion",
            executable="cartesian_jog_bridge",
            output="screen",
            condition=IfCondition(use_servo),
            parameters=[{
                "use_sim_time": True,
                "maximum_linear_speed": ParameterValue(
                    LaunchConfiguration("tcp_control_maximum_speed"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="massage_motion",
            executable="cartesian_jog_keyboard",
            output="screen",
            condition=IfCondition(use_keyboard),
            prefix="bash -c 'exec \"$@\" </dev/tty' --",
        ),
        move_group,
        prone_planning_scene,
        rviz,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, "launch", "mujoco_monitor.launch.py",
            )),
            condition=IfCondition(LaunchConfiguration("use_monitor")),
            launch_arguments={
                "plot_group": LaunchConfiguration("plot_group"),
                "plot_joint": LaunchConfiguration("plot_joint"),
            }.items(),
        ),
    ])
