from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    s5_config_share = get_package_share_directory("jaka_s5_moveit_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ip", default_value="10.5.5.100", description="JAKA robot IP address"
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", description="Start the optional MoveIt/RViz stack"
        ),
        DeclareLaunchArgument(
            "diagnostics_enabled",
            default_value="false",
            description="Publish JAKA controller diagnostic state",
        ),
        DeclareLaunchArgument(
            "auto_power_off_on_exit",
            default_value="true",
            description="Disable and power off the robot when the controller exits",
        ),
        DeclareLaunchArgument(
            "auto_move_to_initial_on_start",
            default_value="false",
            description="Move to the S5 initial pose once after startup",
        ),
        DeclareLaunchArgument(
            "joint_move_max_speed_rad_s",
            default_value="1.57",
            description="Maximum joint_move speed used at 100 percent",
        ),
        DeclareLaunchArgument(
            "joint_move_max_acceleration_rad_s2",
            default_value="1.57",
            description="Maximum joint_move acceleration used at 100 percent",
        ),
        Node(
            package="control_s5_controller",
            executable="s5_motion_server",
            name="control_s5_motion_server",
            output="screen",
            parameters=[{
                "ip": LaunchConfiguration("ip"),
                "model": "s5",
                "diagnostics_enabled": LaunchConfiguration("diagnostics_enabled"),
                "auto_power_off_on_exit": LaunchConfiguration("auto_power_off_on_exit"),
                "joint_move_max_speed_rad_s": LaunchConfiguration(
                    "joint_move_max_speed_rad_s"
                ),
                "joint_move_max_acceleration_rad_s2": LaunchConfiguration(
                    "joint_move_max_acceleration_rad_s2"
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(s5_config_share, "launch", "rsp.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    s5_config_share, "launch", "static_virtual_joint_tfs.launch.py"
                )
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(s5_config_share, "launch", "move_group.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(s5_config_share, "launch", "moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        Node(
            package="control_s5_controller",
            executable="s5_joint_controller",
            name="s5_joint_controller",
            output="screen",
            parameters=[{
                "auto_power_off_on_exit": LaunchConfiguration("auto_power_off_on_exit"),
                "auto_move_to_initial_on_start": LaunchConfiguration(
                    "auto_move_to_initial_on_start"
                ),
            }],
        ),
    ])
