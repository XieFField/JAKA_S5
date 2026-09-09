import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    use_gazebo = LaunchConfiguration("use_gazebo")
    robot_xacro = os.path.join(
        get_package_share_directory("massage_description"),
        "urdf",
        "jaka_s5_massage.urdf.xacro",
    )
    semantic_file = os.path.join(
        get_package_share_directory("massage_description"),
        "config",
        "jaka_s5_massage.srdf",
    )
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_s5",
            package_name="jaka_s5_moveit_config",
        )
        .robot_description(
            file_path=robot_xacro,
            mappings={
                "use_gazebo": use_gazebo,
                "use_rviz_sim": "false",
                "use_massage_head": "true",
                "initial_positions_file": initial_positions_file,
            },
        )
        .robot_description_semantic(file_path=semantic_file)
        .joint_limits(file_path="config/joint_limits_sim.yaml")
        .to_moveit_configs()
    )
    publish_scene = LaunchConfiguration("publish_monitored_planning_scene")
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": ParameterValue(
                    LaunchConfiguration("allow_trajectory_execution"),
                    value_type=bool,
                ),
                "publish_planning_scene": publish_scene,
                "publish_geometry_updates": publish_scene,
                "publish_state_updates": publish_scene,
                "publish_transforms_updates": publish_scene,
                "monitor_dynamics": False,
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
        DeclareLaunchArgument(
            "publish_monitored_planning_scene", default_value="true",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=os.path.join(
                get_package_share_directory("massage_mujoco"),
                "config",
                "initial_positions.yaml",
            ),
            description="Initial joint positions embedded in robot_description",
        ),
        move_group,
    ])
