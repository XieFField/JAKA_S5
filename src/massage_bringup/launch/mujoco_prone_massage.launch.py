import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_share = get_package_share_directory("massage_bringup")
    prone_initial_positions = os.path.join(
        bringup_share,
        "config",
        "prone_back_initial_positions.yaml",
    )
    default_initial_positions = os.path.join(
        get_package_share_directory("massage_mujoco"),
        "config",
        "initial_positions.yaml",
    )
    initial_positions = PythonExpression([
        "'", LaunchConfiguration("use_prone_mannequin"),
        "'.lower() == 'true' and '", prone_initial_positions,
        "' or '", default_initial_positions, "'",
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "mujoco_sim.launch.py")
        ),
        launch_arguments={
            "use_prone_mannequin": LaunchConfiguration("use_prone_mannequin"),
            "initial_positions_file": initial_positions,
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_mujoco_viewer": LaunchConfiguration("use_mujoco_viewer"),
            "use_keyboard": LaunchConfiguration("use_keyboard"),
            "use_monitor": LaunchConfiguration("use_monitor"),
            "plot_group": LaunchConfiguration("plot_group"),
            "show_contact_points": "true",
            "show_contact_forces": "true",
            "surface_follow_approach_x": "0.0",
            "surface_follow_approach_y": "0.0",
            "surface_follow_approach_z": "-1.0",
            "surface_follow_tangent_x": "0.02",
            "surface_follow_tangent_y": "0.0",
            "surface_follow_tangent_z": "0.0",
            "surface_follow_maximum_approach_distance": "0.05",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_prone_mannequin", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_mujoco_viewer", default_value="true"),
        DeclareLaunchArgument("use_keyboard", default_value="false"),
        DeclareLaunchArgument("use_monitor", default_value="false"),
        DeclareLaunchArgument("plot_group", default_value="contact"),
        simulation,
    ])
