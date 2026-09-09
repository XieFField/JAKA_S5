"""Open standard rqt plots without taking simulation control ownership."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _plots(context):
    group = LaunchConfiguration('plot_group').perform(context)
    joint = int(LaunchConfiguration('plot_joint').perform(context))
    if joint not in range(1, 7):
        raise ValueError('plot_joint must be 1 through 6')
    path = (Path(get_package_share_directory('massage_bringup'))
            / 'config/mujoco_plots.yaml')
    config = yaml.safe_load(path.read_text())
    selections = [(group, joint)]
    if group == 'all':
        selections = [('joint', j) for j in range(1, 7)] + [
            (name, joint) for name in config if name != 'joint']
    elif group not in config:
        raise ValueError(
            'plot_group must be joint, effort, force, torque, contact, '
            'tcp_position, tcp_velocity or all'
        )
    return [Node(
        package='rqt_plot', executable='rqt_plot',
        name=f'mujoco_plot_{name}_{j}', output='screen',
        arguments=['--force-discover', '--empty'] + [
            topic.format(joint=j - 1) for topic in config[name]],
    ) for name, j in selections]


def generate_launch_description():
    """Select one plot by default; all opens joint and sensor plots."""
    return LaunchDescription([
        DeclareLaunchArgument('plot_group', default_value='joint'),
        DeclareLaunchArgument('plot_joint', default_value='1'),
        OpaqueFunction(function=_plots),
    ])
