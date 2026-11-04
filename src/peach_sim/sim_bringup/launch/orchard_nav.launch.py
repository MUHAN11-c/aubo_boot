"""Compatibility entry sharing the unified Nav2 and manipulation composition."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context):
    """Translate legacy GUI arguments and include the coherent orchard preset."""
    share = Path(get_package_share_directory('sim_bringup'))
    gui = LaunchConfiguration('gui').perform(context)
    if LaunchConfiguration('gz_gui').perform(context).lower() == 'true':
        gui = 'true'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'harvest.launch.py')),
        launch_arguments={
            'scene': 'orchard', 'gui': gui, 'rviz': LaunchConfiguration('rviz'),
            'slam': LaunchConfiguration('slam'), 'map': LaunchConfiguration('map'),
            'nav2_params': LaunchConfiguration('nav2_params'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'false',
        }.items())]


def generate_launch_description():
    """Retain historical navigation arguments without a second bringup implementation."""
    args = {'use_sim_time': 'true', 'slam': 'false', 'map': '', 'nav2_params': '',
            'gui': 'false', 'gz_gui': 'false', 'rviz': 'true'}
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=default) for key, default in args.items()],
        OpaqueFunction(function=_setup),
    ])
