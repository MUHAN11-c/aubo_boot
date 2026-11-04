"""Compatibility entry for the unified simulator; use sim.launch.py for new deployments."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Keep historical world/gui arguments while sharing the complete robot model."""
    bringup = Path(get_package_share_directory('sim_bringup'))
    worlds = Path(get_package_share_directory('sim_worlds'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(worlds / 'worlds' / 'peach_orchard.sdf')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('mobile', default_value='true'),
        DeclareLaunchArgument('moveit', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('tool_profile', default_value='bite_shear_v1'),
        LogInfo(msg=(
            'orchard_sim is a compatibility alias. The robot now always contains the mobile '
            'platform and complete arm; mobile no longer selects a separate robot model.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch' / 'sim.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'),
                'moveit': LaunchConfiguration('moveit'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'tool_profile': LaunchConfiguration('tool_profile'), 'spawn': 'corridor',
            }.items()),
    ])
