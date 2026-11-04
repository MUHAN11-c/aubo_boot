# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Hardware-free convenience entry point; never sends a command itself."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Use the demo views and examples while forwarding every normal option."""
    share = get_package_share_directory('ros2_web_link')
    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='true'),
        DeclareLaunchArgument('profile', default_value='demo', choices=['demo', 'robot']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            share + '/launch/web.launch.py'), launch_arguments={
                'demo': LaunchConfiguration('demo'),
                'profile': LaunchConfiguration('profile'),
            }.items()),
    ])
