"""
v2 mock stack + frozen RGB-D replay: one photo-pose view then mock FULL.

Uses real peach2_perception (not the harness stubs). Does not send RunBatch
or SetEnables. hardware_mode is mock; the arm never talks to a real controller.
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_BAG = str(
    Path.cwd() / 'runs' / 'peach_camera_capture_20260930' / 'stereo_10s')


def generate_launch_description() -> LaunchDescription:
    system = os.path.join(
        get_package_share_directory('peach2_bringup'),
        'launch', 'peach2_system.launch.py')
    share = get_package_share_directory('peach2_system_tests')
    overlay = os.path.join(share, 'config', 'frozen_target_model.yaml')
    tools = os.path.join(share, 'config', 'frozen_tools')
    scene = os.path.join(share, 'config', 'frozen_scene.yaml')
    perception = os.path.join(share, 'config', 'frozen_perception.yaml')
    bag_default = os.environ.get('PEACH_FROZEN_BAG', '').strip() or _DEFAULT_BAG
    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'offscreen'),
        DeclareLaunchArgument(
            'bag', default_value=bag_default,
            description='mcap bag directory with /camera/* (frozen frame)'),
        DeclareLaunchArgument('frame_index', default_value='70'),
        DeclareLaunchArgument('hz', default_value='2.0'),
        DeclareLaunchArgument(
            'tool_id', default_value='adaptive_shear_v1',
            description='v2 mock FULL uses the mock IO backend (no real SetIO)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system),
            launch_arguments={
                'hardware_mode': 'mock',
                'camera_enabled': 'true',
                'camera_frontend': 'replay',
                'moveit_enabled': 'true',
                'bond_timeout': '0.0',
                'tool_id': LaunchConfiguration('tool_id'),
                'target_model_config': overlay,
                'tool_description_dir': tools,
                'scene_config': scene,
                'perception_config': perception,
                'lock_wait_s': '20.0',
                'observe_max_views': '1',
            }.items()),
        Node(
            package='peach2_system_tests',
            executable='frozen_camera_replay.py',
            name='frozen_camera_replay',
            output='screen',
            arguments=[
                '--bag', LaunchConfiguration('bag'),
                '--index', LaunchConfiguration('frame_index'),
                '--hz', LaunchConfiguration('hz'),
            ]),
    ])
