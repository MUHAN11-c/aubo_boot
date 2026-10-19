"""v1 mock stack + frozen RGB-D replay: one photo-pose view then dry FULL grasp.

Does not send RunHarvest or SetEnables. Operator / drive_frozen_grasp.py starts
the batch. hardware_mode is mock; the arm never talks to a real controller.
skip_reconstruction stays false: overlay sets min_views=1 (unrefined diameter
is 0 with real perception).
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_BAG = str(
    Path.cwd() / 'runs' / 'peach_camera_capture_20260930' / 'stereo_10s')


def generate_launch_description() -> LaunchDescription:
    harvest = os.path.join(
        get_package_share_directory('peach_bringup'),
        'launch', 'harvest_system.launch.py')
    overlay = os.path.join(
        get_package_share_directory('peach_system_tests'),
        'config', 'frozen_one_view.yaml')
    bag_default = os.environ.get('PEACH_FROZEN_BAG', '').strip() or _DEFAULT_BAG
    return LaunchDescription([
        # 不再强设 QT_QPA_PLATFORM=offscreen：它会破坏 rviz2 的 Ogre GLX 窗口句柄。
        # 冻结回归默认无头；桌面验收显式传 rviz:=true。
        DeclareLaunchArgument(
            'bag', default_value=bag_default,
            description='mcap bag directory with /camera/* (frozen frame)'),
        DeclareLaunchArgument('frame_index', default_value='70'),
        DeclareLaunchArgument('hz', default_value='2.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'mock_initial_positions',
            default_value='0.425175,0.194274,1.679546,1.462579,-0.279948,0.038859',
            description='mock spawn joints; default is SRDF global_photo_pose '
                        'so the frozen frame is not republished during a '
                        'home-to-photo transit'),
        DeclareLaunchArgument(
            'tool_profile', default_value='bite_shear_v1',
            description='v1 main tool for this dry-grasp path'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(harvest),
            launch_arguments={
                'hardware_mode': 'mock',
                'camera_enabled': 'true',
                'camera_frontend': 'replay',
                'skip_reconstruction': 'false',
                'brain_params_overlay': overlay,
                'force_enabled': 'false',
                'moveit_enabled': 'true',
                'rviz': LaunchConfiguration('rviz'),
                'extrinsics_enabled': 'true',
                'hand_eye_enabled': 'false',
                'hand_eye_web_enabled': 'false',
                'autostart': 'false',
                'bond_timeout': '0.0',
                'tool_profile': LaunchConfiguration('tool_profile'),
                'mock_initial_positions': LaunchConfiguration(
                    'mock_initial_positions'),
            }.items()),
        Node(
            package='peach_system_tests',
            executable='frozen_camera_replay.py',
            name='frozen_camera_replay',
            output='screen',
            arguments=[
                '--bag', LaunchConfiguration('bag'),
                '--index', LaunchConfiguration('frame_index'),
                '--hz', LaunchConfiguration('hz'),
            ]),
    ])
