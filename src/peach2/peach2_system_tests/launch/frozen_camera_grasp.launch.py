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
        # 不再强设 QT_QPA_PLATFORM=offscreen：它会破坏 rviz2 的 Ogre GLX 窗口句柄
        # （每轮 100 次 Invalid parentWindowHandle 后退出）。桌面测试保留 rviz 可视化；
        # 无头环境显式传 rviz:=false 关掉它。
        DeclareLaunchArgument(
            'bag', default_value=bag_default,
            description='mcap bag directory with /camera/* (frozen frame)'),
        DeclareLaunchArgument('frame_index', default_value='70'),
        DeclareLaunchArgument('hz', default_value='2.0'),
        DeclareLaunchArgument(
            'replay_start_paused', default_value='false',
            description='true = replay publishes nothing until its ~/pause '
                        'service resumes it (blind photo pose)'),
        DeclareLaunchArgument(
            'photo_pose', default_value='global_photo_pose',
            description='SRDF survey pose (product photo pose); the batch '
                        'surveys in place — the mock spawns there via '
                        'mock_initial_positions'),
        DeclareLaunchArgument(
            'tool_id', default_value='adaptive_shear_v1',
            description='v2 mock FULL uses the mock IO backend (no real SetIO)'),
        DeclareLaunchArgument(
            'mock_initial_positions',
            default_value='0.425175,0.194274,1.679546,1.462579,-0.279948,0.038859',
            description='mock spawn joints (shoulder..wrist3); default = SRDF '
                        'global_photo_pose (true photo pose, 2026-09-17 '
                        'teaching); --start-pose stow passes the home literals'),
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
                'photo_pose': LaunchConfiguration('photo_pose'),
                # spawn pose: default = true photo pose (launch arg);
                # photo-move flow passes home joints (its rigid-transport
                # arithmetic is calibrated from the home camera pose).
                'mock_initial_positions': LaunchConfiguration(
                    'mock_initial_positions'),
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
                '--start-paused', LaunchConfiguration('replay_start_paused'),
            ]),
    ])
