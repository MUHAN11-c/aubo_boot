# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('aubo_hand_eye_calibration')
    config = os.path.join(share, 'config', 'calibration.yaml')
    poses = os.path.join(share, 'config', 'poses.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('extrinsics_enabled', default_value='true'),
        DeclareLaunchArgument('web_enabled', default_value='true'),
        # 注意: 不要命名为 'config_file' —— 避免与其他 launch 的同名参数碰撞
        DeclareLaunchArgument('calibration_config', default_value=config),
        # 相机驱动不在此启动: 由 percipio_camera.launch.py 独立拉起
        # (bringup 则由 camera_enabled 统一管理)。
        Node(
            package='aubo_hand_eye_calibration',
            executable='extrinsics_publisher',
            name='hand_eye_extrinsics_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('extrinsics_enabled')),
        ),
        Node(
            package='aubo_hand_eye_calibration',
            executable='calibration_server',
            name='hand_eye_calibration_server',
            output='screen',
            parameters=[
                LaunchConfiguration('calibration_config'),
                {'poses_file': poses},
            ],
        ),
        Node(
            package='aubo_hand_eye_calibration',
            executable='web_gateway',
            name='hand_eye_web_gateway',
            output='screen',
            condition=IfCondition(LaunchConfiguration('web_enabled')),
        ),
    ])
