# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('aubo_hand_eye_calibration')
    camera_share = get_package_share_directory('percipio_camera')
    config = os.path.join(share, 'config', 'calibration.yaml')
    poses = os.path.join(share, 'config', 'poses.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('extrinsics_enabled', default_value='true'),
        DeclareLaunchArgument('web_enabled', default_value='true'),
        # 注意: 不要命名为 'config_file' —— 会与内嵌的 percipio_camera
        # launch 的同名参数碰撞, 导致相机驱动误读本文件作为参数文件
        DeclareLaunchArgument('calibration_config', default_value=config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                camera_share, 'launch', 'percipio_camera.launch.py')),
            condition=IfCondition(LaunchConfiguration('camera_enabled')),
            # 官方驱动的 launch 参数: 手眼标定只用 RGB, 关闭深度/点云
            launch_arguments={
                'camera_name': 'camera',
                'device_ip': '169.254.10.110',
                'color_enable': 'true',
                'depth_enable': 'false',
                'depth_registration_enable': 'false',
                'point_cloud_enable': 'false',
                'color_point_cloud_enable': 'false',
            }.items(),
        ),
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
