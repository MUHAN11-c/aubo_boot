#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手眼标定 TF 发布启动文件
根据标定结果文件发布相机与末端执行器之间的静态 TF 变换
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成launch描述"""
    
    # 声明启动参数
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='',
        description='手眼标定结果文件路径（YAML格式）。如果为空，将使用默认路径：config/calibration_results/hand_eye_calibration_20260112_155451.yaml'
    )
    
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='ee_link',
        description='TF 变换的父坐标系（默认：ee_link，机器人末端执行器链接）'
    )
    
    child_frame_arg = DeclareLaunchArgument(
        'child_frame',
        default_value='camera_link',
        description='TF 变换的子坐标系（默认：camera_link，相机链接）'
    )
    
    # 尝试获取默认标定文件路径
    default_calibration_file = ''
    try:
        package_share = get_package_share_directory('hand_eye_calibration')
        default_calibration_file = os.path.join(
            package_share,
            'config',
            'calibration_results',
            'hand_eye_calibration_20260112_155451.yaml'
        )
        # 检查文件是否存在
        if not os.path.exists(default_calibration_file):
            default_calibration_file = ''
    except:
        pass
    
    # 如果 install 目录不存在，尝试从源码目录查找
    if not default_calibration_file:
        try:
            # 从当前文件位置推断源码目录
            current_file = os.path.abspath(__file__)
            # 从 launch/ 目录回到 src/hand_eye_calibration/
            src_dir = os.path.dirname(os.path.dirname(current_file))
            default_calibration_file = os.path.join(
                src_dir,
                'config',
                'calibration_results',
                'hand_eye_calibration_20260112_155451.yaml'
            )
            if os.path.exists(default_calibration_file):
                default_calibration_file = os.path.abspath(default_calibration_file)
            else:
                default_calibration_file = ''
        except:
            pass
    
    # 手眼标定 TF 发布节点
    hand_eye_tf_publisher_node = Node(
        package='hand_eye_calibration',
        executable='hand_eye_calibration_tf_publisher',
        name='hand_eye_calibration_tf_publisher',
        output='screen',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'parent_frame': LaunchConfiguration('parent_frame'),
            'child_frame': LaunchConfiguration('child_frame'),
            'publish_rate': 1.0,
        }]
    )
    
    return LaunchDescription([
        calibration_file_arg,
        parent_frame_arg,
        child_frame_arg,
        hand_eye_tf_publisher_node,
    ])
