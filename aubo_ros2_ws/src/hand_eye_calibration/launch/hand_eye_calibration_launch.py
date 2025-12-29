#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手眼标定工具启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成launch描述"""
    
    # 声明启动参数
    web_host_arg = DeclareLaunchArgument(
        'web_host',
        default_value='localhost',
        description='Web服务器监听地址'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Web服务器端口'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='相机图像话题'
    )
    
    robot_status_topic_arg = DeclareLaunchArgument(
        'robot_status_topic',
        default_value='/robot_status',
        description='机器人状态话题'
    )
    
    # 手眼标定节点
    hand_eye_calibration_node = Node(
        package='hand_eye_calibration',
        executable='hand_eye_calibration_node',
        name='hand_eye_calibration_node',
        output='screen',
        parameters=[{
            'web_host': LaunchConfiguration('web_host'),
            'web_port': LaunchConfiguration('web_port'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'robot_status_topic': LaunchConfiguration('robot_status_topic'),
        }]
    )
    
    return LaunchDescription([
        web_host_arg,
        web_port_arg,
        camera_topic_arg,
        robot_status_topic_arg,
        hand_eye_calibration_node,
    ])





