#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手眼标定工具启动文件
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
        description='机器人状态话题（节点内部使用的话题名，通过remapping映射到实际话题）'
    )
    
    actual_robot_status_topic_arg = DeclareLaunchArgument(
        'actual_robot_status_topic',
        default_value='/demo_robot_status',
        description='实际发布的机器人状态话题名称（默认：/demo_robot_status）'
    )
    
    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/camera/color/image_raw',
        description='输入图像话题（相机发布的原始图像话题）'
    )
    
    # 图像数据转换节点（将 sensor_msgs/Image 转换为 ImageData）
    # 如果不需要转换，可以注释掉这个节点，直接使用 image_data_bridge
    image_data_converter_node = Node(
        package='hand_eye_calibration',
        executable='image_data_converter_node',
        name='image_data_converter_node',
        # output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_image_topic'),
            'output_topic': '/image_data',
            'camera_id': '207000152740',
        }]
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
        }],
        remappings=[
            # 将节点内部订阅的 /robot_status 映射到实际的话题 /demo_robot_status
            ('robot_status', LaunchConfiguration('actual_robot_status_topic')),
        ]
    )
    
    return LaunchDescription([
        web_host_arg,
        web_port_arg,
        camera_topic_arg,
        robot_status_topic_arg,
        actual_robot_status_topic_arg,
        input_image_topic_arg,
        image_data_converter_node,
        hand_eye_calibration_node,
    ])





