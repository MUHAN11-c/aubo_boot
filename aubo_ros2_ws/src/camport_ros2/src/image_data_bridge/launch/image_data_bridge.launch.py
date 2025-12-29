#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
图像数据桥接节点启动文件
将相机图像转换为ImageData消息格式
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成launch描述"""
    
    # 声明启动参数
    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='color/image_raw',
        description='输入图像话题（相机发布的彩色图像话题）'
    )
    
    camera_status_topic_arg = DeclareLaunchArgument(
        'camera_status_topic',
        default_value='/camera_status',
        description='相机状态话题'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/image_data',
        description='输出ImageData话题'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='207000152740',
        description='相机ID（如果相机状态话题不可用时的默认值）'
    )
    
    use_jpeg_encoding_arg = DeclareLaunchArgument(
        'use_jpeg_encoding',
        default_value='false',
        description='是否使用JPEG编码压缩图像（true/false）'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='90',
        description='JPEG压缩质量（1-100，仅在use_jpeg_encoding=true时有效）'
    )
    
    # 图像数据桥接节点
    image_data_bridge_node = Node(
        package='image_data_bridge',
        executable='image_data_bridge_node',
        name='image_data_bridge_node',
        output='screen',
        parameters=[{
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'camera_status_topic': LaunchConfiguration('camera_status_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'camera_id': LaunchConfiguration('camera_id'),
            'use_jpeg_encoding': LaunchConfiguration('use_jpeg_encoding'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
        }]
    )
    
    return LaunchDescription([
        input_image_topic_arg,
        camera_status_topic_arg,
        output_topic_arg,
        camera_id_arg,
        use_jpeg_encoding_arg,
        jpeg_quality_arg,
        image_data_bridge_node,
    ])

