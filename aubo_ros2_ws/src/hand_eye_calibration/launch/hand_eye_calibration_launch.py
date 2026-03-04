#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手眼标定工具启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='相机内参话题（CameraInfo，用于自动获取相机标定参数）'
    )
    
    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='/camera/depth/image_raw',
        description='深度图话题（用于深度误差检查，已对齐到彩色图）'
    )
    
    depth_scale_unit_arg = DeclareLaunchArgument(
        'depth_scale_unit',
        default_value='',
        description='深度图缩放因子（f_scale_unit），用于将深度图像素值转换为毫米。参考 depth_z_reader 的实现：对于 scale_unit=0.25 的相机，使用 0.25（转换为毫米）或 0.00025（转换为米）。如果未设置，将自动推断。可以通过查看相机节点日志获取：Depth stream scale unit: <value>'
    )
    
    # 手眼标定 TF 发布参数
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='',
        description='手眼标定结果文件路径（YAML格式）。如果为空，将使用默认路径：config/calibration_results/hand_eye_calibration_20260112_155451.yaml'
    )
    
    tf_parent_frame_arg = DeclareLaunchArgument(
        'tf_parent_frame',
        default_value='ee_link',
        description='TF 变换的父坐标系（默认：ee_link）'
    )
    
    tf_child_frame_arg = DeclareLaunchArgument(
        'tf_child_frame',
        default_value='camera_link',
        description='TF 变换的子坐标系（默认：camera_link）'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',  # 暂时关闭手眼标定 TF 发布；需要时改为 true
        description='是否发布手眼标定 TF 变换（true/false）'
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
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'robot_status_topic': LaunchConfiguration('robot_status_topic'),
            'depth_image_topic': LaunchConfiguration('depth_image_topic'),
            'depth_scale_unit': LaunchConfiguration('depth_scale_unit'),
        }],
        remappings=[
            # 将节点内部订阅的 /robot_status 映射到实际的话题 /demo_robot_status
            ('robot_status', LaunchConfiguration('actual_robot_status_topic')),
        ]
    )
    
    # 手眼标定 TF 发布节点（根据标定结果发布相机与末端执行器之间的 TF 变换）
    hand_eye_tf_publisher_node = Node(
        package='hand_eye_calibration',
        executable='hand_eye_calibration_tf_publisher',
        name='hand_eye_calibration_tf_publisher',
        output='screen',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'parent_frame': LaunchConfiguration('tf_parent_frame'),
            'child_frame': LaunchConfiguration('tf_child_frame'),
            'publish_rate': 1.0,
        }],
        condition=IfCondition(LaunchConfiguration('publish_tf'))
    )
    
    return LaunchDescription([
        web_host_arg,
        web_port_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        robot_status_topic_arg,
        actual_robot_status_topic_arg,
        input_image_topic_arg,
        depth_image_topic_arg,
        depth_scale_unit_arg,
        calibration_file_arg,
        tf_parent_frame_arg,
        tf_child_frame_arg,
        publish_tf_arg,
        image_data_converter_node,
        hand_eye_calibration_node,
        hand_eye_tf_publisher_node,
    ])





