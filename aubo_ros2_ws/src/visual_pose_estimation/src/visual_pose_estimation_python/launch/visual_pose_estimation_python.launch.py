#!/usr/bin/env python3
"""
视觉姿态估计Python版本启动文件

使用方法:
    ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成launch描述"""
    
    # 获取包路径
    pkg_share = get_package_share_directory('visual_pose_estimation_python')
    
    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'configs', 'default.yaml'),
        description='配置文件路径'
    )
    
    calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value='',
        description='手眼标定文件路径'
    )
    
    template_root_arg = DeclareLaunchArgument(
        'template_root',
        default_value='/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/templates',
        description='模板根目录路径'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='是否启用调试模式'
    )
    
    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='/camera/depth/image_raw',
        description='深度图话题名称'
    )
    
    color_image_topic_arg = DeclareLaunchArgument(
        'color_image_topic',
        default_value='/camera/color/image_raw',
        description='彩色图话题名称'
    )
    
    # 创建节点
    visual_pose_estimation_node = Node(
        package='visual_pose_estimation_python',
        executable='visual_pose_estimation_node',
        name='visual_pose_estimation_python',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'calib_file': LaunchConfiguration('calib_file'),
            'template_root': LaunchConfiguration('template_root'),
            'debug': LaunchConfiguration('debug'),
            'depth_image_topic': LaunchConfiguration('depth_image_topic'),
            'color_image_topic': LaunchConfiguration('color_image_topic'),
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        calib_file_arg,
        template_root_arg,
        debug_arg,
        depth_image_topic_arg,
        color_image_topic_arg,
        visual_pose_estimation_node
    ])
