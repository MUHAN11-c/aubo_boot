#!/usr/bin/env python3
"""
视觉姿态估计Python版本启动文件

使用方法:
    ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成launch描述"""
    # 获取包路径
    pkg_share = get_package_share_directory('visual_pose_estimation_python')

    # 声明launch参数
    calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value=os.path.join(pkg_share, 'web_ui', 'configs', 'hand_eye_calibration.yaml'),
        description='手眼标定文件路径'
    )

    template_root_arg = DeclareLaunchArgument(
        'template_root',
        default_value='',
        description='模板根目录路径（空则按包资源规则自动解析）'
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

    soft_trigger_topic_arg = DeclareLaunchArgument(
        'soft_trigger_topic',
        default_value='/camera/soft_trigger',
        description='Percipio 软触发话题（std_msgs/String）'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='基座坐标系'
    )

    ee_frame_arg = DeclareLaunchArgument(
        'ee_frame',
        default_value='tcp',
        description='末端坐标系（本区 SRDF tip）'
    )

    camera_optical_frame_arg = DeclareLaunchArgument(
        'camera_optical_frame',
        default_value='camera_color_optical_frame',
        description='彩色光学系（T_B_C TF 查询）'
    )

    # 创建节点
    visual_pose_estimation_node = Node(
        package='visual_pose_estimation_python',
        executable='visual_pose_estimation_node',
        name='visual_pose_estimation_python',
        output='screen',
        parameters=[{
            'calib_file': LaunchConfiguration('calib_file'),
            'template_root': LaunchConfiguration('template_root'),
            'depth_image_topic': LaunchConfiguration('depth_image_topic'),
            'color_image_topic': LaunchConfiguration('color_image_topic'),
            'soft_trigger_topic': LaunchConfiguration('soft_trigger_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        calib_file_arg,
        template_root_arg,
        depth_image_topic_arg,
        color_image_topic_arg,
        soft_trigger_topic_arg,
        base_frame_arg,
        ee_frame_arg,
        camera_optical_frame_arg,
        visual_pose_estimation_node
    ])
