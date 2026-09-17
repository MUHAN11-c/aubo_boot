#!/usr/bin/env python3
"""
视觉姿态估计节点启动文件.

使用方法:
    ros2 launch ivg_pose_estimation ivg_pose_estimation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch描述"""
    # 声明launch参数
    # calib_file 留空时按 path_resolver 标准链查找（含 aubo_hand_eye_calibration/hand_eye/active.yaml）
    calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value='',
        description='手眼标定文件路径（空则按标准候选链自动解析）'
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
    pose_estimation_node = Node(
        package='ivg_pose_estimation',
        executable='ivg_pose_estimation_node',
        name='ivg_pose_estimation',
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
        pose_estimation_node
    ])
