#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 点云版 + TF 专用 Launch（与 aubo_moveit_bridge_ros1 配合使用）

功能：
  1. 可选启动 Percipio 相机驱动（launch_camera:=true 时）
  2. 可选发布手眼标定静态 TF（末端 -> camera_frame）及 camera_frame -> camera_link
  3. 启动 graspnet_demo_points_node（订阅 PointCloud2，预测抓取，发布 MarkerArray 与 TF）

使用：
  终端1: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py
  终端2: ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py

不启动相机（点云由外部提供）：launch_camera:=false

点云话题默认：/camera/depth_registered/points
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_GRASPNET = 'graspnet_ros2'


def get_package_share_directory(package_name):
    """获取包的 share 目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory as _get
        return _get(package_name)
    except ImportError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _declare_launch_arguments(package_path, baseline_dir):
    """声明 launch 参数。"""
    try:
        hand_eye_default = os.path.join(
            get_package_share_directory('hand_eye_calibration'),
            'config', 'hand_eye_calibration_best.yaml'
        )
    except Exception:
        hand_eye_default = ''

    return [
        # 相机
        DeclareLaunchArgument('launch_camera', default_value='true',
                              description='是否启动 Percipio 相机驱动'),
        DeclareLaunchArgument('ee_frame_id', default_value='wrist3_Link', description='末端 link（手眼 TF 父坐标系）'),
        DeclareLaunchArgument('hand_eye_yaml_path', default_value=hand_eye_default, description='手眼标定 YAML'),
        # GraspNet
        DeclareLaunchArgument('baseline_dir', default_value=baseline_dir,
                              description='graspnet-baseline 路径'),
        DeclareLaunchArgument('model_path', default_value=os.path.join(
            baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar'
        ), description='模型权重路径'),
        DeclareLaunchArgument('input_pointcloud_topic', default_value='/camera/depth_registered/points',
                              description='输入点云话题'),
        DeclareLaunchArgument('marker_topic', default_value='grasp_markers', description='MarkerArray 话题'),
        DeclareLaunchArgument('frame_id', default_value='camera_frame', description='抓取坐标系'),
        DeclareLaunchArgument('use_open3d', default_value='false', description='是否启用 Open3D 可视化'),
    ]


def generate_launch_description():
    """生成 LaunchDescription。"""
    package_path = get_package_share_directory(PKG_GRASPNET)
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    declared_arguments = _declare_launch_arguments(package_path, baseline_dir)

    # 相机驱动（与 graspnet 点云话题 /camera/depth_registered/points 对应）
    percipio_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'percipio_camera_calibration.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    # 手眼静态 TF（末端 -> camera_frame）及 camera_frame -> camera_link
    hand_eye_static_tf_node = Node(
        package=PKG_GRASPNET,
        executable='hand_eye_static_tf_node',
        name='hand_eye_static_tf_node',
        output='screen',
        parameters=[{
            'hand_eye_yaml_path': LaunchConfiguration('hand_eye_yaml_path'),
            'ee_frame_id': LaunchConfiguration('ee_frame_id'),
            'child_frame_id': 'camera_frame',
        }],
    )
    camera_frame_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_to_camera_link',
        arguments=['0', '0', '0', '1.5708', '-1.5708', '0', 'camera_frame', 'camera_link'],
    )
    hand_eye_and_camera_tf = GroupAction(
        [hand_eye_static_tf_node, camera_frame_to_camera_link],
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    graspnet_demo_points_node = Node(
        package=PKG_GRASPNET,
        executable='graspnet_demo_points_node',
        name='graspnet_demo_points_node',
        output='screen',
        parameters=[{
            'baseline_dir': LaunchConfiguration('baseline_dir'),
            'model_path': LaunchConfiguration('model_path'),
            'input_pointcloud_topic': LaunchConfiguration('input_pointcloud_topic'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'use_open3d': LaunchConfiguration('use_open3d'),
        }],
    )

    return LaunchDescription(
        declared_arguments + [
            percipio_camera_launch,
            hand_eye_and_camera_tf,
            graspnet_demo_points_node,
        ]
    )
