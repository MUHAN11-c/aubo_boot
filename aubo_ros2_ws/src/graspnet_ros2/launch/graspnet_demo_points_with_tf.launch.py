#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 点云版 + TF 专用 Launch（与 aubo_moveit_bridge_ros1 配合使用）

功能：
  启动 graspnet_demo_points_node（订阅 PointCloud2，预测抓取，发布 MarkerArray 与 TF）。
  点云由外部节点提供，不启动相机。

使用：
  终端1: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py
  终端2: ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py

点云话题默认：/camera/depth_registered/points
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    """声明 launch 参数（仅 graspnet_demo_points_node 所需）。"""
    return [
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
    declared_arguments = _declare_launch_arguments(package_path, baseline_dir)

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

    return LaunchDescription(declared_arguments + [graspnet_demo_points_node])
