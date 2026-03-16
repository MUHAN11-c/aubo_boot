#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet Demo + TF 专用 Launch（与 aubo_moveit_bridge_ros1 配合使用）

功能：
  启动 graspnet_demo_node（从文件或话题读取数据并预测抓取，发布 MarkerArray、点云和 grasp_pose_X 的 TF）

使用：
  终端1: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py
  终端2: ros2 launch graspnet_ros2 graspnet_demo_with_tf.launch.py
  手动触发: ros2 run graspnet_ros2 publish_grasps_client

TF 链：bridge 发布 base_link -> ... -> wrist3_Link -> camera_frame；本节点发布 camera_frame -> grasp_pose_0, ...
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
    """声明 launch 参数（仅 graspnet_demo_node 所需）。"""
    try:
        hand_eye_default = os.path.join(
            get_package_share_directory('hand_eye_calibration'),
            'config', 'hand_eye_calibration_best.yaml'
        )
    except Exception:
        hand_eye_default = ''

    return [
        DeclareLaunchArgument('baseline_dir', default_value=baseline_dir, description='graspnet-baseline 路径'),
        DeclareLaunchArgument('model_path', default_value=os.path.join(
            baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar'
        ), description='模型权重路径'),
        DeclareLaunchArgument('data_dir', default_value=os.path.join(
            baseline_dir, 'doc', 'pose_1'
        ), description='数据目录（含 color/depth/mask/meta）'),
        DeclareLaunchArgument('marker_topic', default_value='grasp_markers', description='MarkerArray 话题'),
        DeclareLaunchArgument('pointcloud_topic', default_value='graspnet_pointcloud', description='点云话题'),
        DeclareLaunchArgument('frame_id', default_value='camera_frame', description='抓取坐标系'),
        DeclareLaunchArgument('hand_eye_yaml_path', default_value=hand_eye_default, description='手眼标定 YAML'),
        DeclareLaunchArgument('use_open3d', default_value='true', description='是否启用 Open3D'),
        DeclareLaunchArgument('auto_run', default_value='false', description='是否自动运行（否则需服务触发）'),
    ]


def generate_launch_description():
    """生成 LaunchDescription。"""
    package_path = get_package_share_directory(PKG_GRASPNET)
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')
    declared_arguments = _declare_launch_arguments(package_path, baseline_dir)

    graspnet_demo_node = Node(
        package=PKG_GRASPNET,
        executable='graspnet_demo_node',
        name='graspnet_demo_node',
        output='screen',
        parameters=[{
            'baseline_dir': LaunchConfiguration('baseline_dir'),
            'model_path': LaunchConfiguration('model_path'),
            'data_dir': LaunchConfiguration('data_dir'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'hand_eye_yaml_path': LaunchConfiguration('hand_eye_yaml_path'),
            'use_open3d': LaunchConfiguration('use_open3d'),
            'auto_run': LaunchConfiguration('auto_run'),
        }],
    )

    return LaunchDescription(declared_arguments + [graspnet_demo_node])
