#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet Demo + TF 专用 Launch（与 aubo_moveit_bridge_ros1 配合使用）

基于 graspnet_demo.launch.py，仅保留 GraspNet 相关部分，不含：
  - 机械臂模型与 ros2_control 仿真
  - 手眼静态 TF（由 aubo_moveit_bridge_ros1 发布 末端->camera_frame）
  - MoveIt2 move_group 与 MoveToPose 服务
  - RViz2

功能：
  启动 graspnet_demo_node（从文件或话题读取数据并预测抓取，发布 MarkerArray、点云和 grasp_pose_X 的 TF）

使用方式：先启动机械臂与 MoveIt 桥接，再启动本 launch。
  终端1: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py
  终端2: ros2 launch graspnet_ros2 graspnet_demo_with_tf.launch.py

手动触发发布抓取：
  ros2 run graspnet_ros2 publish_grasps_client

TF 链（由 bridge 与本节点共同构成）：
  [aubo_moveit_bridge_ros1] base_link -> ... -> wrist3_Link -> camera_frame
  [graspnet_demo_node] camera_frame -> grasp_pose_0, grasp_pose_1, ...
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    """声明 launch 参数（与 graspnet_demo.launch.py 一致，去掉 aubo_type、rviz_config）。"""
    try:
        hand_eye_default = os.path.join(
            get_package_share_directory('hand_eye_calibration'),
            'config', 'hand_eye_calibration_best.yaml'
        )
    except Exception:
        hand_eye_default = ''

    return [
        # GraspNet 核心
        DeclareLaunchArgument('model_path', default_value=os.path.join(
            baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar'
        ), description='模型权重路径'),
        DeclareLaunchArgument('data_dir', default_value=os.path.join(
            baseline_dir, 'doc', 'pose_1'
        ), description='数据目录（含 color/depth/mask/meta）'),
        DeclareLaunchArgument('marker_topic', default_value='grasp_markers', description='MarkerArray 话题'),
        DeclareLaunchArgument('pointcloud_topic', default_value='graspnet_pointcloud', description='点云话题'),
        DeclareLaunchArgument('frame_id', default_value='camera_frame', description='抓取坐标系'),
        DeclareLaunchArgument('use_open3d', default_value='true', description='是否启用 Open3D'),
        DeclareLaunchArgument('auto_run', default_value='false', description='是否自动运行（否则需服务触发）'),
        DeclareLaunchArgument('baseline_dir', default_value=baseline_dir, description='graspnet-baseline 路径'),

        # 手眼（节点可能使用；末端->camera_frame TF 由 aubo_moveit_bridge_ros1 发布）
        DeclareLaunchArgument('ee_frame_id', default_value='wrist3_Link', description='末端 link（与 bridge 一致）'),
        DeclareLaunchArgument('hand_eye_yaml_path', default_value=hand_eye_default, description='手眼标定 YAML'),

        # 相机与触发（可选覆盖）
        DeclareLaunchArgument('trigger_service', default_value='/software_trigger', description='软触发服务名'),
        DeclareLaunchArgument('camera_id', default_value='207000152740', description='相机 ID'),
        DeclareLaunchArgument('color_image_topic', default_value='/camera/color/image_raw', description='彩色图话题'),
        DeclareLaunchArgument('depth_image_topic', default_value='/camera/depth/image_raw', description='深度图话题'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info', description='相机内参话题'),
        DeclareLaunchArgument('factor_depth', default_value='4000.0', description='深度缩放因子'),
        DeclareLaunchArgument('trigger_grasp_service', default_value='trigger_grasp', description='触发抓取服务名'),
    ]


def launch_setup_graspnet_only(context):
    """仅启动 graspnet_demo_node（与 graspnet_demo.launch.py 中节点一致）。"""
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
    return [graspnet_demo_node]


def generate_launch_description():
    """生成 LaunchDescription。"""
    package_path = get_package_share_directory(PKG_GRASPNET)
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')

    declared_arguments = _declare_launch_arguments(package_path, baseline_dir)
    graspnet_only = OpaqueFunction(function=launch_setup_graspnet_only)

    return LaunchDescription(declared_arguments + [graspnet_only])
