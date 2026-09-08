#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 抓取检测 launch.

只起 graspnet_demo_points_node；相机与手眼 TF 由本区 bringup /
extrinsics_publisher 提供。参数默认值以 config/graspnet.yaml 为单一事实源，
launch 仅暴露 model_path 与 workspace 两个常用旋钮（其余可用 yaml 覆盖）。

用法：
  ros2 launch graspnet_ros2 graspnet_detect.launch.py
  例：workspace:='-0.3,0.3,-0.3,0.3,0.2,1.0'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_GRASPNET = 'graspnet_ros2'


def _default_model_path() -> str:
    share = get_package_share_directory(PKG_GRASPNET)
    return os.path.join(share, 'models', 'checkpoint-rs.tar')


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG_GRASPNET)

    declared_arguments = [
        DeclareLaunchArgument(
            'model_path', default_value=_default_model_path(),
            description='GraspNet 权重路径',
        ),
        DeclareLaunchArgument(
            'workspace', default_value='',
            description="工作区盒 'xmin,xmax,ymin,ymax,zmin,zmax'（点云系，米），空=不过滤",
        ),
    ]

    graspnet_node = Node(
        package=PKG_GRASPNET,
        executable='graspnet_demo_points_node',
        name='graspnet_demo_points_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'graspnet.yaml'), {
            'model_path': LaunchConfiguration('model_path'),
            'workspace': LaunchConfiguration('workspace'),
        }],
    )

    return LaunchDescription(declared_arguments + [graspnet_node])
