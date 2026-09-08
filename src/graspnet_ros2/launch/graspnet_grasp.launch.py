#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 抓取执行 launch：检测节点 + 抓取执行客户端.

前置条件（外部启动，本 launch 不拉起，也不自动运动）：
  - move_group（aubo_e5_moveit_config）已启动，/move_action、
    /compute_cartesian_path、/execute_trajectory 可用；
  - 授权真机运动后，由操作者手动触发（publish_grasps_client 收到足够
    PoseArray 组后自动执行一次抓取接近）。

用法：
  终端1: ros2 launch aubo_e5_moveit_config <moveit launch>
  终端2: ros2 launch graspnet_ros2 graspnet_grasp.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_GRASPNET = 'graspnet_ros2'


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG_GRASPNET)

    detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'graspnet_detect.launch.py')
        ),
        launch_arguments={
            'workspace': LaunchConfiguration('workspace'),
            'grasp_poses_topic': LaunchConfiguration('grasp_poses_topic'),
        }.items(),
    )

    grasp_client = Node(
        package=PKG_GRASPNET,
        executable='publish_grasps_client',
        name='publish_grasps_client',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'graspnet.yaml'), {
            'grasp_poses_topic': LaunchConfiguration('grasp_poses_topic'),
            'planning_group': LaunchConfiguration('planning_group'),
            'ee_link': LaunchConfiguration('ee_link'),
            'base_frame': LaunchConfiguration('base_frame'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace', default_value='',
            description="透传给检测节点的工作区盒 'xmin,xmax,ymin,ymax,zmin,zmax'",
        ),
        DeclareLaunchArgument(
            'grasp_poses_topic', default_value='grasp_poses_base',
            description='检测→执行客户端之间的 PoseArray 话题',
        ),
        DeclareLaunchArgument(
            'planning_group', default_value='manipulator_e5',
            description='MoveIt 规划组（本区 aubo_e5.srdf）',
        ),
        DeclareLaunchArgument(
            'ee_link', default_value='tcp',
            description='MoveIt 末端 link（本区 aubo_e5.srdf tip）',
        ),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='规划参考系',
        ),
        detect_launch,
        grasp_client,
    ])
