#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet Demo ROS2 Launch 文件

功能：
  1. 启动 demo.py 脚本（启用 ROS2 MarkerArray 发布）
  2. 启动 rviz2 可视化工具，显示抓取标记

使用方法：
  ros2 launch graspnet_baseline demo_rviz2.launch.py
  或
  ros2 launch graspnet_baseline demo_rviz2.launch.py data_dir:=/path/to/data
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # 声明启动参数
    declared_arguments = []
    
    # 数据目录参数
    declared_arguments.append(
        DeclareLaunchArgument(
            "data_dir",
            default_value=os.path.join(package_path, "doc", "pose_1"),
            description="数据目录路径（包含 color.png, depth.png, workspace_mask.png, meta.mat）",
        )
    )
    
    # RViz2 配置文件路径
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(package_path, "launch", "demo.rviz"),
            description="RViz2 配置文件路径",
        )
    )
    
    # 是否启动 RViz2
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否启动 RViz2",
        )
    )
    
    # 获取参数值
    data_dir = LaunchConfiguration("data_dir")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    
    # 设置环境变量，使 demo.py 启用 ROS2 发布
    set_ros2_env = SetEnvironmentVariable(
        "USE_ROS2",
        "1"
    )
    
    # 启动 demo.py 脚本
    # 注意：需要设置 USE_ROS2=1 环境变量以启用 ROS2 MarkerArray 发布
    demo_script_path = os.path.join(package_path, "demo.py")
    demo_node = ExecuteProcess(
        cmd=["python3", demo_script_path],
        output="screen",
        env=dict(os.environ, USE_ROS2="1"),
        cwd=package_path,  # 设置工作目录
    )
    
    # 启动 RViz2（如果启用）
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )
    
    # 启动静态 TF 发布器（可选，用于显示坐标系）
    # 注意：如果数据目录中有不同的坐标系，可以修改 frame_id 参数
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_frame_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "camera_frame"],
        output="screen",
    )
    
    return LaunchDescription(
        declared_arguments + [
            set_ros2_env,
            demo_node,
            rviz_node,
            static_tf_node,
        ]
    )
