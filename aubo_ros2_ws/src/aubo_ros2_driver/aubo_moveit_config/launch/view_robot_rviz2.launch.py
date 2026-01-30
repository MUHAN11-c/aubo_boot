#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单独启动 aubo_ros2.xacro 模型的 RViz2 可视化
功能：仅启动 robot_state_publisher 和 rviz2，用于查看机器人模型
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    # 获取启动参数值
    support_package = LaunchConfiguration("support_package")
    aubo_type = LaunchConfiguration("aubo_type")
    rviz_config = LaunchConfiguration("rviz_config")
    use_gui = LaunchConfiguration("use_gui")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    
    # 获取 use_ros2_control 参数的值
    use_ros2_control_val = use_ros2_control.perform(context)
    
    # 构建机器人描述（使用 aubo_ros2.xacro，已包含ros2_control定义）
    robot_xacro_file = "aubo_ros2.xacro"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(support_package), "urdf/xacro/inc/", robot_xacro_file]
            ),
            " ",
            "aubo_type:=",
            aubo_type,
            " ",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    nodes_to_start = []
    
    # Robot State Publisher 节点（始终启动）
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    nodes_to_start.append(robot_state_pub_node)
    
    # 根据 use_ros2_control 参数决定使用哪种控制方式
    if use_ros2_control_val == "true":
        # ROS2 Control 控制器管理器节点
        ros2_controllers_path = os.path.join(
            get_package_share_directory("aubo_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        )
        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, ros2_controllers_path],
            output="screen",
        )
        
        # 启动 joint_state_broadcaster 控制器
        # 使用 TimerAction 延迟启动控制器，增加超时时间以确保controller_manager已启动
        joint_state_broadcaster_spawner = TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "run", "controller_manager", "spawner.py", "joint_state_broadcaster", "--controller-manager-timeout", "20"],
                    shell=True,
                    output="screen",
                )
            ]
        )
        
        nodes_to_start.extend([
            controller_manager_node,
            joint_state_broadcaster_spawner,
        ])
    else:
        # 使用传统的 joint_state_publisher
        use_gui_val = use_gui.perform(context)
        if use_gui_val == "true":
            joint_state_pub_gui_node = Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[robot_description],
            )
            nodes_to_start.append(joint_state_pub_gui_node)
        else:
            joint_state_pub_node = Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[robot_description],
            )
            nodes_to_start.append(joint_state_pub_node)
    
    # RViz2 节点（始终启动）
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[robot_description],
    )
    nodes_to_start.append(rviz_node)
    
    return nodes_to_start


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            default_value="aubo_description",
            description="机器人描述包名称",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_e5_10",
            description="AUBO机器人型号",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("aubo_moveit_config"),
                "config",
                "view_robot_simple.rviz"  # 使用简单配置，不包含MoveIt插件
            ),
            description="RViz配置文件路径",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",  # 默认启用GUI，方便手动控制关节
            description="是否启动joint_state_publisher_gui（用于手动调整关节，仅在use_ros2_control=false时有效）",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="false",  # 默认不使用ros2_control，使用简单的joint_state_publisher
            description="是否使用ros2_control（需要ros2_controllers包）",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
