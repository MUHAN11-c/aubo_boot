#!/usr/bin/env python3

"""
Launch file to start ROS 1 side components.
This should be run in a separate terminal with ROS 1 environment sourced.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='127.0.0.1',
        description='Robot IP address'
    )
    
    # Note: This launch file is for reference only.
    # In practice, you should run ROS 1 components manually:
    # 1. roscore
    # 2. roslaunch aubo_e5_moveit_config moveit_planning_execution.launch robot_ip:=<ip>
    # 3. rosrun demo_driver execute_trajectory_server_node
    # 4. rosrun demo_driver robot_status_publisher_node
    
    return LaunchDescription([
        robot_ip_arg,
        # ROS 1 components should be started manually
    ])

