"""
仅启动 moveit_cpp_tutorial_node，并注入 robot_description / robot_description_semantic。
用于单独跑教程节点（不依赖 graspnet_demo 或 move_group）。

用法:
  ros2 launch aubo_moveit_config moveit_cpp_tutorial.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder("aubo_robot", package_name="aubo_moveit_config")
        .robot_description_semantic(file_path="config/aubo_i5.srdf")
        .to_moveit_configs()
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("aubo_description"),
                "urdf/xacro/inc/",
                "aubo_ros2.xacro",
            ]),
            " aubo_type:=aubo_e5_10 ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = moveit_config.robot_description_semantic

    node = Node(
        package="demo_driver",
        executable="moveit_cpp_tutorial_node",
        name="run_moveit_cpp",
        output="screen",
        parameters=[robot_description, robot_description_semantic],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
