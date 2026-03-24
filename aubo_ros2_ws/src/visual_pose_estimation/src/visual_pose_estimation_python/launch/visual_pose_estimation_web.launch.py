#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        "host",
        default_value="127.0.0.1",
        description="FastAPI bind host",
    )

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8088",
        description="FastAPI bind port",
    )

    web_process = Node(
        package="visual_pose_estimation_python",
        executable="visual_pose_estimation_web",
        arguments=[
            "--host",
            LaunchConfiguration("host"),
            "--port",
            LaunchConfiguration("port"),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([host_arg, port_arg, web_process])
