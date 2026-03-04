#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coordinate_transforms_py',
            executable='coord_tf_demo_node',
            name='coord_tf_demo_node',
            output='screen',
        ),
    ])
