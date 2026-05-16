"""生产默认：同时启动 gripper_swap_worker + scene_attach_worker（ACO + world 清理）喵~"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tool_changer",
            executable="gripper_swap_worker_node",
            name="gripper_swap_worker",
            output="screen",
        ),
        Node(
            package="tool_changer",
            executable="scene_attach_worker_node",
            name="scene_attach_worker",
            output="screen",
        ),
    ])
