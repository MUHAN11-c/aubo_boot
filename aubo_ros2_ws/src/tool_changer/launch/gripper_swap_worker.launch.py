from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tool_changer',
            executable='gripper_swap_worker_node',
            name='gripper_swap_worker',
            output='screen',
        ),
    ])
