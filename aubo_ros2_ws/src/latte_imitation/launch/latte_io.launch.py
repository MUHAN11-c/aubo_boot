from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='latte_imitation',
            executable='latte_io_node',
            name='latte_io',
            output='screen',
        ),
    ])
