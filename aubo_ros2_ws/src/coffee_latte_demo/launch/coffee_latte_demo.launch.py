from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coffee_latte_demo',
            executable='latte_node',
            name='coffee_latte_demo',
            output='screen',
        ),
    ])
