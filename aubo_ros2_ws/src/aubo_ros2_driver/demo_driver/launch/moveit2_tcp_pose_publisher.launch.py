from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_driver',
            executable='moveit2_tcp_pose_publisher.py',
            name='moveit2_tcp_pose_publisher',
            output='screen',
            parameters=[{
                'planning_group': 'manipulator',
                'base_frame': 'base_link',
                'end_effector_link': 'tool_center_point',  # 使用SRDF中定义的TCP
                'publish_rate_hz': 20.0,
            }]
        )
    ])
