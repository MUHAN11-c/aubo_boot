from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Name of the camera node'
        ),
        DeclareLaunchArgument(
            'status_publish_rate',
            default_value='1.0',
            description='Rate at which to publish camera status (Hz)'
        ),
        Node(
            package='percipio_camera_interface',
            executable='camera_control_node',
            name='camera_control_node',
            parameters=[{
                'camera_name': LaunchConfiguration('camera_name'),
                'status_publish_rate': LaunchConfiguration('status_publish_rate'),
            }],
            remappings=[
                # 话题重映射：将节点内部的话题名重映射到全局话题名
                ('camera_status', '/camera_status'),
                # 服务重映射：将节点内部的服务名重映射到全局服务名
                ('software_trigger', '/software_trigger'),
            ],
            output='screen'
        ),
    ])

