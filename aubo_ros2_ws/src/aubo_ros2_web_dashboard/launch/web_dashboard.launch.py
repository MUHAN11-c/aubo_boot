"""Launch rosbridge (rosbridge_suite), rosapi, tf2_web_republisher, and static HTTP for RWT demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aubo_ros2_web_dashboard')
    web_dir = os.path.join(pkg_share, 'web', 'ros2_web_bridge_demo')
    rosbridge_xml = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )

    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    return LaunchDescription(
        [
            DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('web_port', default_value='8090'),
            DeclareLaunchArgument('rosbridge_port', default_value='9090'),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(rosbridge_xml),
                launch_arguments={'port': rosbridge_port}.items(),
            ),
            Node(
                package='tf2_web_republisher',
                executable='tf2_web_republisher_node',
                name='tf2_web_republisher',
                output='screen',
            ),
            ExecuteProcess(
                cmd=[
                    'python3',
                    '-m',
                    'http.server',
                    web_port,
                    '--bind',
                    web_host,
                    '--directory',
                    web_dir,
                ],
                output='screen',
            ),
        ]
    )
