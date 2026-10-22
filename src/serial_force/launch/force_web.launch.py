"""五点力调试页。只订 /force/*，不打开串口，可与驱动并行。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """在 8099 起 force_web。页面和贴图装在 share/serial_force/web。"""
    port = LaunchConfiguration('port')
    script = PathJoinSubstitution([
        FindPackageShare('serial_force'), 'web', 'force_web.py'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='8099',
            description='五点力调试页端口'),
        ExecuteProcess(
            cmd=['python3', script, '--port', port],
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
        ),
    ])
