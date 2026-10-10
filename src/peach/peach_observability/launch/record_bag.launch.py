"""独立 rosbag2 进程（MCAP）。有界关闭走进程 SIGINT，不经 Web 节点."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """ros2 bag record 独立进程."""
    output = LaunchConfiguration('output')
    return LaunchDescription([
        DeclareLaunchArgument(
            'output', default_value='runs/session_bag',
            description='rosbag2 输出目录'),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-s', 'mcap', '-o', output,
                '/peach_supervisor/state', '/peach_supervisor/events',
            ],
            output='screen',
        ),
    ])
