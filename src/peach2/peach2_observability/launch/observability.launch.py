"""Standalone peach2_observability (regular node, not lifecycle-managed)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution(
        [FindPackageShare('peach2_observability'), 'config', 'observability.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=config_file),
        Node(
            package='peach2_observability',
            executable='peach2_observability',
            name='peach2_observability',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
