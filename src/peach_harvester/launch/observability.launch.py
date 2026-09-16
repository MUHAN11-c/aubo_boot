"""Thin forwarder: observability node now lives in peach_observability."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """转发到 peach_observability；不自动 RunHarvest."""
    config = PathJoinSubstitution([
        FindPackageShare('peach_harvester'),
        'config', 'observability.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=config,
            description='只读监控参数文件；默认包内 config/observability.yaml'),
        DeclareLaunchArgument(
            'host', default_value='',
            description='覆盖监听地址；空串使用 YAML'),
        DeclareLaunchArgument(
            'port', default_value='0',
            description='覆盖监听端口；0 使用 YAML'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='false 时保持 Unconfigured，须外部 configure/activate'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('peach_observability'),
                    'launch', 'observability.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'autostart': LaunchConfiguration('autostart'),
                'record_bag': 'false',
            }.items()),
    ])
