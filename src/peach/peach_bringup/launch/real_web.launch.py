"""真机台架一次拉起：臂、立体相机、8090 调试页、8099 五点力页。

不自动发 RunHarvest，不发运动，不 SetIO。授权只覆盖把栈拉起来。
收回位用现行拍照位 global_photo_pose（harvest_stow 是旧拍照位，已不再作为目标）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Include harvest_system 的真机参数，并带上五点力页。"""
    harvest = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('peach_bringup'), 'launch',
            'harvest_system.launch.py']),
        launch_arguments=[
            ('hardware_mode', 'real'),
            ('robot_ip', LaunchConfiguration('robot_ip')),
            ('camera_enabled', 'true'),
            ('camera_frontend', LaunchConfiguration('camera_frontend')),
            ('autostart', 'false'),
            ('force_enabled', 'true'),
            ('use_sim_time', 'false'),
            ('skip_reconstruction', LaunchConfiguration('skip_reconstruction')),
            ('rviz', LaunchConfiguration('rviz')),
        ],
    )
    force_web = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('serial_force'), 'launch',
            'force_web.launch.py']),
        launch_arguments=[
            ('port', LaunchConfiguration('force_web_port')),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='169.254.10.98',
            description='AUBO 控制器 IP'),
        DeclareLaunchArgument(
            'camera_frontend', default_value='stereo',
            description='相机前端，默认 stereo'),
        DeclareLaunchArgument(
            'skip_reconstruction', default_value='true',
            description='跳过 BuildTargetModel，用未精化场景观测进接触'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否起 RViz'),
        DeclareLaunchArgument(
            'force_web_port', default_value='8099',
            description='五点力调试页端口'),
        harvest,
        force_web,
    ])
