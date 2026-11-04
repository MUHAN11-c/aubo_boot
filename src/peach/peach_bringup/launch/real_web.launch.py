"""
真机台架一次拉起：臂、立体相机、8090 调试页（含五点力页）.

不自动发 RunHarvest，不发运动，不 SetIO。授权只覆盖把栈拉起来。
收回位用现行拍照位 global_photo_pose（harvest_stow 是旧拍照位，已不再作为目标）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Include the real harvest stack; Web is consolidated on port 8090."""
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
            ('reconstruction_enabled', LaunchConfiguration('reconstruction_enabled')),
            ('skip_reconstruction', LaunchConfiguration('skip_reconstruction')),
            ('task_params_file', LaunchConfiguration('task_params_file')),
            ('rviz', LaunchConfiguration('rviz')),
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
            'reconstruction_enabled', default_value='true',
            description='是否启动 target_reconstruction 进程；false 时强制 skip'),
        DeclareLaunchArgument(
            'skip_reconstruction', default_value='false',
            description='与整栈共用的显式重建选项；默认完整重建'),
        DeclareLaunchArgument(
            'task_params_file', default_value='',
            description='与 mock 共用的标准 ROS 作业参数 YAML'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否起 RViz'),
        harvest,
    ])
