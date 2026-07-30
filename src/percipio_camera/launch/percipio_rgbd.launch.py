# Percipio RGB-D 启动封装：在现有 percipio_camera.launch.py 上强制打开
# 深度、深度到 color 配准与点云，供 peach_pose 等需要逐像素对齐 RGB-D 的节点使用。
# 不改变默认 RGB-only launch，避免影响手眼标定路径。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 允许覆盖设备 IP 等常用参数；深度相关三项在 include 时强制为 true。
    device_ip = LaunchConfiguration('device_ip')
    camera_name = LaunchConfiguration('camera_name')
    color_resolution = LaunchConfiguration('color_resolution')
    depth_resolution = LaunchConfiguration('depth_resolution')
    color_camera_info_file = LaunchConfiguration('color_camera_info_file')

    percipio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('percipio_camera'),
                'launch',
                'percipio_camera.launch.py',
            ])),
        launch_arguments={
            'device_ip': device_ip,
            'camera_name': camera_name,
            'color_enable': 'true',
            'color_resolution': color_resolution,
            'color_camera_info_file': color_camera_info_file,
            'depth_enable': 'true',
            'depth_resolution': depth_resolution,
            'depth_registration_enable': 'true',
            'point_cloud_enable': 'true',
            'color_point_cloud_enable': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('device_ip', default_value='169.254.10.110'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('color_resolution', default_value='640x480'),
        DeclareLaunchArgument('depth_resolution', default_value='640x400'),
        DeclareLaunchArgument(
            'color_camera_info_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('percipio_camera'),
                'config',
                'color_camera_info.yaml',
            ])),
        percipio,
    ])
