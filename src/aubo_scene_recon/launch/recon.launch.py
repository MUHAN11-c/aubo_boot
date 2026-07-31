"""只启动 recon_fusion_node（不带臂、不带相机）."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('aubo_scene_recon'))
    default_params = str(pkg_share / 'config' / 'recon.yaml')

    # save_dir 默认空串：share 目录无法可靠反推工作区根，硬编码绝对路径不可移植；
    # 空串交由节点回退到 <进程CWD>/recon_maps（见 fusion_node.py 的 save_dir 处理）。
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='参数文件路径（默认包内 config/recon.yaml）。'),
        DeclareLaunchArgument(
            'save_dir',
            default_value='',
            description='地图保存目录；空串时节点回退到 <进程CWD>/recon_maps。'),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/camera/depth_registered/points',
            description='输入彩色点云话题（open3d 后端）。'),
        DeclareLaunchArgument(
            'backend',
            default_value='open3d',
            description='融合后端：open3d（点云累加）或 tsdf（RGB-D TSDF）。'),
        Node(
            package='aubo_scene_recon',
            executable='recon_fusion_node',
            name='recon_fusion_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'save_dir': LaunchConfiguration('save_dir'),
                    'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                    'backend': LaunchConfiguration('backend'),
                },
            ],
        ),
    ])
