"""只启动 recon_fusion_node（不带臂、不带相机）。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('aubo_scene_recon'))
    default_params = str(pkg_share / 'config' / 'recon.yaml')

    # 工作区根：.../src/aubo_scene_recon/share/... 无法可靠反推；
    # 用常见绝对路径；也可 launch 参数覆盖。
    default_save = '/home/mu/Desktop/aubo_e5_jazzy_ws/recon_maps'

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('save_dir', default_value=default_save),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/camera/depth_registered/points'),
        DeclareLaunchArgument('backend', default_value='open3d'),
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
