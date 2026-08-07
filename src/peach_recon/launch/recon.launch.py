"""
peach_recon 重建精化启动（P0 骨架）.

启动 recon_refiner_node（console_scripts 入口，shebang 在构建期由
setup.py 的 options.build_scripts.executable 指向 aubo_py3.12 venv
解释器；约定同 peach_pose_ros2 / aubo_scene_recon）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成 peach_recon P0 骨架 launch 描述."""
    share = get_package_share_directory('peach_recon')
    config = os.path.join(share, 'config', 'recon.yaml')

    return LaunchDescription([
        Node(
            package='peach_recon',
            executable='recon_refiner_node',
            name='recon_refiner_node',
            parameters=[config],
            output='screen',
        )
    ])
