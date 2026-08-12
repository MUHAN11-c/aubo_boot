"""
桃子局部重建节点启动.

节点为标准 console_scripts 入口，其 shebang 在构建期由 setup.py 的
options.build_scripts.executable 指向 aubo_py3.12 venv 解释器
（保证 numpy 1.26 + cv_bridge + open3d）。默认自动加载包内
config/reconstruction.yaml；reconstruction_params_file launch 参数可覆盖。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成局部重建 launch 描述."""
    share = get_package_share_directory('peach_reconstruction_ros2')
    config = os.path.join(share, 'config', 'reconstruction.yaml')

    reconstruction_params_file = LaunchConfiguration('reconstruction_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'reconstruction_params_file', default_value=config,
            description='重建参数文件；默认包内 config/reconstruction.yaml'),
        Node(
            package='peach_reconstruction_ros2',
            executable='peach_reconstruction_node',
            name='peach_reconstruction_node',
            parameters=[reconstruction_params_file],
            output='screen',
        )
    ])
