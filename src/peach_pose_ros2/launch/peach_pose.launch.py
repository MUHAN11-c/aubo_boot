"""
PeachPose 感知节点启动.

节点为标准 console_scripts 入口，其 shebang 在构建期由 setup.py 的
options.build_scripts.executable 指向 aubo_py3.12 venv 解释器
（保证 numpy 1.26 + cv_bridge + torch）。无相机冒烟用的数据集回放工具
为 tools/peach_dataset_replayer.py（不随 colcon 构建），需另开终端手动运行。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成 PeachPose 感知 launch 描述."""
    share = get_package_share_directory('peach_pose_ros2')
    config = os.path.join(share, 'config', 'peach_pose.yaml')

    return LaunchDescription([
        Node(
            package='peach_pose_ros2',
            executable='peach_pose_node',
            name='peach_pose_node',
            parameters=[config],
            output='screen',
        )
    ])
