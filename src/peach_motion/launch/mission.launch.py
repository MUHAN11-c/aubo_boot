"""
peach_motion 任务编排启动（P0 骨架）.

启动 mission_node（console_scripts 标准入口，系统 python3）：
同进程组合 MissionManager（注册表/占位状态机/registry+stats 发布）
与 PickExecutor（~/pick 假桩 action server）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成 peach_motion P0 任务编排 launch 描述."""
    share = get_package_share_directory('peach_motion')
    config = os.path.join(share, 'config', 'mission.yaml')

    return LaunchDescription([
        Node(
            package='peach_motion',
            executable='mission_node',
            parameters=[config],
            output='screen',
        )
    ])
