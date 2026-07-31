"""
PeachPose 感知节点启动.

节点经安装空间的包装脚本（scripts/）以 aubo_py3.12 venv 解释器启动
（保证 numpy 1.26 + cv_bridge）。无相机冒烟用的数据集回放工具已独立为
tools/peach_dataset_replayer.py（不随 colcon 构建），需另开终端手动运行。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve(context, *args, **kwargs):
    share = get_package_share_directory('peach_pose_ros2')
    config = os.path.join(share, 'config', 'peach_pose.yaml')
    # python_executable 经 AUBO_PYTHON 传给包装脚本；空则脚本默认 aubo_py3.12 venv
    node_env = {}
    python_exe = LaunchConfiguration('python_executable').perform(context)
    if python_exe:
        node_env['AUBO_PYTHON'] = python_exe

    return [
        Node(
            package='peach_pose_ros2',
            executable='peach_pose_node',
            name='peach_pose_node',
            parameters=[config],
            output='screen',
            additional_env=node_env,
        )
    ]


def generate_launch_description():
    """生成 PeachPose 感知 launch 描述."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'python_executable',
            default_value='',
            description='Peach 节点使用的 Python 解释器（经包装脚本 AUBO_PYTHON 传入）；'
                        '空 → 工作区 aubo_py3.12/bin/python'),
        OpaqueFunction(function=_resolve),
    ])
