"""PeachPose 感知节点启动。

默认用 aubo_py3.12 解释器跑节点（保证 numpy 1.26 + cv_bridge）；
replay:=true 时同时起 dataset_replayer（无相机冒烟）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve(context, *args, **kwargs):
    share = get_package_share_directory('peach_pose_ros2')
    config = os.path.join(share, 'config', 'peach_pose.yaml')
    python_exe = LaunchConfiguration('python_executable').perform(context)
    replay = LaunchConfiguration('replay').perform(context).lower() in (
        'true', '1', 'yes')
    limit = LaunchConfiguration('replay_limit').perform(context)
    dataset = LaunchConfiguration('dataset').perform(context)
    ws = os.environ.get(
        'AUBO_WS', os.path.abspath(os.path.join(share, '..', '..', '..', '..')))
    # share = install/peach_pose_ros2/share/peach_pose_ros2 → ws 根约四级上
    if not os.path.isdir(ws) or not os.path.isfile(
            os.path.join(ws, 'aubo_py3.12', 'bin', 'python')):
        ws = '/home/mu/Desktop/aubo_e5_jazzy_ws'
    if not python_exe:
        python_exe = os.path.join(ws, 'aubo_py3.12', 'bin', 'python')

    # 用 ExecuteProcess 指定解释器，避免 ros2 run 落到系统 python
    node_cmd = [
        python_exe, '-m', 'peach_pose_ros2.peach_pose_node',
        '--ros-args',
        '--params-file', config,
        '-r', '__node:=peach_pose_node',
    ]
    actions = [
        ExecuteProcess(
            cmd=node_cmd,
            output='screen',
            additional_env={
                # 确保能 import install 空间
                'PYTHONPATH': os.environ.get('PYTHONPATH', ''),
            },
        )
    ]

    if replay:
        if not dataset:
            dataset = os.path.join(
                ws, 'src', 'peach_pose_ros2', 'data', 'dataset')
            if not os.path.isdir(dataset):
                dataset = '/home/mu/Desktop/demo(1)/peach_canopy/data/dataset'
        replay_loop = LaunchConfiguration('replay_loop').perform(context).lower() in (
            'true', '1', 'yes')
        replay_cmd = [
            python_exe, '-m', 'peach_pose_ros2.dataset_replayer',
            '--dataset', dataset,
            '--limit', limit,
            '--rate', '0.5',
        ]
        if replay_loop:
            replay_cmd.append('--loop')
        replay_cmd += [
            '--ros-args',
            '-r', '__node:=peach_pose_dataset_replayer',
        ]
        actions.append(ExecuteProcess(cmd=replay_cmd, output='screen'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'python_executable',
            default_value='',
            description='Python for peach nodes; empty → aubo_py3.12/bin/python'),
        DeclareLaunchArgument('replay', default_value='false'),
        DeclareLaunchArgument('replay_limit', default_value='3'),
        DeclareLaunchArgument('replay_loop', default_value='true'),
        DeclareLaunchArgument('dataset', default_value=''),
        OpaqueFunction(function=_resolve),
    ])
