#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 测试 Launch 文件

功能：
  1. 启动测试节点，测试路径查找、模型加载、数据读取等功能
  2. 可选：启动 RViz2 可视化

使用方法：
  ros2 launch graspnet_ros2 test.launch.py
  或
  ros2 launch graspnet_ros2 test.launch.py test_prediction:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def get_package_share_directory(package_name):
    """获取包的共享目录（ROS2 标准方式）"""
    try:
        from ament_index_python.packages import get_package_share_directory as get_share_dir
        return get_share_dir(package_name)
    except (ImportError, ValueError):
        # 回退到源码目录（开发环境）
        # 从 launch 文件位置推断：launch/test.launch.py -> src/graspnet_ros2/
        launch_file_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.dirname(launch_file_dir)


def create_test_node(context):
    """创建测试节点的函数"""
    package_path = get_package_share_directory('graspnet_ros2')
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')
    
    # 获取源码目录 - 使用 __file__ 的绝对路径
    # __file__ 在 launch 文件中是安装后的路径，需要找到源码路径
    launch_file_abs = os.path.abspath(__file__)
    # 如果是安装目录，需要找到源码目录
    if 'install' in launch_file_abs:
        # 从 install 路径推断 src 路径
        ws_root = launch_file_abs.split('/install/')[0]
        src_package_path = os.path.join(ws_root, 'src', 'graspnet_ros2')
    else:
        # 源码目录
        launch_file_dir = os.path.dirname(launch_file_abs)
        src_package_path = os.path.dirname(launch_file_dir)
    
    # 查找测试脚本（优先使用源码目录，因为 test_node.py 可能没有安装）
    test_script_paths = [
        os.path.join(src_package_path, 'graspnet_ros2', 'test_node.py'),  # 源码目录（优先）
        os.path.join(package_path, 'graspnet_ros2', 'test_node.py'),  # 安装目录
    ]
    
    test_script_path = None
    for path in test_script_paths:
        if os.path.exists(path):
            test_script_path = path
            break
    
    # 如果都找不到，使用源码目录（开发环境）
    if not test_script_path:
        test_script_path = test_script_paths[0]
    
    # 获取 Python 解释器（优先使用 conda 环境）
    conda_prefix = os.environ.get('CONDA_PREFIX', '')
    if conda_prefix and os.path.exists(os.path.join(conda_prefix, 'bin', 'python3')):
        python_exec = os.path.join(conda_prefix, 'bin', 'python3')
    else:
        # 使用系统的 python3
        python_exec = 'python3'
    
    # 获取参数值
    test_model_load_val = LaunchConfiguration('test_model_load').perform(context)
    test_data_read_val = LaunchConfiguration('test_data_read').perform(context)
    test_prediction_val = LaunchConfiguration('test_prediction').perform(context)
    model_path_val = LaunchConfiguration('model_path').perform(context)
    data_dir_val = LaunchConfiguration('data_dir').perform(context)
    baseline_dir_val = LaunchConfiguration('baseline_dir').perform(context)
    
    # 构建参数字符串
    test_params = [
        '--ros-args',
        '-r', '__node:=graspnet_test_node',
        '-p', f'baseline_dir:={baseline_dir_val}',
        '-p', f'test_model_load:={test_model_load_val}',
        '-p', f'test_data_read:={test_data_read_val}',
        '-p', f'test_prediction:={test_prediction_val}',
    ]
    
    if model_path_val:
        test_params.extend(['-p', f'model_path:={model_path_val}'])
    if data_dir_val:
        test_params.extend(['-p', f'data_dir:={data_dir_val}'])
    
    # 设置环境变量
    env = dict(os.environ)
    # 添加源码目录到 PYTHONPATH
    if src_package_path not in env.get('PYTHONPATH', ''):
        current_pythonpath = env.get('PYTHONPATH', '')
        env['PYTHONPATH'] = src_package_path + (':' + current_pythonpath if current_pythonpath else '')
    
    # 设置工作目录为源码目录
    cwd = src_package_path if os.path.exists(src_package_path) else os.path.dirname(test_script_path) if test_script_path else package_path
    
    return [ExecuteProcess(
        cmd=[python_exec, test_script_path] + test_params,
        output='screen',
        cwd=cwd,
        env=env,
    )]


def generate_launch_description():
    package_path = get_package_share_directory('graspnet_ros2')
    
    # 声明启动参数
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'test_model_load',
            default_value='true',
            description='是否测试模型加载',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'test_data_read',
            default_value='true',
            description='是否测试数据读取',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'test_prediction',
            default_value='false',
            description='是否测试抓取预测（需要 GPU）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='模型权重文件路径（留空使用默认路径）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'data_dir',
            default_value='',
            description='数据目录路径（留空使用默认路径）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'baseline_dir',
            default_value=os.path.join(package_path, 'graspnet-baseline'),
            description='graspnet-baseline 目录路径（用于设置 Python 导入路径）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='是否启动 RViz2',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_path, 'config', 'demo.rviz'),
            description='RViz2 配置文件路径',
        )
    )
    
    # 获取参数值
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # 测试节点（使用 OpaqueFunction）
    test_node = OpaqueFunction(function=create_test_node)
    
    # RViz2 节点（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )
    
    # 静态 TF 发布器（如果使用 RViz）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera_frame'],
        output='screen',
        condition=IfCondition(use_rviz),
    )
    
    return LaunchDescription(
        declared_arguments + [
            test_node,
            rviz_node,
            static_tf_node,
        ]
    )
