#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ivg_table_sim：IVG 抓取验证 Gazebo Harmonic 桌面场景一键起栈.

组成：gz sim（无头默认）→ /clock 桥 → 点云桥 → cloud_relay（帧重写）→
静态 TF base_link←camera_depth_optical_frame（layout 单源推导）→
可选 detect:=true 起 ivg_graspnet 检测节点（python3 -m 直跑，配合
aubo_py3.12 venv 运行链；参数走本包 config/sim_graspnet.yaml）。

前置：generate_table_world 已产 worlds/ivg_table.sdf（colcon --symlink-install
下 share 与源码树联动）；YCB meshes 已就位（models/fetch_ycb.sh）。

用法（工作区根，venv 链激活后）：
  ros2 launch ivg_sim ivg_table_sim.launch.py
  ros2 service call /graspnet_capture_control std_srvs/srv/SetBool "{data: true}"
  ros2 run ivg_sim score_grasps
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'ivg_sim'


def _workspace_root() -> str:
    """工作区根：pkg_share=<ws>/install/ivg_sim/share/ivg_sim → 上溯 4 级."""
    pkg_share = get_package_share_directory(PKG)
    return os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _resolve_world() -> str:
    """世界路径：源码树优先（symlink-install 下 share 不落 data_files）→ share 兜底."""
    pkg_share = get_package_share_directory(PKG)
    source = os.path.join(_workspace_root(), 'src', PKG, 'worlds', 'ivg_table.sdf')
    if os.path.exists(source):
        return source
    return os.path.join(pkg_share, 'worlds', 'ivg_table.sdf')


def _camera_tf_arguments() -> list:
    """静态 TF 参数（与生成器共用 layout.camera_optical_tf 推导）."""
    from ivg_sim.layout import GZ_OPTICAL_CONVENTION, camera_optical_tf, load_layout

    layout = load_layout()
    xyz, quat = camera_optical_tf(layout, GZ_OPTICAL_CONVENTION)
    return [
        '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
        '--qx', str(quat[0]), '--qy', str(quat[1]),
        '--qz', str(quat[2]), '--qw', str(quat[3]),
        '--frame-id', 'base_link',
        '--child-frame-id', layout.optical_frame,
    ]


def _launch_setup(context: LaunchContext):
    pkg_share = get_package_share_directory(PKG)
    workspace = _workspace_root()
    resource_path = os.pathsep.join(filter(None, [
        # 源码树优先（symlink-install 下 share 不含 worlds/models data_files）
        os.path.join(workspace, 'src', PKG, 'models'),
        os.path.join(workspace, 'src', PKG, 'worlds'),
        os.path.join(pkg_share, 'models'),
        os.path.join(pkg_share, 'worlds'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    gui = LaunchConfiguration('gui').perform(context).strip().lower()
    gz_args = ['-s'] + (['-g'] if gui in ('true', '1', 'yes') else []) + ['-r', _resolve_world()]

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ivg_clock_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
    )
    cloud_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ivg_cloud_bridge', output='screen',
        arguments=[
            '/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[('/camera/depth/points', '/ivg_sim/gz_points')],
    )
    relay = Node(
        package=PKG, executable='cloud_relay', name='ivg_cloud_relay',
        output='screen',
        parameters=[{
            'input_topic': '/ivg_sim/gz_points',
            'output_topic': '/camera/depth_registered/points',
            'optical_frame': 'camera_depth_optical_frame',
            'use_sim_time': True,
        }],
    )
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='ivg_camera_tf', output='screen',
        arguments=_camera_tf_arguments(),
    )
    detect_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration('detect_python'), '-m', 'ivg_graspnet.graspnet_node',
            '--ros-args',
            '--params-file', os.path.join(pkg_share, 'config', 'sim_graspnet.yaml'),
            '-p', ['backend:=', LaunchConfiguration('backend')],
            '-r', '__node:=graspnet_demo_points_node',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('detect')),
    )

    return [
        ExecuteProcess(
            cmd=['gz', 'sim'] + gz_args,
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': resource_path,
            },
        ),
        clock_bridge,
        cloud_bridge,
        relay,
        static_tf,
        detect_process,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='true 时带 GUI 起 gz sim（默认无头 -s）',
        ),
        DeclareLaunchArgument(
            'detect', default_value='true',
            description='true 时一并起 ivg_graspnet 检测节点（venv 运行链）',
        ),
        DeclareLaunchArgument(
            'backend', default_value='graspnet_torch',
            description='检测后端（graspnet_torch | contact_graspnet）',
        ),
        DeclareLaunchArgument(
            'detect_python', default_value='python3',
            description='检测节点解释器（默认 python3；须指向 aubo_py3.12 venv）',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
