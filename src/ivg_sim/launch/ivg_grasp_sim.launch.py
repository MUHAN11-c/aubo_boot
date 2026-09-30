#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ivg_grasp_sim：ivg_sim 检测栈 + AUBO E5 机械臂抓取执行闭环.

组成：
  检测半场（与 ivg_table_sim 同链）：重力世界（ivg_table_grasp.sdf，对象可被
  搬动）+ /clock 桥 + 点云桥 + cloud_relay + 静态 TF world←camera_optical +
  检测节点（grasp_poses 输出帧=world）。
  执行半场：RSP（gz 机器人描述，world link 已剥）+ 静态 TF world→base_link +
  ros_gz_sim create spawn + gz_ros2_control 三控制器（JSB/JTC/夹爪）+
  move_group（管线/限位/IK 复用 aubo_e5_moveit_config，控制器映射=标准 JTC）。

帧约定：cell 根帧=``world``（桌面中心原点）。检测链 base_frame=world（与
机械臂 base_link 撞名的解法）；机械臂基座摆放单源=arm_description.SPAWN_XYZ
（gz spawn 位姿与静态 TF 同值）。

用法（工作区根，venv 链激活后）：
  ros2 launch ivg_sim ivg_grasp_sim.launch.py [gui:=true]
  # 另一终端（系统 python，无 venv 依赖）：
  ros2 run ivg_sim execute_grasp --target potted_meat_can
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import ivg_sim.arm_description as arm_description
from ivg_sim.generate_table_world import generate

PKG = 'ivg_sim'
GRAVITY_MPS2 = 9.81
SPAWN_DELAY_S = 2.0  # RSP 参数就绪 → create 之间的保守间隔


def _workspace_root() -> str:
    pkg_share = get_package_share_directory(PKG)
    return os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _resolve_grasp_world() -> str:
    """重力执行世界（无固定相机架——v2 腕上相机，真机同构 eye-in-hand）：
    缺则现场生成（幂等），源码树优先."""
    from pathlib import Path

    world = generate(gravity_mps2=GRAVITY_MPS2, camera_rig=False)
    return str(Path(world).resolve())


def _launch_setup(context: LaunchContext):
    pkg_share = get_package_share_directory(PKG)
    workspace = _workspace_root()

    # gz package://mesh 解析：GZ_SIM_RESOURCE_PATH 按目录名搜索，
    # 加 aubo_description 的 share 父目录使 package://aubo_description/... 可达
    aubo_share_parent = os.path.dirname(
        get_package_share_directory('aubo_description'))
    resource_path = os.pathsep.join(filter(None, [
        # 源码树优先（symlink-install 下 share 不含 worlds/models data_files）
        os.path.join(workspace, 'src', PKG, 'models'),
        os.path.join(workspace, 'src', PKG, 'worlds'),
        # <ws>/src：gz 把 package:// 转 model:// 后按「条目+包名」搜索，
        # package://ivg_sim/meshes/... 需要 <ws>/src 这一级作为条目
        os.path.join(workspace, 'src'),
        # 原装相机支架/快换盘件引用 package://tool_changer（gz 侧解析）
        os.path.join(workspace, 'install', 'tool_changer', 'share'),
        aubo_share_parent,
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    gui = LaunchConfiguration('gui').perform(context).strip().lower()
    world_path = _resolve_grasp_world()
    # gui 时省略 -s：`gz sim -r world` = 服务端+GUI 双拉起（-s 会吞掉 GUI，
    # ivg_table_sim 的 gui:=true 缺陷不在此重演）
    gz_args = (['-s'] if gui not in ('true', '1', 'yes') else []) + \
        ['-r', world_path]

    arm_urdf = arm_description.build_arm_urdf()
    spawn_x, spawn_y, spawn_z = arm_description.SPAWN_XYZ

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
            # 腕上相机（v2）：REP-103 光学系由 URDF 标定外参直挂（RSP 发 TF）
            'optical_frame': 'camera_color_optical_frame',
            'use_sim_time': True,
        }],
    )
    detect_node = ExecuteProcess(
        cmd=[
            LaunchConfiguration('detect_python'), '-m', 'ivg_graspnet.graspnet_node',
            '--ros-args',
            '--params-file', os.path.join(pkg_share, 'config', 'sim_graspnet.yaml'),
            # 本栈 v2 腕上相机：REP-103 光学帧（标定外参 URDF 直挂）；
            # 工作区放宽（拍照位视角与固定架不同，首轮实测后收紧）。
            # 检测门世界（ivg_table_sim）仍用 yaml 默认，两栈互不影响。
            '-p', 'base_frame:=world',
            '-p', 'frame_id:=camera_color_optical_frame',
            '-p', 'workspace:=0.2,1.6,-0.45,0.45,-0.50,0.50',
            '-p', ['backend:=', LaunchConfiguration('backend')],
            '-r', '__node:=graspnet_demo_points_node',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('detect')),
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': arm_urdf, 'use_sim_time': True}],
    )
    base_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='ivg_arm_base_tf', output='screen',
        arguments=[
            '--x', str(spawn_x), '--y', str(spawn_y), '--z', str(spawn_z),
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'world', '--child-frame-id', 'base_link',
        ],
    )
    create_entity = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'aubo_e5',
            '-world', 'ivg_table',
            '-x', str(spawn_x), '-y', str(spawn_y), '-z', str(spawn_z),
            '-R', '0', '-P', '0', '-Y', '0',
        ],
    )
    spawner_jsb = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster'],
    )
    spawner_jtc = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_trajectory_controller'],
    )
    spawner_gripper = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['gripper_position_controller'],
    )

    from ivg_sim.moveit_params import build_moveit_params
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        name='move_group', output='screen',
        parameters=[build_moveit_params()],
    )

    delayed_create = TimerAction(
        period=SPAWN_DELAY_S,
        actions=[create_entity],
    )
    # spawner 不等 create 退出：其内部本就轮询等待 controller_manager
    # 服务可用，立即起可把「spawn→JTC 激活」空窗压到最短（空窗内臂在
    # 重力下无约束漂移）。三路错峰 1s：controller_manager 在 gz 里加载
    # 网格时并发抢锁会 "Failed to acquire lock" 互踩致死（2026-09-30 实测）。
    # move_group 延迟 8s 等 JTC。
    spawners_staggered = [
        spawner_jsb,
        TimerAction(period=1.0, actions=[spawner_jtc]),
        TimerAction(period=2.0, actions=[spawner_gripper]),
    ]
    move_group_delayed = TimerAction(period=8.0, actions=[move_group])

    return [
        ExecuteProcess(
            cmd=['gz', 'sim'] + gz_args,
            output='screen',
            additional_env={'GZ_SIM_RESOURCE_PATH': resource_path},
        ),
        clock_bridge,
        cloud_bridge,
        relay,
        detect_node,
        rsp,
        base_tf,
        delayed_create,
        *spawners_staggered,
        move_group_delayed,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='true 时 gz sim 服务端+GUI 一起拉起（默认无头 -s）',
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
