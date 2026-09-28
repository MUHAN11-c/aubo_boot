"""
室外套袋桃果园场景 launch（Gazebo Harmonic）.

三件事：
1. ``gz sim`` 起 ``worlds/peach_orchard.sdf``（``gui:=false`` 走 ``-s`` 无头）；
2. ``robot_state_publisher`` + ``ros_gz_sim create`` 把采摘工位（开源履带底盘
   改型 + AUBO E5 + 套袋刀）生成到 ``config/orchard.yaml`` 的 ``platform`` 作业位；
3. ``ros_gz_bridge`` 桥 ``/clock`` 与关节状态（``gz.msgs.Model`` →
   ``sensor_msgs/msg/JointState``），RSP 出的 TF 与 gz 内几何同位姿。

场景几何/摆位唯一事实源是 ``config/orchard.yaml``：改挂果、树行或工位后先
``ros2 run peach_sim generate_orchard`` 重生成世界与目标清单，再起本 launch。
本轮是场景建模：工位经 URDF ``world_joint`` 锚在作业位（履带不可驾驶），
``gz_ros2_control`` 接臂与履带驱动属后续轮（须授权动只读 bringup 面）。
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from peach_sim.params import params_from_dict
from peach_sim.scene import PLATFORM_MODEL, work_pose

import yaml


def _resource_path(aubo_share: Path, sim_share: Path) -> str:
    """让 gz 解析 model://aubo_description/...：share 的父目录要进资源路径."""
    entries = [str(aubo_share.parent), str(sim_share.parent), str(sim_share)]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        entries.append(existing)
    return os.pathsep.join(dict.fromkeys(entries))


def _setup(context, *args, **kwargs):
    sim_share = Path(get_package_share_directory('peach_sim'))
    aubo_share = Path(get_package_share_directory('aubo_description'))
    scene_path = Path(LaunchConfiguration('scene_params').perform(context))
    world_path = Path(LaunchConfiguration('world').perform(context))
    tool_profile = LaunchConfiguration('tool_profile').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')

    raw = yaml.safe_load(scene_path.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw if isinstance(raw, dict) else {})
    if problems:
        raise RuntimeError(
            f'{scene_path} 场景参数不合法：' + '；'.join(problems))
    pose = work_pose(params)
    platform = params.platform

    robot_description = ParameterValue(
        Command([
            'xacro ', str(sim_share / 'urdf' / 'harvester_robot.urdf.xacro'),
            ' tool_profile:=', tool_profile,
            ' arm_mount_height:=', str(platform.arm_mount_height),
            ' arm_mount_yaw_deg:=', str(platform.arm_mount_yaw_deg),
            ' body_x:=', str(platform.body_size[0]),
            ' body_y:=', str(platform.body_size[1]),
            ' body_z:=', str(platform.body_size[2]),
            ' body_bottom:=', str(platform.body_bottom),
            ' track_length:=', str(platform.track_length),
            ' track_width:=', str(platform.track_width),
            ' track_height:=', str(platform.track_height),
            ' track_separation:=', str(platform.track_separation),
        ]),
        value_type=str)

    gz_args = ['-r', str(world_path)]
    if not gui:
        gz_args.insert(0, '-s')

    return [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=_resource_path(aubo_share, sim_share)),
        LogInfo(msg=[
            f'果园场景：{world_path}；工位 spawn xyz=({pose.mount[0]:.3f}, '
            f'{pose.mount[1]:.3f}, {pose.mount[2]:.3f}) '
            f'yaw={pose.mount_yaw:.3f} rad']),
        ExecuteProcess(
            cmd=['gz', 'sim', *gz_args], output='screen', shell=False),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }]),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', PLATFORM_MODEL,
                '-topic', 'robot_description',
                '-x', str(pose.mount[0]),
                '-y', str(pose.mount[1]),
                '-z', str(pose.mount[2]),
                '-Y', str(pose.mount_yaw),
            ]),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                # GZ→ROS：仿真时钟与关节状态（RSP 据此发 TF）
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/peach_sim/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            remappings=[('/peach_sim/joint_states', '/joint_states')]),
    ]


def generate_launch_description() -> LaunchDescription:
    sim_share = Path(get_package_share_directory('peach_sim'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'scene_params', default_value=str(sim_share / 'config' / 'orchard.yaml'),
            description='场景参数 yaml（几何/摆位唯一事实源）'),
        DeclareLaunchArgument(
            'world', default_value=str(sim_share / 'worlds' / 'peach_orchard.sdf'),
            description='世界 SDF（generate_orchard 的产物）'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false=无头 gz sim -s（CI/远程）'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_shear_v1',
            description='末端工具档案（同整栈 tool_profile 语义）'),
        OpaqueFunction(function=_setup),
    ])
