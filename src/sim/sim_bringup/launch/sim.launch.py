"""
统一机体仿真主入口（P2）：可驾驶履带 + gz_ros2_control 臂 + MoveIt（可选）.

组成：
1. ``gz sim -r <world>``（默认程序化果园 ``peach_orchard.sdf``；无头 ``-s``）；
2. ``robot_state_publisher``（``harvester_unified.urdf.xacro``：platform_base_link
   根 + 臂链 + 相机帧链）+ ``ros_gz_sim create`` 走廊中心落地 spawn；
3. ``ros_gz_bridge``（``sim_bridge.yaml``：clock/odom/tf/scan 入、cmd_vel 出；
   ``/joint_states`` 由 gz_ros2_control 的 JSB 原生发布，不经桥）；
4. 控制器 spawner（JSB + JTC，延迟到实体生成后； peach_orchard 首载 40-60 s）；
5. ``moveit:=true``（默认）include ``aubo_e5_moveit_config/moveit.launch.py``，
   ``controllers_file:=controllers_mock.yaml``——仿真 JTC 与 mock 同名同关节，
   映射零新文件（ivg_sim 同款复用口径）。

手动验收：``ros2 topic echo /joint_states --once``（六 AUBO 名）；MoveIt RViz
plan&execute 或 ``ros2 action send_goal /joint_trajectory_controller/
follow_joint_trajectory``；``ros2 topic pub /cmd_vel`` 驾驶。测完 Ctrl+C 停栈并
pgrep 清残留（MUST）。
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

from sim_worlds.params import params_from_dict
from sim_worlds.scene import PLATFORM_MODEL, work_pose

import yaml


def _resource_path(aubo_share: Path, desc_share: Path) -> str:
    """让 gz 解析 model://aubo_description 与 model://sim_description."""
    entries = [str(aubo_share.parent), str(desc_share.parent)]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        entries.append(existing)
    return os.pathsep.join(dict.fromkeys(entries))


def _setup(context, *args, **kwargs):
    worlds_share = Path(get_package_share_directory('sim_worlds'))
    desc_share = Path(get_package_share_directory('sim_description'))
    aubo_share = Path(get_package_share_directory('aubo_description'))
    scene_path = Path(LaunchConfiguration('scene_params').perform(context))
    world_path = Path(LaunchConfiguration('world').perform(context))
    tool_profile = LaunchConfiguration('tool_profile').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')
    moveit = LaunchConfiguration('moveit').perform(context).lower() in ('true', '1')
    spawn_mode = LaunchConfiguration('spawn').perform(context)

    raw = yaml.safe_load(scene_path.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw if isinstance(raw, dict) else {})
    if problems:
        raise RuntimeError(
            f'{scene_path} 场景参数不合法：' + '；'.join(problems))
    pose = work_pose(params)
    platform = params.platform

    robot_description = ParameterValue(
        Command([
            'xacro ', str(desc_share / 'urdf' / 'harvester_unified.urdf.xacro'),
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

    # 落地 spawn：spawn_x/y/yaw 显式覆盖（field 世界 = generate_field 站位）；
    # 否则 work=作业位（0.95 m，感知深度窗/臂展/拍照位标定；默认）；
    # corridor=行间走廊中心（程序化世界导航起步位：作业位车侧缘距树干膨胀圈
    # 仅 ~2.5 cm 骑在障碍上，起步干净——orchard_nav 用）。
    spawn_x_arg = LaunchConfiguration('spawn_x').perform(context)
    spawn_y_arg = LaunchConfiguration('spawn_y').perform(context)
    spawn_yaw_arg = LaunchConfiguration('spawn_yaw').perform(context)
    if spawn_x_arg and spawn_y_arg:
        spawn_xyz = (float(spawn_x_arg), float(spawn_y_arg), 0.0)
        spawn_yaw = (float(spawn_yaw_arg) if spawn_yaw_arg
                     else 0.0)
    elif spawn_mode == 'corridor':
        spawn_xyz = (
            pose.vehicle_center[0] + params.rows.spacing / 2.0
            - params.platform.row_gap,
            pose.vehicle_center[1], 0.0)
        spawn_yaw = pose.vehicle_yaw
    else:
        spawn_xyz = (pose.mount[0], pose.mount[1], 0.0)
        # 车头 = 行驶朝向；转台 -32° 后臂基落在 mount_yaw（与锚定模式逐位一致）
        spawn_yaw = pose.vehicle_yaw

    nodes = [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=_resource_path(aubo_share, desc_share)),
        SetParameter('use_sim_time', True),
        LogInfo(msg=[
            f'统一机体场景：{world_path}；spawn xyz=({spawn_xyz[0]:.3f}, '
            f'{spawn_xyz[1]:.3f}, {spawn_xyz[2]:.3f}) yaw={spawn_yaw:.3f} rad；',
            'moveit' if moveit else '无 moveit']),
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
                '-x', str(spawn_xyz[0]),
                '-y', str(spawn_xyz[1]),
                '-z', str(spawn_xyz[2]),
                '-Y', str(spawn_yaw),
            ]),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{
                'config_file': str(Path(get_package_share_directory('sim_bringup'))
                                   / 'config' / 'sim_bridge.yaml'),
                'use_sim_time': True}]),
        # odom→platform_base_link TF 中继：OdometryPublisher 的 gz 帧名
        # （harvester/base_footprint）与 ROS 帧名不一致，TF 树在此断裂
        Node(
            package='sim_bringup',
            executable='odom_tf_relay',
            output='screen',
            parameters=[{'use_sim_time': True}]),
        # 控制器加载在实体生成之后（重世界首载 40-60 s；gz 内 controller_manager
        # 冷启动服务调用 >10s，必须拉长 spawner 超时，否则 load 重试撞 already-loaded）
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=[
                        'joint_state_broadcaster',
                        'joint_trajectory_controller',
                        '--service-call-timeout', '60',
                        '--switch-timeout', '30',
                    ]),
            ]),
    ]

    if moveit:
        moveit_share = Path(get_package_share_directory('aubo_e5_moveit_config'))
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(moveit_share / 'launch' / 'moveit.launch.py')),
                launch_arguments={
                    'controllers_file': 'controllers_mock.yaml',
                    'tool_profile': tool_profile,
                    'standalone_state_publishers': 'false',
                    'rviz': 'false',
                }.items()))

    return nodes


def generate_launch_description() -> LaunchDescription:
    worlds_share = Path(get_package_share_directory('sim_worlds'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'scene_params',
            default_value=str(worlds_share / 'config' / 'orchard.yaml'),
            description='场景参数 yaml（几何/摆位唯一事实源，sim_worlds）'),
        DeclareLaunchArgument(
            'world',
            default_value=str(worlds_share / 'worlds' / 'peach_orchard.sdf'),
            description='世界 SDF（peach_orchard 带地面碰撞；'
                        'field_no_grass 需配地面碰撞版）'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='gz 仿真器自带 GUI（默认无头 -s，可视化用 RViz）'),
        DeclareLaunchArgument(
            'moveit', default_value='true',
            description='include aubo_e5_moveit_config（JTC 映射复用 '
                        'controllers_mock.yaml）'),
        DeclareLaunchArgument(
            'spawn', default_value='work',
            description='work=作业位 0.95m（感知标定位）；corridor=走廊中心'
                        '（程序化世界导航起步位，orchard_nav 用）'),
        DeclareLaunchArgument(
            'spawn_x', default_value='',
            description='显式 spawn x（field 世界 = generate_field 站位；'
                        '给出则覆盖 spawn 模式计算）'),
        DeclareLaunchArgument('spawn_y', default_value='', description='同上 y'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='', description='同上 yaw (rad)'),
        DeclareLaunchArgument(
            'tool_profile', default_value='bite_shear_v1',
            description='peach 真实末端（bite_shear_v1 / shear_v1 / '
                        'adaptive_shear_v1）'),
        OpaqueFunction(function=_setup),
    ])
