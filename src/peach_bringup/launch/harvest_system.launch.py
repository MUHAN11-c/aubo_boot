"""
启动桃子采摘完整业务栈（整栈入口；autostart 参数控制是否自动开批）.

清洁重写轮阶段 5：生命周期管理换 nav2_lifecycle_manager（bond_timeout=0.0
管 rclpy 节点；名单/顺序语义同原自研件；进程死检由 supervisor
HeartbeatWatchdog 承担）+ lifecycle_flag_bridge 把 is_active 桥接为闩锁
/peach/lifecycle/managed_nodes_activated（消费方零改动）。autostart 客户端
在栈就绪后自动发 RunHarvest——原「launch 绝不自动开批」红线已按用户核定
删除（2026-09-16），授权语义=操作员发起 launch（红线 3）；默认关，部署
档自开。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from peach_bringup.preflight import running_stack_pids


def _include(package, launch_file, launch_arguments=None, condition=None):
    """按包 share 目录包含一个 launch 文件."""
    kwargs = {}
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare(package), 'launch', launch_file]),
        launch_arguments=(launch_arguments or {}).items(),
        **kwargs,
    )


def _preflight(context):
    """Launch 展开期前置检查：发现旧实例则打印 PID 并拒绝启动."""
    del context
    stale = running_stack_pids()
    if stale:
        lines = ['检测到旧实例仍在运行，拒绝重复启动；先清理：']
        lines += [f'  kill -9 {pid}  # {cmd}' for pid, cmd in stale[:12]]
        text = '\n'.join(lines)
        print(f'\n[preflight] {text}\n', flush=True)
        raise RuntimeError(text)
    return []


def generate_launch_description():
    """构造从 ros2_control 到 Web 的完整采摘系统入口."""
    hardware_mode = LaunchConfiguration('hardware_mode')
    robot_ip = LaunchConfiguration('robot_ip')
    moveit_enabled = LaunchConfiguration('moveit_enabled')
    camera_enabled = LaunchConfiguration('camera_enabled')
    extrinsics_enabled = LaunchConfiguration('extrinsics_enabled')
    hand_eye_enabled = LaunchConfiguration('hand_eye_enabled')
    hand_eye_web_enabled = LaunchConfiguration('hand_eye_web_enabled')
    tool_profile = LaunchConfiguration('tool_profile')
    autostart = LaunchConfiguration('autostart')
    return LaunchDescription([
        OpaqueFunction(function=_preflight),
        DeclareLaunchArgument(
            'hardware_mode', default_value='mock',
            choices=['mock', 'real'],
            description='硬件模式；默认 mock；真机须显式 hardware_mode:=real'),
        DeclareLaunchArgument(
            'robot_ip', default_value='169.254.10.98',
            description='AUBO 控制器 IP；mock 模式不使用'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_cylinder_v1',
            choices=['hollow_cylinder_v1', 'adaptive_cylinder_v1'],
            description='末端工具档案'),
        DeclareLaunchArgument(
            'autostart', default_value='false',
            description='true 时托管栈就绪后自动发 RunHarvest（授权=操作员'
                        '发起本 launch，红线 3；默认关，部署档自定）'),
        DeclareLaunchArgument(
            'moveit_enabled', default_value='true',
            description='启动 MoveIt move_group 和 RViz2'),
        DeclareLaunchArgument(
            'camera_enabled', default_value='false',
            description='启动 Percipio 相机'),
        DeclareLaunchArgument(
            'extrinsics_enabled', default_value='true',
            description='启动手眼外参静态 TF'),
        DeclareLaunchArgument(
            'hand_eye_enabled', default_value='false',
            description='启动手眼标定采集流程'),
        DeclareLaunchArgument(
            'hand_eye_web_enabled', default_value='false',
            description='启动手眼标定 Web'),
        DeclareLaunchArgument(
            'imu_enabled', default_value='true',
            description='启动 USB 串口 IMU；不进 lifecycle'),
        _include(
            'aubo_e5_bringup', 'bringup.launch.py', {
                'hardware_mode': hardware_mode,
                'robot_ip': robot_ip,
                'tool_profile': tool_profile,
                'moveit_enabled': moveit_enabled,
                'camera_enabled': camera_enabled,
                'extrinsics_enabled': extrinsics_enabled,
                'hand_eye_enabled': hand_eye_enabled,
                'hand_eye_web_enabled': hand_eye_web_enabled,
            }),
        _include(
            'serial_imu', 'serial_imu.launch.py', {
                'use_rviz': 'false',
                'tf_parent_frame': 'tcp',
                'align_to_parent': 'true',
            },
            condition=IfCondition(LaunchConfiguration('imu_enabled'))),
        _include(
            'peach_harvester', 'brain.launch.py',
            {'require_managed_stack': 'true', 'tool_profile': tool_profile}),
        _include(
            'peach_arm', 'peach_arm.launch.py',
            {'autostart': 'false', 'tool_profile': tool_profile}),
        _include('peach_observability', 'observability.launch.py'),
        # 阶段 5：nav2_lifecycle_manager 替自研件（bond_timeout=0.0 管
        # rclpy；名单顺序=场景→重建→技能→调度；进程死检=watchdog）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='peach_lifecycle_manager',
            output='screen',
            parameters=[{
                'node_names': [
                    'peach_scene_perception_node',
                    'peach_target_reconstruction_node',
                    'peach_arm',
                    'peach_supervisor',
                ],
                'autostart': True,
                'bond_timeout': 0.0,
            }]),
        # is_active → 闩锁 managed_nodes_activated 桥（消费方零改动）
        Node(
            package='peach_bringup',
            executable='peach_lifecycle_flag_bridge',
            name='peach_lifecycle_flag_bridge',
            output='screen'),
        # mock 初始位姿：xacro state_interface initial_value（SRDF 拍照位）
        # 官方 ros2_control 机制，非轨迹——详见 aubo_e5.ros2_control.xacro
        # autostart 客户端（默认关；授权=操作员发起 launch，红线 3）
        Node(
            package='peach_bringup',
            executable='peach_autostart_client',
            name='peach_autostart_client',
            output='screen',
            condition=IfCondition(autostart)),
    ])
