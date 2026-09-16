"""启动桃子采摘完整业务栈（整栈入口；launch 不自动 RunHarvest）."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            'peach_harvester', 'scene_perception.launch.py',
            {'autostart': 'false', 'tool_profile': tool_profile}),
        _include(
            'peach_harvester', 'target_reconstruction.launch.py',
            {'autostart': 'false', 'tool_profile': tool_profile}),
        _include(
            'peach_arm', 'peach_arm.launch.py',
            {'autostart': 'false', 'tool_profile': tool_profile}),
        _include('peach_observability', 'observability.launch.py'),
        _include(
            'peach_harvester', 'peach_executor.launch.py',
            {
                'autostart': 'false',
                'require_managed_stack': 'true',
                'tool_profile': tool_profile,
            }),
        _include('peach_harvester', 'lifecycle_manager.launch.py'),
    ])
