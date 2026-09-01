"""启动桃子采摘完整业务栈."""

from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from peach_executor.batch import default_runs_root


def _include(package, launch_file, launch_arguments=None):
    """按包 share 目录包含一个 launch 文件."""
    return IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare(package), 'launch', launch_file]),
        launch_arguments=(launch_arguments or {}).items(),
    )


# 启动预检（防重复实例）：扫描 /proc 中仍在运行的栈节点进程。
# 历史教训——「重启」若未杀净 venv python 节点，新旧同名实例并存会
# 交错发布同名话题（RViz 闪烁、参数互答错乱），故发现即拒启并列出 PID。
_PREFLIGHT_PATTERNS = (
    'peach_scene_perception_node', 'peach_target_reconstruction_node',
    'peach_manipulation_node', 'peach_executor', 'peach_observability',
    'peach_lifecycle_manager', 'component_container', 'move_group',
    # bringup 侧（旧实例会发旧命名 TF 链，如 link1/link2/tip，污染树）
    'robot_state_publisher', 'ros2_control_node', 'controller_manager',
    'joint_state_publisher',
    # 多代 extrinsics 会叠发 wrist3→camera_link，重建精确 stamp 积分会偏
    'extrinsics_publisher',
)


def _running_stack_pids():
    """返回 (pid, cmdline) 列表：仍在跑的栈节点进程（cmdline 扫描）."""
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as stream:
                cmdline = stream.read().decode('utf-8', 'replace')
        except OSError:
            continue
        if 'harvest_system.launch' in cmdline:
            continue  # 本次启动自身的 launch 进程链在扫描期间可能尚未退出
        for pattern in _PREFLIGHT_PATTERNS:
            if pattern in cmdline and 'cursorsandbox' not in cmdline:
                found.append((int(entry), cmdline.split('\x00')[0][:100]))
                break
    return found


def _preflight(context):
    """Launch 展开期前置检查：发现旧实例则打印 PID 并拒绝启动."""
    stale = _running_stack_pids()
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
    return LaunchDescription([
        OpaqueFunction(function=_preflight),
        DeclareLaunchArgument(
            'hardware_mode', default_value='mock',
            choices=['mock', 'real'],
            description='硬件模式；默认 mock（标准仿真），真机须显式 hardware_mode:=real'),
        DeclareLaunchArgument(
            'robot_ip', default_value='169.254.10.98',
            description='AUBO 控制器 IP；mock 模式不使用'),
        DeclareLaunchArgument(
            'moveit_enabled', default_value='true',
            description='启动 MoveIt move_group 和 RViz2'),
        DeclareLaunchArgument(
            'camera_enabled', default_value='false',
            description='启动 Percipio 相机；真机有设备时显式设为 true'),
        DeclareLaunchArgument(
            'extrinsics_enabled', default_value='true',
            description='启动手眼外参静态 TF'),
        DeclareLaunchArgument(
            'hand_eye_enabled', default_value='false',
            description='启动手眼标定采集流程'),
        DeclareLaunchArgument(
            'hand_eye_web_enabled', default_value='false',
            description='启动手眼标定 Web；仅 hand_eye_enabled 生效'),
        DeclareLaunchArgument(
            'record_mcap', default_value='false',
            description='为 true 时用 ros2 bag record -s mcap 录执行器/感知/重建关键话题'),
        _include(
            'aubo_e5_bringup', 'bringup.launch.py', {
                'hardware_mode': hardware_mode,
                'robot_ip': robot_ip,
                'moveit_enabled': moveit_enabled,
                'camera_enabled': camera_enabled,
                'extrinsics_enabled': extrinsics_enabled,
                'hand_eye_enabled': hand_eye_enabled,
                'hand_eye_web_enabled': hand_eye_web_enabled,
            }),
        _include(
            'peach_perception', 'scene_perception.launch.py',
            {'autostart': 'false'}),
        _include(
            'peach_perception', 'target_reconstruction.launch.py',
            {'autostart': 'false'}),
        _include(
            'peach_manipulation', 'peach_manipulation.launch.py',
            {'autostart': 'false'}),
        _include('peach_executor', 'observability.launch.py'),
        _include(
            'peach_executor', 'peach_executor.launch.py',
            {
                'autostart': 'false',
                'require_managed_stack': 'true',
            }),
        _include('peach_executor', 'lifecycle_manager.launch.py'),
        OpaqueFunction(function=_maybe_record_mcap),
    ])


def _maybe_record_mcap(context):
    """仅在 record_mcap:=true 时启动 rosbag2 MCAP，默认不录以免占盘."""
    flag = LaunchConfiguration('record_mcap').perform(context).lower()
    if flag not in ('true', '1', 'yes'):
        return []
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output = str(default_runs_root() / f'mcap_{stamp}')
    topics = [
        '/peach_executor/events',
        '/peach_executor/state',
        '/peach_executor/scene_snapshot',
        '/peach/perception/target_observations',
        '/peach/reconstruction/status',
        '/peach/reconstruction/shape_hypothesis',
        '/peach/manipulation/grasp_hypothesis',
    ]
    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--output', output,
                 *topics],
            output='screen',
        ),
    ]
