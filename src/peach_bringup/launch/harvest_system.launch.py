"""
启动桃子采摘完整业务栈（整栈入口；autostart 参数控制是否自动开批）.

本文件只做预检与 Include，不含业务。各包 `yaml_params.py` 自持同文副本，
不合并（包独立 KEEP）。

清洁重写轮阶段 5：生命周期管理换 nav2_lifecycle_manager（bond_timeout 参数
化，默认 0.0 关闭——Python 三托管节点的心跳依赖 ros-jazzy-bondpy，本机未装
时守卫降级；装上后置 bond_timeout:=8.0 即开进程死检；C++ 侧 peach_arm 用
bondcpp 开箱即发；名单/顺序语义同原自研件；bond 关闭期进程死检由 supervisor
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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
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
    camera_frontend = LaunchConfiguration('camera_frontend')
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
            description='启动相机（前端由 camera_frontend 决定）'),
        DeclareLaunchArgument(
            'camera_frontend', default_value='percipio',
            choices=['percipio', 'stereo'],
            description='相机前端：percipio=设备端 18 图案深度（2.43fps，'
                        '额定量程 0.4-0.8m）；stereo=peach_stereo 主机单图案'
                        '立体（~13.5fps，话题同构，09-17 A/B：感知锁定 2.8s '
                        'vs 48s）。两者互斥（相机连接独占）'),
        DeclareLaunchArgument(
            'camera_ip', default_value='169.254.10.110',
            description='相机 IP（peach_stereo 前端使用）'),
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
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true 时全图跟 /clock（bag play --clock 回放）；'
                        '真机必须 false'),
        DeclareLaunchArgument(
            'bond_timeout', default_value='0.0',
            description='nav2_lm 进程死检窗口 [s]；0=关（现行）。开启（建议'
                        ' 8.0）前先 sudo apt install ros-jazzy-bondpy 并重启'
                        '栈——Python 托管节点的心跳由 bondpy 提供，缺失时节点'
                        '守卫降级不起 bond，lm 会误报节点崩溃；peach_arm'
                        '（bondcpp）不受影响'),
        # 须在所有 Node / Include 之前：included launch 里的节点同样吃到
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        # stereo include 必须位于 aubo bringup include 之前：jazzy launch 的
        # IncludeLaunchDescription 会把 launch_arguments 落成全局
        # SetLaunchConfiguration 且不回滚——aubo include 传入的 camera_enabled
        # （stereo 时压成 'false'）会覆盖 CLI 原值，若放在其后本 include 条件
        # 恒假、stereo 相机永不启动（09-17 E2E 实测，最小复现见 testing-log）。
        _include(
            'peach_stereo', 'stereo_camera.launch.py',
            {'device_ip': LaunchConfiguration('camera_ip')},
            condition=IfCondition(PythonExpression(
                ["'", camera_enabled, "' == 'true' and '",
                 camera_frontend, "' == 'stereo'"]))),
        # stereo 前端时压掉 aubo bringup 内的 percipio 相机（该文件只读，
        # 相机独占连接，由本文件改起 peach_stereo；percipio 前端保持原链路）
        _include(
            'aubo_e5_bringup', 'bringup.launch.py', {
                'hardware_mode': hardware_mode,
                'robot_ip': robot_ip,
                'tool_profile': tool_profile,
                'moveit_enabled': moveit_enabled,
                'camera_enabled': PythonExpression(
                    ["'false' if '", camera_frontend,
                     "' == 'stereo' else '", camera_enabled, "'"]),
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
        # 阶段 5：nav2_lifecycle_manager 替自研件（bond_timeout 参数化，
        # 默认 0 关；名单顺序=场景→重建→技能→调度；进程死检=watchdog/bond）
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
                'bond_timeout': ParameterValue(
                    LaunchConfiguration('bond_timeout'), value_type=float),
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
