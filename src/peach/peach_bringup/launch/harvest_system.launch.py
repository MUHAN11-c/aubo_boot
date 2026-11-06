"""
启动桃子采摘完整业务栈（整栈入口；autostart 参数控制是否自动开批）.

本文件负责预检、参数装配、Include 与应用辅助节点，不含采摘业务。Python 参数使用
GPL schema；各节点部署 YAML 留在所属能力包。

nav2_lifecycle_manager 管理能力节点，bond_timeout 默认 0.0 关闭进程死检。
lifecycle_flag_bridge 把 is_active 桥接到闩锁话题
/peach/lifecycle/managed_nodes_activated。autostart 默认关；启用时客户端
等待栈就绪与自检通过后发 RunHarvest，运动/工具仍需独立使能与操作员授权。

观测性轮（P0）：展开期采集启动事实（配置 + git 版本 + 域 ID）注入
peach_observability——落 runs/session_*/startup.json、驱动启动自检
（软门 + autostart 硬等 selfcheck_passed）。
"""

import datetime
import json
import math
import os
import shlex

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from peach_bringup.preflight import check_disk_free, running_stack_pids

import yaml


def _include(package, launch_file, launch_arguments=None, condition=None):
    """隔离子 launch 参数，同时继承父级时钟和启动事实."""
    kwargs = {}
    if condition is not None:
        kwargs['condition'] = condition
    return GroupAction(scoped=True, forwarding=True, actions=[
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare(package), 'launch', launch_file]),
            launch_arguments=(launch_arguments or {}).items(),
            **kwargs,
        ),
    ])


def _preflight(context):
    """Launch 展开期前置检查：旧实例拒启 + 磁盘余量 + 域 ID 提示."""
    del context
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    print(f'[preflight] ROS_DOMAIN_ID={domain or "(默认0)"}', flush=True)
    disk_problem = check_disk_free(10.0)
    if disk_problem:
        print(f'\n[preflight] {disk_problem}\n', flush=True)
        raise RuntimeError(disk_problem)
    stale = running_stack_pids()
    if stale:
        lines = ['检测到旧实例仍在运行，拒绝重复启动；先清理：']
        lines += [f'  kill -9 {pid}  # {cmd}' for pid, cmd in stale[:12]]
        text = '\n'.join(lines)
        print(f'\n[preflight] {text}\n', flush=True)
        raise RuntimeError(text)
    return []


def _joint_state_broadcaster_parameters(context):
    """Keep broadcaster parameters inside the driver's launch scope."""
    # Jazzy 的 scope 浅拷贝配置；SetParametersFromFile 会原地追加列表。
    context.launch_configurations['global_params'] = list(
        context.launch_configurations.get('global_params', []))
    return [SetParametersFromFile(PathJoinSubstitution([
        FindPackageShare('peach_bringup'), 'config',
        'joint_state_broadcasters.yaml']))]


def _move_group_shutdown_compat(context):
    """仅为 move_group 保留默认 capability 库至进程退出（Jazzy #3680）."""
    if (LaunchConfiguration('move_group_shutdown_workaround').perform(context)
            != 'true'
            or LaunchConfiguration('moveit_enabled').perform(context) != 'true'):
        return []
    if context.launch_configurations.get('launch-prefix', ''):
        raise RuntimeError(
            'move_group shutdown compatibility needs its own launch-prefix; '
            'set move_group_shutdown_workaround:=false to use another prefix')
    library = os.path.join(
        get_package_prefix('moveit_ros_move_group'), 'lib',
        'libmoveit_move_group_default_capabilities.so')
    if not os.path.isfile(library):
        raise RuntimeError(f'MoveIt default capability library missing: {library}')
    previous = context.environment.get('LD_PRELOAD', '')
    preload = library + (':' + previous if previous else '')
    return [
        SetLaunchConfiguration('launch-prefix', shlex.join([
            'env', f'LD_PRELOAD={preload}'])),
        SetLaunchConfiguration('launch-prefix-filter', '^move_group$'),
    ]


def _contains_key(value, key):
    """Recursively detect a forbidden policy key in a task parameter file."""
    if isinstance(value, dict):
        return key in value or any(_contains_key(v, key) for v in value.values())
    if isinstance(value, list):
        return any(_contains_key(v, key) for v in value)
    return False


def _resolve_reconstruction_control(context):
    """Composition-level switch: absent module implies skip + unrefined."""
    enabled = LaunchConfiguration(
        'reconstruction_enabled').perform(context).lower() in (
            '1', 'true', 'yes')
    skip = LaunchConfiguration(
        'skip_reconstruction').perform(context).lower() in (
            '1', 'true', 'yes')
    task_path = LaunchConfiguration('task_params_file').perform(context).strip()
    if task_path:
        with open(task_path, encoding='utf-8') as stream:
            task = yaml.safe_load(stream) or {}
        forbidden = ('skip_reconstruction', 'allow_unrefined_geometry')
        found = [key for key in forbidden if _contains_key(task, key)]
        if found:
            raise RuntimeError(
                f'task_params_file 不得包含 {found}；这些是 launch/ops policy，'
                '请用 reconstruction_enabled/skip_reconstruction 显式设置')
    return [SetLaunchConfiguration(
        'effective_skip_reconstruction', 'true' if (not enabled or skip) else 'false')]


def _lifecycle_manager(context):
    """Build lifecycle list without an absent reconstruction node."""
    enabled = LaunchConfiguration(
        'reconstruction_enabled').perform(context).lower() in (
            '1', 'true', 'yes')
    node_names = ['peach_scene_perception_node']
    if enabled:
        node_names.append('peach_target_reconstruction_node')
    node_names += ['peach_arm', 'peach_supervisor']
    return [Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='peach_lifecycle_manager',
        output='screen',
        parameters=[{
            'node_names': node_names,
            'autostart': True,
            'bond_timeout': ParameterValue(
                LaunchConfiguration('bond_timeout'), value_type=float),
        }])]


def _web_bridge():
    """Expose the console's native ROS endpoints on one loopback WebSocket."""
    # The application supplies all ROS types; the SDK needs neither rosapi
    # graph discovery nor the dependency's demo and standalone HTTP server.
    parameter_nodes = (
        'peach_supervisor', 'peach_arm', 'peach_scene_perception_node',
        'peach_target_reconstruction_node', 'peach_observability')
    services = [
        f'/{node}/{op}_parameters' for node in parameter_nodes
        for op in ('list', 'get', 'set')]
    services.extend((
        '/peach_supervisor/control',
        '/peach_supervisor/set_enables',
        '/peach_scene_perception_node/begin_scene',
        '/peach_arm/check_reachability',
        '/peach_arm/go_to_photo_pose',
        '/peach_arm/preview_approach_insert',
        '/peach_arm/preview_full_contact',
        '/peach_arm/acknowledge_recovery',
        '/peach_arm/set_execution_armed',
        '/peach_arm/cancel_cycle',
        '/peach_target_reconstruction_node/save_session',
        '/peach_target_reconstruction_node/reset_reconstruction',
        '/peach_target_reconstruction_node/finalize_reconstruction',
        '/peach_target_reconstruction_node/query_reconstruction_state',
        '/peach_lifecycle_manager/manage_nodes',
    ))
    return Node(
        package='ros2_web_link', executable='bridge_node.py',
        name='peach_rosbridge', output='screen',
        condition=IfCondition(LaunchConfiguration('rosbridge_enabled')),
        parameters=[{
            'address': '127.0.0.1',
            'port': ParameterValue(LaunchConfiguration('rosbridge_port'), value_type=int),
            # rosbridge 2.7.1 parses these as strings with single-quoted names.
            'topics_pub_glob': '[]',
            'topics_sub_glob': "['/peach_observability/web_state', "
                               "'/peach_observability/web_force', "
                               "'/peach_observability/web_trajectory']",
            'services_glob': str(services),
            'actions_glob': "['/peach_supervisor/run_harvest', "
                            "'/peach_arm/survey_scene', '/peach_arm/execute_target', "
                            "'/peach_target_reconstruction_node/build_target_model']",
            'call_services_in_new_thread': True,
            'send_action_goals_in_new_thread': True,
            'default_call_service_timeout': 10.0,
            'write_queue_size': 64,
        }],
        sigterm_timeout='15')


def _workspace_root() -> str:
    """本文件向上找含 .git 的目录（源树 4 级；install 树找不到给空串）."""
    directory = os.path.abspath(__file__)
    for _ in range(6):
        directory = os.path.dirname(directory)
        if os.path.isdir(os.path.join(directory, '.git')):
            return directory
    return ''


def _git_facts() -> dict:
    """Git 身份经纯文件读取（无子进程）：HEAD ref + commit sha."""
    root = _workspace_root()
    if not root:
        return {'root': 'unknown', 'branch': 'unknown', 'commit': 'unknown'}
    head_path = os.path.join(root, '.git', 'HEAD')
    try:
        with open(head_path, encoding='utf-8') as stream:
            head = stream.read().strip()
    except OSError:
        head = ''
    if head.startswith('ref: '):
        ref = head[5:]
        branch = ref.removeprefix('refs/heads/')
        commit = 'unknown'
        try:
            with open(os.path.join(root, '.git', ref),
                      encoding='utf-8') as stream:
                commit = stream.read().strip()[:12]
        except OSError:
            try:
                with open(os.path.join(root, '.git', 'packed-refs'),
                          encoding='utf-8') as stream:
                    for line in stream:
                        fields = line.split()
                        if len(fields) == 2 and fields[1] == ref:
                            commit = fields[0][:12]
                            break
            except OSError:
                pass
    else:  # detached HEAD：内容即 commit sha
        branch = '(detached)'
        commit = head[:12]
    return {'root': root, 'branch': branch, 'commit': commit}


def _expected_tcp_norm_m(tool_profile: str) -> float:
    """从 aubo_description 工具档案算 |wrist3_Link→tcp|（文件读，不跨包 import）."""
    try:
        archive = os.path.join(
            get_package_share_directory('aubo_description'), 'config',
            f'{tool_profile}.yaml')
        with open(archive, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        xyz = data['frames']['tool_axis']['xyz']
        return float(math.sqrt(sum(float(v) ** 2 for v in xyz)))
    except (OSError, KeyError, TypeError, ValueError):
        return -1.0


def _collect_startup_facts(context):
    """启动事实采集（展开期一次）：配置 + git + 域 ID → observability 注入."""
    cfg = context.launch_configurations
    hardware_mode = cfg.get('hardware_mode', 'mock')
    facts = {
        'launched_at': datetime.datetime.now().isoformat(timespec='seconds'),
        'hardware_mode': hardware_mode,
        'robot_ip': cfg.get('robot_ip', ''),
        'tool_profile': cfg.get('tool_profile', ''),
        'moveit_enabled': cfg.get('moveit_enabled', 'true'),
        'rosbridge_enabled': cfg.get('rosbridge_enabled', 'true'),
        'rosbridge_port': cfg.get('rosbridge_port', '9090'),
        'move_group_shutdown_workaround': cfg.get(
            'move_group_shutdown_workaround', 'true'),
        'camera_enabled': cfg.get('camera_enabled', 'false'),
        'camera_frontend': cfg.get('camera_frontend', 'stereo'),
        'camera_ip': cfg.get('camera_ip', ''),
        'extrinsics_enabled': cfg.get('extrinsics_enabled', 'true'),
        'force_enabled': cfg.get('force_enabled', 'true'),
        'bond_timeout': cfg.get('bond_timeout', '0.0'),
        'use_sim_time': cfg.get('use_sim_time', 'false'),
        'reconstruction_enabled': cfg.get('reconstruction_enabled', 'true'),
        # Vegetation has an independent entry point and is not deployed here.
        'vegetation_enabled': False,
        'skip_reconstruction': cfg.get(
            'effective_skip_reconstruction',
            cfg.get('skip_reconstruction', 'false')),
        'task_params_file': cfg.get('task_params_file', ''),
        'autostart': cfg.get('autostart', 'false'),
        'require_robot_status':
            'false' if hardware_mode == 'mock' else 'true',
        'expected_tcp_norm_m': _expected_tcp_norm_m(
            cfg.get('tool_profile', '')),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
        'git': _git_facts(),
    }
    context.launch_configurations['peach_startup_facts'] = json.dumps(
        facts, ensure_ascii=False)
    frontend = facts['camera_frontend']
    enabled = facts['camera_enabled']
    print(
        f'[startup] camera_enabled={enabled} camera_frontend={frontend}',
        flush=True)
    if enabled == 'true' and frontend == 'percipio':
        print(
            '[startup] 官方 percipio 前端：不起 peach_stereo；'
            '感知装入 percipio_frontend.yaml（max_frame_age_s=2.5）',
            flush=True)
    elif enabled == 'true' and frontend == 'stereo':
        print(
            '[startup] peach_stereo 前端：aubo bringup 内 percipio 已压掉',
            flush=True)
    return []


def _fixed_camera_mount(context):
    """Publish base_link→camera_link when the camera is on and wrist extrinsics are off."""
    if (LaunchConfiguration('camera_enabled').perform(context) != 'true' or
            LaunchConfiguration('extrinsics_enabled').perform(context) == 'true'):
        return []
    path = os.path.join(
        get_package_share_directory('peach_bringup'),
        'config', 'fixed_camera_mount.yaml')
    with open(path, encoding='utf-8') as stream:
        pose = yaml.safe_load(stream) or {}
    xyz = pose.get('xyz_m')
    quat = pose.get('quaternion_xyzw')
    parent = pose.get('parent_frame')
    child = pose.get('child_frame')
    if (not isinstance(xyz, list) or len(xyz) != 3 or
            not isinstance(quat, list) or len(quat) != 4 or
            not parent or not child):
        raise RuntimeError(
            f'fixed camera mount {path} needs parent_frame, child_frame, '
            'xyz_m[3] and quaternion_xyzw[4]')
    return [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='fixed_camera_mount_publisher', output='screen',
        arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--qx', str(quat[0]), '--qy', str(quat[1]),
            '--qz', str(quat[2]), '--qw', str(quat[3]),
            '--frame-id', str(parent), '--child-frame-id', str(child)])]


def _application_rviz(context):
    """Load the application view with the same tool model as move_group."""
    if LaunchConfiguration('rviz').perform(context).lower() != 'true':
        return []
    profile = LaunchConfiguration('tool_profile').perform(context)
    mode = LaunchConfiguration('hardware_mode').perform(context)
    model = (MoveItConfigsBuilder('aubo_e5', package_name='aubo_e5_moveit_config')
             .robot_description(mappings={'tool_profile': profile})
             .trajectory_execution(file_path='config/' + (
                 'controllers_mock.yaml' if mode == 'mock' else 'controllers.yaml'),
                 moveit_manage_controllers=False)
             .to_moveit_configs())
    return [Node(
        package='rviz2', executable='rviz2', name='peach_rviz', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[model.to_dict()])]


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
            'tool_profile', default_value='bite_shear_v1',
            choices=['shear_v1', 'bite_shear_v1', 'adaptive_shear_v1'],
            description='末端工具档案；默认 bite_shear_v1（2026-10-13 操作员裁定）'),
        DeclareLaunchArgument(
            'mock_initial_positions',
            default_value='0.425083,0.195177,1.677740,1.461739,-0.500161,0.038621',
            description='透传 aubo bringup 的 mock 六关节初值（rad，逗号串）。'
                        '缺省=harvest_stow，与不传该参时的 xacro 字面量相同；'
                        '冻结仿真传入 SRDF global_photo_pose'),
        DeclareLaunchArgument(
            'autostart', default_value='false',
            description='true 时托管栈与自检就绪后自动发 RunHarvest；'
                        '默认关，运动/工具仍须独立使能和操作员授权'),
        DeclareLaunchArgument(
            'moveit_enabled', default_value='true',
            description='启动 MoveIt move_group；RViz2 由 rviz 单独开关'),
        DeclareLaunchArgument(
            'move_group_shutdown_workaround', default_value='true',
            choices=['true', 'false'],
            description='仅为 move_group 预加载默认 capability 库，规避本机 '
                        'Jazzy MoveIt 退出析构崩溃；上游修复后可关闭'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='由应用入口启动 rviz2，驱动 include 内的 RViz 已关闭；'
                        '无头/CI 关窗口，move_group 仍在'),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([
                FindPackageShare('peach_bringup'), 'config', 'harvest_app.rviz']),
            description='RViz 配置；默认应用视图 harvest_app.rviz；需要 MoveIt 规划面板用 harvest.rviz'),
        DeclareLaunchArgument(
            'camera_enabled', default_value='false',
            description='启动相机（前端由 camera_frontend 决定）'),
        DeclareLaunchArgument(
            'camera_frontend', default_value='stereo',
            choices=['percipio', 'stereo', 'replay'],
            description='相机前端（launch 期互斥开关，禁止运行期热切）：'
                        'stereo=peach_stereo 主机单图案立体（~13 fps）；'
                        'percipio=官方设备 18 图案深度（~1 fps，轮廓清晰；'
                        '感知装入 percipio_frontend.yaml 年龄门 2.5 s；候选保持当帧几何）；'
                        'replay=不起驱动，冻结 bag 重发 /camera/*。'
                        '同一相机连接独占；预检拒绝残留 stereo/percipio 进程'),
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
            'force_enabled', default_value='true',
            description='启动 USB 串口 5 点力传感器；不进 lifecycle'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true 时全图跟 /clock（bag play --clock 回放）；'
                        '真机必须 false'),
        DeclareLaunchArgument(
            'bond_timeout', default_value='0.0',
            description='nav2_lm 进程死检窗口 [s]；0=关（现行）。bondpy 守卫 import '
                        '已修（2026-10-08 勘定：apt 包在装但顶层 __init__ 0 字节，'
                        '须子模块 from bondpy.bondpy import Bond）；包在装时 Python '
                        '托管节点心跳即生效，真缺包时守卫降级不起 bond，lm 会误报'
                        '节点崩溃；peach_arm（bondcpp）不受影响'),
        DeclareLaunchArgument(
            'reconstruction_enabled', default_value='true',
            description='是否启动 target_reconstruction 进程并纳入 lifecycle；'
                        'false 时 supervisor 强制 skip、arm 允许未精化几何'),
        DeclareLaunchArgument(
            'skip_reconstruction', default_value=PythonExpression([
                "'false' if '", hardware_mode, "' == 'real' else 'true'"]),
            description='本批是否跳过 Build/补视（supervisor 作业策略）；'
                        'mock 默认跳过、real 默认完整重建；显式传参优先；'
                        '不控制 reconstruction 进程是否启动'),
        DeclareLaunchArgument(
            'task_params_file', default_value='',
            description='可选标准 ROS 节点参数 YAML；mock/real 共用同一作业配置'),
        DeclareLaunchArgument(
            'brain_params_overlay', default_value='',
            description='透传能力包 params_overlay（按节点名分节）；空=不装。'
                        '冻结相机系统测用来把 reconstruction min_views 置 1'),
        DeclareLaunchArgument(
            'arm_params_overlay', default_value='',
            description='透传 peach_arm params_overlay；空=不装。'
                        '调试时显式使用；业务配置优先用 task_params_file，两种硬件同值'),
        DeclareLaunchArgument(
            'observability_port', default_value='0',
            description='透传 peach_observability HTTP 端口；0=用 YAML 8090。'
                        '他人占用 8090 时测试入口可改 8091，不清理其他进程'),
        DeclareLaunchArgument('rosbridge_enabled', default_value='true'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        # 须在所有 Node / Include 之前：included launch 里的节点同样吃到
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        # Must run before any include: policy conflicts fail closed and the
        # effective skip is then captured by startup facts.
        OpaqueFunction(function=_resolve_reconstruction_control),
        # 启动事实采集（须在全部 DeclareLaunchArgument 之后；结果进
        # peach_startup_facts 配置，供 observability include 读取）
        OpaqueFunction(function=_collect_startup_facts),
        _web_bridge(),
        # stereo 与驱动内的 percipio 互斥；驱动参数覆盖由下方 scoped GroupAction 隔离。
        _include(
            'peach_stereo', 'stereo_camera.launch.py',
            {'device_ip': LaunchConfiguration('camera_ip')},
            condition=IfCondition(PythonExpression(
                ["'", camera_enabled, "' == 'true' and '",
                 camera_frontend, "' == 'stereo'"]))),
        # 场景障碍快照节点（2026-09-29 避障只为保护相机）：Survey 完成触发后
        # 启用建图后将最新点云转 PlanningScene 碰撞对象；部署 enabled 默认 false。
        OpaqueFunction(function=_fixed_camera_mount),
        # 作业期冻结。两前端话题/帧同构，percipio/stereo 通用。
        # 不进 lifecycle 名单（无状态快照服务）；mock camera_enabled:=false
        # 不起，零副作用。
        _include(
            'peach_scene_obstacles', 'scene_obstacles.launch.py', {},
            condition=IfCondition(camera_enabled)),
        # stereo 前端时压掉 aubo bringup 内的 percipio 相机（该文件只读，
        # 相机独占连接，由本文件改起 peach_stereo；percipio 前端保持原链路）
        GroupAction(scoped=True, actions=[
            OpaqueFunction(function=_move_group_shutdown_compat),
            OpaqueFunction(function=_joint_state_broadcaster_parameters),
            _include(
                'aubo_e5_bringup', 'bringup.launch.py', {
                    'hardware_mode': hardware_mode,
                    'robot_ip': robot_ip,
                    'tool_profile': tool_profile,
                    'mock_initial_positions': LaunchConfiguration(
                        'mock_initial_positions'),
                    'moveit_enabled': moveit_enabled,
                    'rviz': 'false',
                    'camera_enabled': PythonExpression(
                        ["'true' if '", camera_frontend,
                         "' == 'percipio' and '", camera_enabled,
                         "' == 'true' else 'false'"]),
                    'extrinsics_enabled': extrinsics_enabled,
                    'hand_eye_enabled': hand_eye_enabled,
                    'hand_eye_web_enabled': hand_eye_web_enabled,
                }),
            # 固定六轴发布序与完整硬件状态分别交给官方只读广播器。
            # 只重映射额外实例，避免 /joint_states 双发或改动消费者接线。
            Node(
                package='controller_manager', executable='spawner',
                arguments=[
                    'dynamic_joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '60',
                    '--controller-ros-args',
                    '--remap /joint_states:='
                    '/dynamic_joint_state_broadcaster/joint_states'],
                output='screen')]),
        OpaqueFunction(function=_application_rviz),
        _include(
            'serial_force', 'serial_force.launch.py', {},
            condition=IfCondition(LaunchConfiguration('force_enabled'))),
        _include('peach_scene_perception', 'scene_perception.launch.py', {
            'autostart': 'false',
            'tool_profile': tool_profile,
            'params_overlay': LaunchConfiguration('brain_params_overlay'),
            'task_params_file': LaunchConfiguration('task_params_file'),
            'camera_frontend': camera_frontend,
        }),
        _include(
            'peach_target_reconstruction', 'target_reconstruction.launch.py', {
                'autostart': 'false',
                'tool_profile': tool_profile,
                'capture_auto_mode': PythonExpression([
                    "'false' if '", LaunchConfiguration('effective_skip_reconstruction'),
                    "' == 'true' else 'true'"]),
                'params_overlay': LaunchConfiguration('brain_params_overlay'),
                'task_params_file': LaunchConfiguration('task_params_file'),
            },
            condition=IfCondition(
                LaunchConfiguration('reconstruction_enabled'))),
        _include('peach_supervisor', 'supervisor.launch.py', {
            'autostart': 'false',
            'require_managed_stack': 'true',
            'tool_profile': tool_profile,
            'skip_reconstruction': LaunchConfiguration('effective_skip_reconstruction'),
            'params_overlay': LaunchConfiguration('brain_params_overlay'),
            'task_params_file': LaunchConfiguration('task_params_file'),
        }),
        _include(
            'peach_arm', 'peach_arm.launch.py',
            {
                'autostart': 'false',
                'tool_profile': tool_profile,
                # mock 不起 aubo_io_controller（bringup 仅 real 拉起）；
                # 仍要求 robot_status 则 Survey goToPhotoPose 入口
                # robot_status_missing → 立即 survey_failed。
                'require_robot_status': PythonExpression(
                    ["'false' if '", hardware_mode, "' == 'mock' else 'true'"]),
                'allow_unrefined_geometry': LaunchConfiguration(
                    'effective_skip_reconstruction'),
                'params_overlay': LaunchConfiguration(
                    'arm_params_overlay'),
                'task_params_file': LaunchConfiguration('task_params_file'),
            }),
        _include('peach_observability', 'observability.launch.py', {
            # 启动事实注入（startup.json 工件 + 自检 facts 单通道推导）
            'startup_facts': LaunchConfiguration('peach_startup_facts'),
            'force_expected': LaunchConfiguration('force_enabled'),
            'port': LaunchConfiguration('observability_port'),
            'task_params_file': LaunchConfiguration('task_params_file'),
        }),
        # 阶段 5：nav2_lifecycle_manager 名单随 reconstruction_enabled 组合
        OpaqueFunction(function=_lifecycle_manager),
        # is_active → 闩锁 managed_nodes_activated 桥（消费方零改动）
        Node(
            package='peach_bringup',
            executable='peach_lifecycle_flag_bridge',
            name='peach_lifecycle_flag_bridge',
            output='screen'),
        # mock 初始位姿：xacro state_interface initial_value（harvest_stow，
        # 不是 SRDF global_photo_pose；第一次 Survey 才 PTP 到拍照位）
        # 官方 ros2_control 机制，非轨迹——详见 aubo_e5.ros2_control.xacro
        # autostart 客户端默认关；启用只发批，不代替运动/工具使能与授权。
        Node(
            package='peach_bringup',
            executable='peach_autostart_client',
            name='peach_autostart_client',
            output='screen',
            condition=IfCondition(autostart)),
    ])
