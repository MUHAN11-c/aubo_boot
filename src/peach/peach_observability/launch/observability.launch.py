"""
观测节点、HTTP 调试桥与可选会话 rosbag2 写线程。核心调度不依赖本 launch.

2026-09-29 观测性轮修复：launch 不再发 configure/activate 事件——节点
main() 的 ensure_active() 是唯一转换源（与 vegetation.launch 同款修法；
此前 launch EmitEvent 与 main 自激活存在竞态，对已 active 节点再发转换
会以「Transition is not registered」打死进程——全栈内靠启动时序侥幸
存活，独立起栈必现）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def _launch_node(context):
    """仅在用户显式给出 host/port/startup_facts 时覆盖 YAML；节点 main 自激活."""
    parameters = [ParameterFile(
        LaunchConfiguration('params_file'), allow_substs=True)]
    overrides = {}
    host = LaunchConfiguration('host').perform(context)
    port = int(LaunchConfiguration('port').perform(context))
    startup_facts = LaunchConfiguration('startup_facts').perform(context)
    force_expected = LaunchConfiguration('force_expected').perform(context)
    if host:
        overrides['host'] = host
    if port:
        overrides['port'] = port
    if startup_facts:
        overrides['startup_facts'] = startup_facts
    if force_expected:
        overrides['selfcheck.force_expected'] = (
            force_expected.lower() in ('1', 'true', 'yes'))
    if overrides:
        parameters.append(overrides)
    task_params = LaunchConfiguration('task_params_file').perform(context).strip()
    if task_params:
        parameters.append(ParameterFile(task_params, allow_substs=True))
    node = LifecycleNode(
        package='peach_observability',
        executable='peach_observability',
        name='peach_observability',
        namespace='',
        parameters=parameters,
        output='screen',
        # 给节点排空写队列、封口 bag 和回收自动报告子进程留出退出窗口。
        # join_report 默认等 30s，再依次 TERM/KILL 各等 2s；超时保留 bag，
        # 报告可离线重算，输出使用 tmp+rename 防止交付半份文件。
        # 须为字符串 substitution：浮点会让 launch 展开报
        # 'float' object is not iterable（09-21 E2E 实测修复）
        sigterm_timeout='60',
    )
    return [node]


def generate_launch_description():
    """生成监控节点 launch；yaml 在本包 config/（W11 起随包走）."""
    config = PathJoinSubstitution([
        FindPackageShare('peach_observability'),
        'config', 'observability.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=config,
            description='只读监控参数文件；默认 peach_observability/config/observability.yaml'),
        DeclareLaunchArgument(
            'task_params_file', default_value='',
            description='与 brain/arm 同一份标准 ROS 作业参数 YAML'),
        DeclareLaunchArgument(
            'host', default_value='',
            description='覆盖监听地址；空串使用 YAML'),
        DeclareLaunchArgument(
            'port', default_value='0',
            description='覆盖监听端口；0 使用 YAML'),
        DeclareLaunchArgument(
            'startup_facts', default_value='',
            description='harvest_system 注入的启动事实 JSON（自检/工件用）；'
                        '空=独立起栈'),
        DeclareLaunchArgument(
            'force_expected', default_value='',
            description='覆盖自检 force_topic 期望；空=用 YAML（产品默认 true）'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='兼容保留参数：节点 main() 恒自激活（ensure_active），'
                        'launch 不发转换事件（2026-09-29 互杀修复，'
                        '见文件头）'),
        OpaqueFunction(function=_launch_node),
    ])
