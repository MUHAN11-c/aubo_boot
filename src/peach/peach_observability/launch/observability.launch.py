"""
只读监控节点 + 可选独立 rosbag2。核心调度不依赖本 launch.

2026-09-29 观测性轮修复：launch 不再发 configure/activate 事件——节点
main() 的 ensure_active() 是唯一转换源（与 vegetation.launch 同款修法；
此前 launch EmitEvent 与 main 自激活存在竞态，对已 active 节点再发转换
会以「Transition is not registered」打死进程——全栈内靠启动时序侥幸
存活，独立起栈必现）。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    if host:
        overrides['host'] = host
    if port:
        overrides['port'] = port
    if startup_facts:
        overrides['startup_facts'] = startup_facts
    if overrides:
        parameters.append(overrides)
    node = LifecycleNode(
        package='peach_observability',
        executable='peach_observability',
        name='peach_observability',
        namespace='',
        parameters=parameters,
        output='screen',
        # G4（2026-09-20 审查）：launch 默认停栈窗 5+5s 会杀掉 bag 报告线程
        # （join_report 上限 55s）——给本进程放宽 SIGTERM 窗，写盘为原子
        # （tmp+rename），超窗最坏留「无新报告」（CLI 可复跑）而非半份。
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
            'record_bag', default_value='false',
            description='true 时另起独立 ros2 bag record 进程'),
        DeclareLaunchArgument(
            'params_file', default_value=config,
            description='只读监控参数文件；默认 peach_observability/config/observability.yaml'),
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
            'autostart', default_value='true',
            description='兼容保留参数：节点 main() 恒自激活（ensure_active），'
                        'launch 不发转换事件（2026-09-29 互杀修复，'
                        '见文件头）'),
        OpaqueFunction(function=_launch_node),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('peach_observability'),
                    'launch', 'record_bag.launch.py'])),
            condition=IfCondition(LaunchConfiguration('record_bag'))),
    ])
