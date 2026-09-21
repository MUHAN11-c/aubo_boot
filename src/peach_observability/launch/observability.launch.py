"""只读监控节点 + 可选独立 rosbag2。核心调度不依赖本 launch."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def _launch_node(context):
    """仅在用户显式给出 host/port 时覆盖 YAML；默认自行 configure/activate."""
    parameters = [ParameterFile(
        LaunchConfiguration('params_file'), allow_substs=True)]
    overrides = {}
    host = LaunchConfiguration('host').perform(context)
    port = int(LaunchConfiguration('port').perform(context))
    if host:
        overrides['host'] = host
    if port:
        overrides['port'] = port
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
    autostart = LaunchConfiguration('autostart')
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(autostart))
    return [activate, node, configure]


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
            'autostart', default_value='true',
            description='false 时保持 Unconfigured，须外部 configure/activate'),
        OpaqueFunction(function=_launch_node),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('peach_observability'),
                    'launch', 'record_bag.launch.py'])),
            condition=IfCondition(LaunchConfiguration('record_bag'))),
    ])
