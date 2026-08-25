# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""启动采摘链路只读 Web 监控."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _launch_node(context):
    """仅在用户显式给出 host/port 时覆盖 YAML；默认自行 configure/activate."""
    parameters = [LaunchConfiguration('params_file')]
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
        package='peach_task_executor',
        executable='peach_observability',
        name='peach_observability',
        namespace='',
        parameters=parameters,
        output='screen',
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
    """生成默认加载 config/observability.yaml 的 launch 描述."""
    share = get_package_share_directory('peach_task_executor')
    config = os.path.join(share, 'config', 'observability.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=config,
            description='只读监控参数文件；默认包内 config/observability.yaml'),
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
    ])
