"""启动生命周期任务执行器；不自动发送 RunHarvest."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """配置并激活执行器，等待显式 RunHarvest."""
    params = os.path.join(
        get_package_share_directory('peach_task_executor'),
        'config', 'executor.yaml')
    node = LifecycleNode(
        package='peach_task_executor',
        executable='peach_task_executor',
        name='peach_task_executor',
        namespace='',
        output='screen',
        parameters=[params],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))],
    ))
    return LaunchDescription([activate, node, configure])
