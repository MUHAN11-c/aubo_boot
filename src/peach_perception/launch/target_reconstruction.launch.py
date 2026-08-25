"""当前目标局部重建：Lifecycle configure → activate 后才积分."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """配置并激活重建节点."""
    share = get_package_share_directory('peach_perception')
    config = os.path.join(share, 'config', 'target_reconstruction.yaml')
    params_file = LaunchConfiguration('params_file')
    node = LifecycleNode(
        package='peach_perception',
        executable='peach_target_reconstruction_node',
        name='peach_target_reconstruction_node',
        namespace='',
        parameters=[params_file],
        output='screen',
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(LaunchConfiguration('autostart')))
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(LaunchConfiguration('autostart')))
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=config,
            description='目标重建参数文件；默认包内 config/target_reconstruction.yaml'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='false 时由 peach_lifecycle_manager 有序 configure/activate'),
        activate,
        node,
        configure,
    ])
