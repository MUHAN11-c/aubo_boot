"""PeachPose 感知节点：Lifecycle configure → activate 后才处理 RGB-D."""
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
    """配置并激活感知节点（与 Nav2 能力端同一套 launch 转换）."""
    share = get_package_share_directory('peach_scene_perception')
    config = os.path.join(share, 'config', 'peach_pose.yaml')
    node = LifecycleNode(
        package='peach_scene_perception',
        executable='peach_scene_perception_node',
        name='peach_scene_perception_node',
        namespace='',
        parameters=[config],
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
            'autostart', default_value='true',
            description='false 时由 peach_lifecycle_manager 有序 configure/activate'),
        activate, node, configure,
    ])
