"""启动生命周期任务执行器；不自动发送 RunHarvest."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """配置并激活执行器，等待显式 RunHarvest."""
    params = PathJoinSubstitution([
        FindPackageShare('peach_executor'),
        'config', 'peach_executor.yaml'])
    node = LifecycleNode(
        package='peach_executor',
        executable='peach_executor',
        name='peach_executor',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(params, allow_substs=True),
            {
                'require_managed_stack': LaunchConfiguration(
                    'require_managed_stack'),
            },
        ],
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='false 时由 peach_lifecycle_manager 有序 configure/activate'),
        DeclareLaunchArgument(
            'require_managed_stack', default_value='false',
            description='true 时须等生命周期管理器就绪旗标才接受 RunHarvest'),
        activate, node, configure,
    ])
