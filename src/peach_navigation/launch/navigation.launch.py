"""作业位导航适配：Lifecycle configure → activate 后才接 NavigateToWorksite."""

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
    """配置并激活导航适配节点."""
    config = PathJoinSubstitution([
        FindPackageShare('peach_navigation'),
        'config', 'navigation.yaml'])
    node = LifecycleNode(
        package='peach_navigation',
        executable='peach_navigation_node',
        name='peach_navigation_node',
        namespace='',
        parameters=[ParameterFile(
            LaunchConfiguration('navigation_params_file'), allow_substs=True)],
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
            'navigation_params_file', default_value=config,
            description='导航参数文件；默认包内 config/navigation.yaml'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='false 时由 peach_lifecycle_manager 有序 configure/activate'),
        activate,
        node,
        configure,
    ])
