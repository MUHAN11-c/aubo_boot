"""独立起 GPU 枝/叶分割节点；不进 harvest_system / lifecycle 名单."""

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
    """配置并可选激活 peach_vegetation；输入图走 remap 而非话题名参数."""
    config = PathJoinSubstitution([
        FindPackageShare('peach_vegetation'),
        'config', 'vegetation.yaml'])
    node = LifecycleNode(
        package='peach_vegetation',
        executable='peach_vegetation',
        name='peach_vegetation',
        namespace='',
        parameters=[ParameterFile(config, allow_substs=True)],
        remappings=[('image', LaunchConfiguration('image'))],
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
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(autostart))
    return LaunchDescription([
        DeclareLaunchArgument(
            'image', default_value='/camera/color/image_raw',
            description='彩色图话题（remap 到节点相对名 image）'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='false 时保持 Unconfigured，须外部 configure/activate'),
        activate, node, configure,
    ])
