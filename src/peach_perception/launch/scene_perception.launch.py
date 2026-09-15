"""场景感知节点：Lifecycle configure → activate 后才处理 RGB-D."""

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

from peach_perception.grasp_standoffs import scene_overlay
from peach_perception.tool_profiles import scene_tool_params


def generate_launch_description():
    """配置并激活感知节点（与 Nav2 能力端同一套 launch 转换）."""
    config = PathJoinSubstitution([
        FindPackageShare('peach_perception'),
        'config', 'scene_perception.yaml'])
    node = LifecycleNode(
        package='peach_perception',
        executable='peach_scene_perception_node',
        name='peach_scene_perception_node',
        namespace='',
        parameters=[
            ParameterFile(config, allow_substs=True),
            scene_overlay(),
            # 工具档案注入：径向走廊门内径随 tool_profile 覆盖 yaml 基础值
            scene_tool_params(LaunchConfiguration('tool_profile')),
        ],
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
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_cylinder_v1',
            choices=['hollow_cylinder_v1', 'adaptive_cylinder_v1'],
            description='末端工具档案（tool.D_inner 随档案注入）'),
        activate, node, configure,
    ])
