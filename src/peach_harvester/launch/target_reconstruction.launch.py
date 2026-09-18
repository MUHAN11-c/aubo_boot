"""当前目标局部重建：Lifecycle configure → activate 后才积分."""

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
from peach_harvester.vision.grasp_standoffs import reconstruction_overlay
from peach_harvester.vision.tool_profiles import reconstruction_tool_params


def generate_launch_description():
    """配置并激活重建节点."""
    config = PathJoinSubstitution([
        FindPackageShare('peach_harvester'),
        'config', 'target_reconstruction.yaml'])
    params_file = LaunchConfiguration('params_file')
    node = LifecycleNode(
        package='peach_harvester',
        executable='peach_target_reconstruction_node',
        name='peach_target_reconstruction_node',
        namespace='',
        parameters=[
            ParameterFile(params_file, allow_substs=True),
            reconstruction_overlay(),
            # 工具档案注入：许可数学内径 + 档案标签随 tool_profile 覆盖基础值
            reconstruction_tool_params(LaunchConfiguration('tool_profile')),
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
            'params_file', default_value=config,
            description='目标重建参数文件；默认包内 config/target_reconstruction.yaml'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='false 时由 peach_lifecycle_manager 有序 configure/activate'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_cylinder_v1',
            choices=['hollow_cylinder_v1', 'adaptive_cylinder_v1'],
            description='末端工具档案（tool.budget.d_inner / tool.profile_id 注入）'),
        activate,
        node,
        configure,
    ])
