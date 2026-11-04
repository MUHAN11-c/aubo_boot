"""Harvest supervisor lifecycle node; does not auto-start a batch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from peach_common.tool_profiles import tool_profile_id_params


def _setup(context):
    """Build node after launch substitutions are available."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    parameters = [
        ParameterFile(value('params_file'), allow_substs=True),
        tool_profile_id_params(LaunchConfiguration('tool_profile')),
    ]
    for name in ('params_overlay', 'task_params_file'):
        path = value(name).strip()
        if path:
            parameters.append(ParameterFile(path, allow_substs=True))
    # Explicit launch policy is authoritative; task YAML must not flip it.
    parameters.append({
        'require_managed_stack': value('require_managed_stack').lower()
        in ('1', 'true', 'yes'),
        'skip_reconstruction': value('skip_reconstruction').lower()
        in ('1', 'true', 'yes'),
        'use_sim_time': value('use_sim_time').lower() in ('1', 'true', 'yes'),
    })

    node = LifecycleNode(
        package='peach_supervisor',
        executable='peach_supervisor',
        name='peach_supervisor',
        namespace=value('namespace'),
        parameters=parameters,
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
    return [activate, node, configure]


def generate_launch_description():
    """Declare standard launch arguments."""
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([
                FindPackageShare('peach_supervisor'),
                'config', 'peach_supervisor.yaml'])),
        DeclareLaunchArgument('params_overlay', default_value=''),
        DeclareLaunchArgument('task_params_file', default_value=''),
        DeclareLaunchArgument('require_managed_stack', default_value='false'),
        DeclareLaunchArgument('skip_reconstruction', default_value='false'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_shear_v1',
            choices=['shear_v1', 'bite_shear_v1', 'adaptive_shear_v1']),
        OpaqueFunction(function=_setup),
    ])
