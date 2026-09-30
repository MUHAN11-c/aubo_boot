"""
peach2_scene lifecycle node.

autostart:=true only drives configure -> activate of this node (it writes PlanningScene world
objects on request and never commands motion or IO); default false leaves it unconfigured for
a lifecycle manager.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    autostart = LaunchConfiguration('autostart')
    node = LifecycleNode(
        package='peach2_scene',
        executable='scene_node',
        name='peach2_scene',
        namespace='',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(node),
                          transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node, goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE))]),
        condition=IfCondition(autostart))
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false',
                              description='configure + activate this node on launch'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('peach2_scene'), 'config', 'scene.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        activate_on_inactive,
        node,
        configure,
    ])
