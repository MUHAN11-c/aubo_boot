# Copyright 2026 wjz
# SPDX-License-Identifier: BSD-3-Clause
"""
Start the peach2_manipulation lifecycle node.

Default: unconfigured, driven by the stack lifecycle manager (autostart:=false). With
autostart:=true this launch configures and activates the node itself. Neither path sends
a goal, a trajectory or a SetIO: permissions arrive on /peach/enables (default all false).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_params(tool_profile: str) -> list:
    moveit_config = (
        MoveItConfigsBuilder('aubo_e5', package_name='aubo_e5_moveit_config')
        .robot_description(mappings={'tool_profile': tool_profile})
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner'],
            default_planning_pipeline='ompl')
        .trajectory_execution(
            file_path='config/controllers.yaml', moveit_manage_controllers=False)
        .to_moveit_configs())
    planning = dict(moveit_config.joint_limits.get('robot_description_planning', {}))
    if moveit_config.pilz_cartesian_limits:
        planning.update(
            moveit_config.pilz_cartesian_limits.get('robot_description_planning', {}))
    return [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        {'robot_description_planning': planning},
    ]


def _launch_setup(context) -> list:
    tool_profile = LaunchConfiguration('tool_profile').perform(context)
    real = LaunchConfiguration('hardware_mode').perform(context) == 'real'
    params_file = PathJoinSubstitution(
        [FindPackageShare('peach2_manipulation'), 'config', 'manipulation.yaml'])
    node = LifecycleNode(
        package='peach2_manipulation',
        executable='peach2_manipulation_node',
        name='peach2_manipulation',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            *_moveit_params(tool_profile),
            {
                'tool_id': tool_profile,
                'io_backend': 'aubo' if real else 'mock',
                'require_robot_status': real,
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
                transition_id=Transition.TRANSITION_ACTIVATE))]),
        condition=IfCondition(autostart))
    return [activate, node, configure]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_mode', default_value='mock', choices=['mock', 'real'],
            description='mock: simulated tool IO, robot_status not required'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_shear_v1',
            choices=['shear_v1', 'bite_shear_v1', 'adaptive_shear_v1'],
            description='mounted tool (aubo_description/config/<tool_profile>.yaml)'),
        DeclareLaunchArgument(
            'autostart', default_value='false',
            description='true: configure + activate here instead of the lifecycle manager'),
        OpaqueFunction(function=_launch_setup),
    ])
