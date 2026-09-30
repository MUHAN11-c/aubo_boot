"""
Peach v2 whole-stack entry: preflight, camera, aubo_e5_bringup, peach2 lifecycle nodes, manager.

Never sends RunBatch, SetEnables, motion goals or SetIO. Every node starts with enables all
false; a batch is always started by the operator (/peach/task/run_batch).

Every include runs in its own scoped GroupAction: IncludeLaunchDescription otherwise writes its
launch_arguments into the parent context, and a later sibling's DeclareLaunchArgument (e.g.
config_file, autostart, camera_enabled) would then silently keep the leaked value.
"""
from __future__ import annotations

import os

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from peach2_bringup import preflight, stack

MANAGER_NAME = 'peach2_lifecycle_manager'


def _include(package: str, launch_file: str, arguments: dict[str, str]) -> GroupAction:
    return GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file]),
            launch_arguments=list(arguments.items()))])


def _preflight(context: LaunchContext) -> list:
    del context
    domain = os.environ.get('ROS_DOMAIN_ID', '').strip() or preflight.DEFAULT_DOMAIN_ID
    print(f'[peach2 preflight] ROS_DOMAIN_ID={domain}', flush=True)
    stale = preflight.running_stack_processes(domain)
    if stale:
        text = preflight.refusal_message(stale)
        print(f'\n[peach2 preflight] {text}\n', flush=True)
        raise RuntimeError(text)
    return []


def _setup(context: LaunchContext) -> list:
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    hardware_mode = arg('hardware_mode')
    camera_enabled = stack.as_bool(arg('camera_enabled'))
    frontend = arg('camera_frontend')
    tool_id = arg('tool_id')
    moveit_enabled = stack.as_bool(arg('moveit_enabled'))
    bond_timeout = float(arg('bond_timeout'))
    use_sim_time = arg('use_sim_time')
    error = stack.validate(hardware_mode, frontend, tool_id, bond_timeout)
    if error:
        raise RuntimeError(error)
    if hardware_mode == 'real' and stack.as_bool(use_sim_time):
        raise RuntimeError('use_sim_time:=true is not allowed with hardware_mode:=real')

    def text(value: bool) -> str:
        return 'true' if value else 'false'

    actions = []
    # Stereo before the aubo include (historic parameter-leak ordering, kept although every
    # include is now scoped).
    if stack.start_stereo(camera_enabled, frontend):
        actions.append(_include(
            'peach_stereo', 'stereo_camera.launch.py', {'device_ip': arg('camera_ip')}))
    actions.append(_include('aubo_e5_bringup', 'bringup.launch.py', {
        'hardware_mode': hardware_mode,
        'robot_ip': arg('robot_ip'),
        'tool_profile': tool_id,
        'moveit_enabled': text(moveit_enabled),
        'camera_enabled': text(stack.aubo_camera_enabled(camera_enabled, frontend)),
        'extrinsics_enabled': 'true',
        'hand_eye_enabled': 'false',
        'hand_eye_web_enabled': 'false',
    }))
    if camera_enabled:
        actions.append(_include('peach2_perception', 'perception.launch.py', {
            'autostart': 'false', 'use_sim_time': use_sim_time}))
    actions.append(_include('peach2_target_model', 'target_model.launch.py', {
        'autostart': 'false', 'use_sim_time': use_sim_time}))
    if camera_enabled:
        actions.append(_include('peach2_scene', 'scene.launch.py', {
            'autostart': 'false', 'use_sim_time': use_sim_time}))
    actions.append(_include('peach2_manipulation', 'manipulation.launch.py', {
        'hardware_mode': hardware_mode, 'tool_profile': tool_id, 'autostart': 'false'}))
    actions.append(_include('peach2_task', 'peach2_task.launch.py', {
        'hardware_mode': hardware_mode, 'tool_profile': tool_id,
        'runs_dir': arg('runs_dir')}))
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=MANAGER_NAME,
        output='screen',
        parameters=[{
            'node_names': stack.managed_node_names(camera_enabled),
            'autostart': True,
            'bond_timeout': bond_timeout,
        }]))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_mode', default_value='mock', choices=list(stack.HARDWARE_MODES),
            description='mock = mock_components + mock IO; real requires the teach pendant '
                        'powered and operator authorisation'),
        DeclareLaunchArgument(
            'robot_ip', default_value='169.254.10.98',
            description='AUBO controller IP (real only)'),
        DeclareLaunchArgument(
            'camera_enabled', default_value='false',
            description='start the camera, peach2_perception and peach2_scene'),
        DeclareLaunchArgument(
            'camera_frontend', default_value='stereo', choices=list(stack.CAMERA_FRONTENDS),
            description='stereo = peach_stereo host stereo; percipio = device depth via '
                        'aubo_e5_bringup (mutually exclusive camera connection)'),
        DeclareLaunchArgument(
            'camera_ip', default_value='169.254.10.110',
            description='camera IP for the stereo frontend'),
        DeclareLaunchArgument(
            'tool_id', default_value='adaptive_shear_v1', choices=list(stack.TOOL_IDS),
            description='mounted tool (aubo_e5_bringup tool_profile)'),
        DeclareLaunchArgument(
            'moveit_enabled', default_value='true',
            description='start move_group (required by peach2_manipulation)'),
        DeclareLaunchArgument(
            'bond_timeout', default_value='4.0',
            description='nav2 lifecycle_manager bond timeout [s]; 0 disables bond'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='follow /clock (bag replay); refused with hardware_mode:=real'),
        DeclareLaunchArgument(
            'runs_dir', default_value='',
            description='peach2_task ledger root; empty = $PEACH_RUNS_DIR, else <cwd>/runs'),
        OpaqueFunction(function=_preflight),
        # Before every node, including those inside included launch files.
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        OpaqueFunction(function=_setup),
    ])
