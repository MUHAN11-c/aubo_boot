# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Compose a browser, upstream ROS bridge and optional hardware-free demo."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _stop_group(event, context):
    """Do not leave a partly running stack after any required process exits."""
    if context.is_shutdown:
        return []
    reason = f'{event.process_name} exited ({event.returncode})'
    return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]


def generate_launch_description():
    """Bind localhost and keep all command allowlists narrow by default."""
    arguments = [
        DeclareLaunchArgument('address', default_value='127.0.0.1'),
        DeclareLaunchArgument('ws_port', default_value='9090'),
        DeclareLaunchArgument('web_port', default_value='8088'),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('demo', default_value='false'),
        DeclareLaunchArgument('profile', default_value='robot', choices=['demo', 'robot']),
        DeclareLaunchArgument('config', default_value='',
                              description='Optional profiles YAML with local view overrides'),
        DeclareLaunchArgument('project', default_value='',
                              description='Selected project contract; empty follows profile'),
        DeclareLaunchArgument('projects_config', default_value='',
                              description='Optional YAML project contracts, merged by ID'),
        DeclareLaunchArgument('asset_packages', default_value='ros2_web_link',
                              description='Comma-separated ROS package resource allowlist'),
        DeclareLaunchArgument('write_queue_size', default_value='64',
                              description='Upstream bounded per-client outgoing message queue'),
        DeclareLaunchArgument('incoming_queue_size', default_value='100'),
        DeclareLaunchArgument('max_message_size', default_value='16000000'),
        DeclareLaunchArgument('fragment_timeout', default_value='30'),
        DeclareLaunchArgument('service_client_cache_size', default_value='0',
                              description='Node-owned cache slots; 0 keeps upstream clients'),
        DeclareLaunchArgument('service_client_max_in_flight', default_value='128'),
        DeclareLaunchArgument('service_client_creation_timeout', default_value='5.0'),
        DeclareLaunchArgument('topics_pub_glob', default_value="['/web_link_demo/*']"),
        DeclareLaunchArgument('topics_sub_glob', default_value="['*']"),
        DeclareLaunchArgument('services_glob',
                              default_value="['/rosapi/topics', '/rosapi/services', "
                              "'/rosapi/nodes', '/rosapi/action_servers', '/web_link_demo/*']"),
        DeclareLaunchArgument('actions_glob', default_value="['/web_link_demo/*']"),
    ]
    # The installed Jazzy upstream XML does not forward actions_glob. Retain
    # the public executables and the existing narrow octet[] compatibility fix.
    parameters = {
        'port': ParameterValue(LaunchConfiguration('ws_port'), value_type=int),
        'call_services_in_new_thread': True,
        'send_action_goals_in_new_thread': True,
        'default_call_service_timeout': 5.0,
    }
    for key in ('write_queue_size', 'incoming_queue_size', 'max_message_size',
                'fragment_timeout', 'service_client_cache_size', 'service_client_max_in_flight'):
        parameters[key] = ParameterValue(LaunchConfiguration(key), value_type=int)
    parameters['service_client_creation_timeout'] = ParameterValue(
        LaunchConfiguration('service_client_creation_timeout'), value_type=float)
    # Upstream parse_glob_string strips single quotes only. These are strings,
    # not ROS string-array parameters or JSON double-quoted lists.
    for key in ('address', 'topics_pub_glob', 'topics_sub_glob',
                'services_glob', 'actions_glob'):
        parameters[key] = ParameterValue(LaunchConfiguration(key), value_type=str)
    # Set a standard DDS profile only in our service-server children. Existing
    # operator profiles take precedence and must include any desired recovery
    # settings themselves. No global RMW, XML QoS or transport switch is made.
    service_profiles = EnvironmentVariable(
        'FASTDDS_DEFAULT_PROFILES_FILE', default_value=EnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', default_value=PathJoinSubstitution([
                FindPackageShare('ros2_web_link'), 'config', 'fastdds_services.xml'])))
    service_environment = {'FASTRTPS_DEFAULT_PROFILES_FILE': service_profiles}
    # Fast DDS 2.14 also loads this implicit cwd profile. Do not inject a
    # service profile which could replace that operator's writer defaults.
    if Path('DEFAULT_FASTRTPS_PROFILES.xml').is_file():
        service_environment = {}
    processes = [
        Node(package='ros2_web_link', executable='bridge_node.py',
             name='rosbridge_websocket', parameters=[parameters], output='screen',
             # Upstream 2.7.1 publisher unregister timers can keep Python alive
             # for its default 10s after the ROS main has returned. Retain that
             # upstream behavior and allow it to finish before SIGTERM.
             sigterm_timeout='15'),
        Node(package='ros2_web_link', executable='rosapi_node.py', name='rosapi',
             parameters=[{'params_glob': ParameterValue('[]', value_type=str)}],
             additional_env=service_environment, output='screen'),
        Node(package='ros2_web_link', executable='serve.py',
             condition=IfCondition(LaunchConfiguration('web')),
             arguments=['--address', LaunchConfiguration('address'),
                        '--port', LaunchConfiguration('web_port'),
                        '--ws-port', LaunchConfiguration('ws_port'),
                        '--profile', LaunchConfiguration('profile'),
                        '--config', LaunchConfiguration('config'),
                        '--project', LaunchConfiguration('project'),
                        '--projects-config', LaunchConfiguration('projects_config'),
                        '--asset-packages', LaunchConfiguration('asset_packages')],
             output='screen'),
        Node(package='ros2_web_link', executable='demo_node.py',
             condition=IfCondition(LaunchConfiguration('demo')),
             additional_env=service_environment, output='screen'),
    ]
    handlers = [
        RegisterEventHandler(OnProcessExit(target_action=process, on_exit=_stop_group))
        for process in processes
    ]
    return LaunchDescription(arguments + handlers + processes)
