# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""
Replay selected observation topics and compose the existing Web workbench.

Use an isolated ROS_DOMAIN_ID. This launch starts no camera, hardware driver,
application service or action server. rosbag2 owns storage and playback time.
"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def _start(context):
    """Use only observation topics declared by the selected data contract."""
    share = Path(get_package_share_directory('ros2_web_link'))

    def value(key):
        return LaunchConfiguration(key).perform(context)

    bag = Path(value('bag')).expanduser().resolve()
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'bag must contain a finalized metadata.yaml: {bag}')
    rate = float(value('rate'))
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('rate must be finite and greater than zero')
    looping = value('loop').lower()
    if looping not in ('true', 'false'):
        raise ValueError('loop must be true or false')
    queue_text = value('bridge_write_queue_size').strip()
    queue_size = None
    if queue_text:
        try:
            queue_size = int(queue_text)
        except ValueError as error:
            raise ValueError('write_queue_size must be an integer from 1 to 4096') from error
        if not 1 <= queue_size <= 4096:
            raise ValueError('write_queue_size must be an integer from 1 to 4096')
    cache_text = value('bridge_service_client_cache_size').strip()
    cache_size = None
    if cache_text:
        try:
            cache_size = int(cache_text)
        except ValueError as error:
            raise ValueError(
                'service_client_cache_size must be an integer from 0 to 1024') from error
        if not 0 <= cache_size <= 1024:
            raise ValueError('service_client_cache_size must be an integer from 0 to 1024')
    projects = yaml.safe_load((share / 'config/projects.yaml').read_text())['projects']
    custom = value('projects_config')
    if custom:
        projects.update(yaml.safe_load(Path(custom).read_text())['projects'])
    project = projects.get(value('project'))
    if not isinstance(project, dict):
        raise ValueError('unknown replay project')
    # Do not expose live checks as a green replay result. Reuse the same
    # project contract; no second handwritten topic/command registry.
    cases = project.get('cases', [])
    if any(c.get('kind') not in ('graph', 'topic', 'image', 'cloud', 'tf', 'interface')
           or c.get('interactive') for c in cases):
        raise ValueError('replay requires a read-only observation project')
    topics = set()
    for case in cases:
        if 'topic' in case:
            topic = case['topic']
            if topic.get('timestampMode') != 'recorded':
                raise ValueError('replay topic checks must set timestampMode: recorded')
            topics.add(topic['name'])
    view = project.get('view', {})
    topics.update(view.get('scene', {}).get(key) for key in (
        'robotDescription', 'jointStates', 'tf', 'tfStatic', 'markers', 'pointCloud', 'path'))
    for key in ('image', 'depthImage', 'plot'):
        topics.add(view.get(key, {}).get('topic'))
    for key in ('diagnostics', 'rosout'):
        topics.add(view.get(key))
    topics.discard(None)
    topics.discard('/clock')  # rosbag2 publishes its own playback clock.
    if not topics or any(not isinstance(t, str) or not t.startswith('/') for t in topics):
        raise ValueError('replay requires explicit absolute observation topic names')
    command = ['ros2', 'bag', 'play', str(bag), '--rate', str(rate),
               '--clock', '30', '--delay', '2', '--read-ahead-queue-size', '100',
               '--qos-profile-overrides-path', str(share / 'config/replay_qos.yaml'),
               '--disable-keyboard-controls', '--topics', *sorted(topics)]
    if looping == 'true':
        command.append('--loop')
    # Keep recovery QoS local to this observation-only player. An operator's
    # existing profile takes precedence and must include desired writer times.
    # Other RMWs ignore this Fast DDS environment variable; never switch RMW.
    profiles = context.environment.get('FASTDDS_DEFAULT_PROFILES_FILE', context.environment.get(
        'FASTRTPS_DEFAULT_PROFILES_FILE', str(share / 'config/fastdds_replay.xml')))
    environment = {'FASTRTPS_DEFAULT_PROFILES_FILE': profiles}
    if Path('DEFAULT_FASTRTPS_PROFILES.xml').is_file():
        environment = {}  # Preserve upstream implicit/explicit XML composition.
    player = ExecuteProcess(cmd=command, output='screen', additional_env=environment)
    # An exhausted/failed player must not leave a green but empty web stack.
    stop = RegisterEventHandler(OnProcessExit(
        target_action=player,
        on_exit=[EmitEvent(event=Shutdown(reason='rosbag playback ended'))]))
    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/web.launch.py')),
        launch_arguments={
            'demo': 'false', 'profile': 'robot', 'project': value('project'),
            'projects_config': custom, 'address': value('address'),
            'web_port': value('web_port'), 'ws_port': value('ws_port'),
            'asset_packages': value('asset_packages'),
            'config': value('config'),
            'topics_pub_glob': '[]', 'actions_glob': '[]',
            'services_glob': "['/rosapi/*']",
            **({'write_queue_size': str(queue_size)} if queue_size is not None else {}),
            **({'service_client_cache_size': str(cache_size)} if cache_size is not None else {}),
        }.items())
    return [stop, web, player]


def generate_launch_description():
    """Require a local finalized bag and keep playback looping for manual use."""
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Absolute path to finalized rosbag2 directory'),
        DeclareLaunchArgument('project', default_value='peach_replay'),
        DeclareLaunchArgument('projects_config', default_value=''),
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('address', default_value='127.0.0.1'),
        DeclareLaunchArgument('web_port', default_value='8388'),
        DeclareLaunchArgument('ws_port', default_value='9390'),
        DeclareLaunchArgument('asset_packages', default_value='ros2_web_link'),
        # A distinct optional name avoids shadowing the included web launch's
        # same-scope write_queue_size default with an empty parent value.
        DeclareLaunchArgument('bridge_write_queue_size', default_value='',
                              description='Optional upstream queue in messages, 1..4096; '
                              'empty preserves the web launch default'),
        DeclareLaunchArgument('bridge_service_client_cache_size', default_value='',
                              description='Optional cache slots; empty preserves defaults'),
        OpaqueFunction(function=_start),
    ])
