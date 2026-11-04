#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check the actual benchmark CLI queue selection without starting ROS processes."""

from contextlib import redirect_stderr
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

import run_transport_benchmark as benchmark


class BeforeSpawn(Exception):
    """Stop at the real launch process boundary after capturing its arguments."""


class ClosedPort:
    """Represent an unused local port without opening sockets in this probe."""

    def __enter__(self):
        """Return the controlled socket surface."""
        return self

    def __exit__(self, *unused):
        """Leave no socket or file descriptor to close."""

    def connect_ex(self, unused):
        """Report no listener."""
        return 1


with tempfile.TemporaryDirectory(prefix='ros2-web-queue-cli-') as directory:
    root = Path(directory)
    share = root / 'share'
    (share / 'config').mkdir(parents=True)
    (share / 'config/projects.yaml').write_text('projects:\n  peach_replay: {}\n')
    bag = root / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 1000000000\n')
    count = 0
    for selected in (None, 1, 16, 4096):
        evidence = root / f'valid-{selected}'
        argv = ['probe', '--bag', str(bag), '--evidence', str(evidence), '--domain', '176']
        if selected is not None:
            argv.extend(['--write-queue-size', str(selected)])
        captured = []

        def spawn(command, **kwargs):
            """Inspect the actual launch command, then prevent process creation."""
            assert command[:4] == ['ros2', 'launch', 'ros2_web_link', 'replay.launch.py']
            captured.append(command)
            assert kwargs['start_new_session'] is True
            raise BeforeSpawn('explicit controlled launch boundary')

        with (
            patch.object(sys, 'argv', argv), redirect_stderr(io.StringIO()),
            patch.object(benchmark, 'get_package_share_directory', return_value=str(share)),
            patch.object(benchmark.socket, 'socket', return_value=ClosedPort()),
            patch.object(benchmark.psutil, 'process_iter', return_value=[]),
            patch.object(benchmark.subprocess, 'run', return_value=SimpleNamespace(stdout='')),
            patch.object(benchmark.subprocess, 'Popen', spawn),
        ):
            try:
                benchmark.main()
            except BeforeSpawn:
                pass
            except SystemExit as error:
                raise AssertionError('Valid queue selection must reach launch') from error
        assert len(captured) == 1
        queue_args = [item for item in captured[0]
                      if item.startswith('bridge_write_queue_size:=')]
        assert queue_args == ([] if selected is None
                              else [f'bridge_write_queue_size:={selected}'])
        count += 1
    for invalid in ('0', '-1', '4097', '1.5', 'invalid', 'nan', 'inf'):
        evidence = root / f'invalid-{invalid}'
        argv = ['probe', '--bag', str(bag), '--evidence', str(evidence), '--domain', '176',
                '--write-queue-size', invalid]
        with patch.object(sys, 'argv', argv), redirect_stderr(io.StringIO()):
            with patch.object(benchmark.subprocess, 'run') as query:
                with patch.object(benchmark.subprocess, 'Popen') as process:
                    try:
                        benchmark.main()
                    except SystemExit as error:
                        assert error.code == 2
                    else:
                        raise AssertionError('Invalid queue limit was accepted')
                    query.assert_not_called()
                    process.assert_not_called()
        assert not evidence.exists(), 'Invalid input must not claim an evidence directory'
        count += 1
    # Construct the actual replay Include action, but never execute a launch.
    spec = importlib.util.spec_from_file_location(
        'queue_replay', Path(__file__).resolve().parents[1] / 'launch/replay.launch.py')
    replay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(replay)
    web_spec = importlib.util.spec_from_file_location(
        'queue_web', Path(__file__).resolve().parents[1] / 'launch/web.launch.py')
    web = importlib.util.module_from_spec(web_spec)
    web_spec.loader.exec_module(web)
    (share / 'config/projects.yaml').write_text(
        'projects:\n  peach_replay:\n    cases:\n      - kind: topic\n'
        '        topic: {name: /points, timestampMode: recorded}\n')
    for selected in ('', '1', '16', '4096'):
        with patch.dict('os.environ', {'ROS_LOG_DIR': str(root / 'ros-log')}):
            context = LaunchContext()
        context.launch_configurations.update({
            'bag': str(bag), 'project': 'peach_replay', 'projects_config': '',
            'config': '', 'rate': '1', 'loop': 'true', 'address': '127.0.0.1',
            'web_port': '21288', 'ws_port': '22890', 'asset_packages': 'ros2_web_link',
            'bridge_write_queue_size': selected,
            'bridge_service_client_cache_size': '',
        })
        with (
            patch.object(replay, 'get_package_share_directory', return_value=str(share)),
            patch.dict('os.environ', {'ROS_LOG_DIR': str(root / 'ros-log')}),
        ):
            actions = replay._start(context)
        forwarded = dict(actions[1].launch_arguments)
        assert forwarded.get('write_queue_size') == (selected or None), (
            'Replay must explicitly forward the selected upstream queue size')
        count += 1
        with patch.dict('os.environ', {'ROS_LOG_DIR': str(root / 'ros-log')}):
            declaration = next(action for action in web.generate_launch_description().entities
                               if isinstance(action, DeclareLaunchArgument)
                               and action.name == 'write_queue_size')
        for key, value in forwarded.items():
            SetLaunchConfiguration(key, value).execute(context)
        declaration.execute(context)
        try:
            actual = ParameterValue(
                LaunchConfiguration('write_queue_size'), value_type=int).evaluate(context)
        except Exception as error:
            raise AssertionError('Inherited queue default must remain valid') from error
        assert actual == (int(selected) if selected else 64)
        count += 1
    context.launch_configurations.update(
        {'bridge_write_queue_size': '', 'write_queue_size': '32'})
    with patch.object(replay, 'get_package_share_directory', return_value=str(share)):
        actions = replay._start(context)
    assert 'write_queue_size' not in dict(actions[1].launch_arguments)
    declaration.execute(context)
    assert ParameterValue(
        LaunchConfiguration('write_queue_size'), value_type=int).evaluate(context) == 32
    count += 1
    for invalid in ('0', '-1', '4097', '1.5', 'invalid'):
        context.launch_configurations['bridge_write_queue_size'] = invalid
        with (
            patch.object(replay, 'get_package_share_directory', return_value=str(share)),
            patch.dict('os.environ', {'ROS_LOG_DIR': str(root / 'ros-log')}),
        ):
            try:
                replay._start(context)
            except ValueError:
                pass
            else:
                raise AssertionError('Invalid replay queue limit was accepted')
        count += 1
print(f'PASS {count} queue CLI groups; defaults preserved, exact launch forwarding, '
      'no ROS process')
