#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check actual benchmark CLI environment selection before any ROS process starts."""

import importlib.util
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

import run_transport_benchmark

benchmark = run_transport_benchmark

if os.environ.get('WEB_LINK_TRACE_MODES_SOURCE'):
    spec = importlib.util.spec_from_file_location(
        'legacy_benchmark', os.environ['WEB_LINK_TRACE_MODES_SOURCE'])
    benchmark = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark)


class BeforeSpawn(Exception):
    """Stop precisely at the benchmark's first external graph query."""


with tempfile.TemporaryDirectory(prefix='ros2-web-trace-modes-') as directory:
    root = Path(directory)
    bag = root / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 1000000000\n')
    count = 0
    # Begin with the metadata mode to expose an incorrectly bound else.
    for mode in ('trace-services', 'trace-waits', '', 'server', 'native', 'late'):
        for inherited in range(32):
            environment = dict(os.environ)
            for key, bit in (('WEB_LINK_TRACE_SERVICE', 1), ('WEB_LINK_WAIT_SERVICE', 2),
                             ('WEB_LINK_TRACE_SERVER_SERVICE', 4),
                             ('WEB_LINK_NATIVE_SERVICE', 8),
                             ('WEB_LINK_LATE_CLIENT_SERVICE', 16)):
                environment.pop(key, None)
                if inherited & bit:
                    environment[key] = '/inherited/endpoint'

            def query(command, **kwargs):
                """Inspect the real main() path, replacing only the process boundary."""
                assert command[:3] == ['ros2', 'node', 'list']
                env = kwargs['env']
                expected_meta = '/rosapi/nodes' if mode == 'trace-services' else None
                expected_wait = ('/rosapi/nodes' if mode in (
                    'trace-waits', 'server', 'native', 'late') else None)
                expected_server = '/rosapi/nodes' if mode in ('server', 'late') else None
                expected_native = '/rosapi/nodes' if mode in ('native', 'late') else None
                expected_late = '/rosapi/nodes' if mode == 'late' else None
                assert env.get('WEB_LINK_TRACE_SERVICE') == expected_meta, (
                    mode, inherited, 'metadata selection', env.get('WEB_LINK_TRACE_SERVICE'))
                assert env.get('WEB_LINK_WAIT_SERVICE') == expected_wait, (
                    mode, inherited, 'wait selection', env.get('WEB_LINK_WAIT_SERVICE'))
                assert env.get('WEB_LINK_TRACE_SERVER_SERVICE') == expected_server, (
                    mode, inherited, 'server selection', env.get('WEB_LINK_TRACE_SERVER_SERVICE'))
                assert env.get('WEB_LINK_NATIVE_SERVICE') == expected_native, (
                    mode, inherited, 'native selection', env.get('WEB_LINK_NATIVE_SERVICE'))
                assert env.get('WEB_LINK_LATE_CLIENT_SERVICE') == expected_late, (
                    mode, inherited, 'late selection', env.get('WEB_LINK_LATE_CLIENT_SERVICE'))
                raise BeforeSpawn()

            argv = ['probe', '--bag', str(bag), '--evidence', str(root / str(count)),
                    '--domain', '148', '--multi-client-seconds', '30']
            if mode == 'server':
                argv.extend(['--trace-waits', '--trace-server'])
            elif mode == 'late':
                argv.extend(['--trace-waits', '--trace-server', '--trace-native',
                             '--trace-late-client'])
            elif mode == 'native':
                argv.extend(['--trace-waits', '--trace-native'])
            elif mode:
                argv.append('--' + mode)
            with patch.dict(os.environ, environment, clear=True), patch.object(sys, 'argv', argv):
                with patch.object(benchmark.subprocess, 'run', query):
                    with patch.object(benchmark.subprocess, 'Popen') as spawn:
                        try:
                            benchmark.main()
                        except BeforeSpawn:
                            pass
                        else:
                            raise AssertionError('Probe did not stop before process creation')
                        spawn.assert_not_called()
            count += 1
    reused = root / 'existing'
    reused.mkdir()
    stale = reused / 'service-native-7.json'
    stale.write_text('{"previousRun":true}')
    argv = ['probe', '--bag', str(bag), '--evidence', str(reused), '--domain', '148']
    with patch.object(sys, 'argv', argv):
        with patch.object(benchmark.subprocess, 'run') as query:
            query.side_effect = AssertionError('Existing evidence reached graph query')
            with patch.object(benchmark.subprocess, 'Popen') as spawn:
                try:
                    benchmark.main()
                except FileExistsError:
                    pass
                else:
                    raise AssertionError('Existing evidence directory was reused')
                query.assert_not_called()
                spawn.assert_not_called()
    assert stale.read_text() == '{"previousRun":true}'
    assert sorted(p.name for p in reused.iterdir()) == [stale.name]
    print(f'PASS {count} mode/environment combinations and atomic refusal of reused evidence; '
          'no ROS process')
