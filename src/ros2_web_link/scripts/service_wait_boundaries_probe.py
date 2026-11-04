#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""
Controlled Future/Event and sink boundaries, without a ROS context or DDS.

Execute the installed upstream call_service functions unchanged. Replace only
native entity creation and executor scheduling; use real rclpy Future and
threading.Event. This verifies the observer, not the real long-test fault.
"""

import argparse
import gc
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import weakref

from rclpy.task import Future
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.internal import services

import service_wait_trace

trace = service_wait_trace

if os.environ.get('WEB_LINK_WAIT_TRACE_SOURCE'):
    spec = importlib.util.spec_from_file_location(
        'legacy_wait_trace', os.environ['WEB_LINK_WAIT_TRACE_SOURCE'])
    trace = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(trace)


class DeferredExecutor:
    """Hold completion callbacks as a busy executor would, then release them."""

    def __init__(self):
        self.tasks = []

    def create_task(self, *args):
        """Keep the real callback and Future until this controlled check ends."""
        self.tasks.append(args)


class BrokenSink:
    """Model write/flush failure and a secondary stderr failure."""

    def __init__(self, fail_write=True):
        self.writes = 0
        self.fail_write = fail_write

    def write(self, value):
        """Count attempts, then either accept bytes or report a full device."""
        self.writes += 1
        if self.fail_write:
            raise OSError('controlled full device')

    def flush(self):
        """Report a device error after a successful write."""
        raise OSError('controlled flush failure')

    def close(self):
        """No native resource exists in this controlled sink."""


def main():
    """Check a single isolated observer scenario."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['done', 'sampling', 'write', 'flush'], required=True)
    args = parser.parse_args()
    executor, references, responses = DeferredExecutor(), [], []

    class Client:
        """Return a real finished Future while its notification stays queued."""

        def wait_for_service(self, timeout):
            """No native endpoint discovery is performed by this controlled check."""
            return True

        def call_async(self, request):
            """Finish the real Future but leave its callback in the controlled queue."""
            future = Future()
            # Controlled scheduling only; not a public production integration API.
            future._set_executor(executor)
            future.set_result(object())
            references.append(weakref.ref(future))
            if args.case == 'sampling':
                return SimpleNamespace(add_done_callback=lambda callback: None,
                                       cancel=lambda: None)
            return future

    service_types = [('/rosapi/nodes', ['std_srvs/srv/Trigger'])]
    node = SimpleNamespace(resolve_service_name=lambda name: name,
                           get_service_names_and_types=lambda: service_types,
                           create_client=lambda *a, **kw: Client())
    capability = object.__new__(CallService)
    capability.protocol = SimpleNamespace(node_handle=node, log=lambda *a: None,
                                          send=responses.append)
    with tempfile.TemporaryDirectory(prefix='ros2-web-wait-boundaries-') as directory:
        root = Path(directory)
        os.environ['WEB_LINK_WAIT_SERVICE'] = '/rosapi/nodes'
        os.environ['WEB_LINK_EVIDENCE'] = str(root)
        sink = BrokenSink(fail_write=args.case != 'flush')
        previous_open = Path.open

        def open_output(path, *a, **kw):
            if args.case in ('write', 'flush') and path.name.startswith('service-waits-'):
                return sink
            return previous_open(path, *a, **kw)

        with patch.object(Path, 'open', open_output):
            trace.install_service_wait_trace()
        with patch.object(services, 'run_on_executor', lambda owner, create: create()):
            with patch.object(services, '_destroy_client_async', lambda *a: None):
                for index in range(20 if args.case in ('write', 'flush') else 1):
                    message = {'service': '/rosapi/nodes', 'id': f'controlled-{index}',
                               'args': {}, 'timeout': .001}
                    original_error = RuntimeError('controlled original send exception')

                    def send(response):
                        responses.append(response)
                        if args.case == 'write' and index == 0:
                            raise original_error

                    capability.protocol.send = send
                    with patch.object(sys, 'stderr', BrokenSink(fail_write=args.case != 'flush')):
                        if args.case == 'write' and index == 0:
                            try:
                                capability.call_service(message)
                            except Exception as error:
                                assert error is original_error, (
                                    'Diagnostic replaced original error')
                            else:
                                raise AssertionError('Original send error was swallowed')
                        else:
                            assert capability.call_service(message) is None
                    assert responses[-1]['result'] is False
                    executor.tasks.clear()
        gc.collect()
        assert all(ref() is None for ref in references), 'Diagnostic retained a native Future'
        if args.case in ('write', 'flush'):
            assert sink.writes <= 8, f'{sink.writes} writes exceeded the failure budget'
        else:
            paths = list(root.glob('service-waits-*.jsonl'))
            record = json.loads(paths[0].read_text().splitlines()[0])
            if args.case == 'done':
                assert record['trigger']['waitState'] == {
                    'futureDone': True, 'futureCancelled': False, 'eventSet': False}
            else:
                assert 'samplingError' in record['trigger']
                assert 'waitState' not in record['trigger']
        print(f'PASS controlled {args.case}: original response/error preserved; '
              'no Future retained')


if __name__ == '__main__':
    main()
