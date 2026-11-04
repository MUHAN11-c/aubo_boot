#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check final success metadata using unchanged upstream calls, without DDS."""

import argparse
import atexit
from contextlib import nullcontext
import gc
import importlib.util
import json
import os
from pathlib import Path
import tempfile
from threading import Lock
from types import SimpleNamespace
from unittest.mock import patch
import weakref

from rclpy.task import Future
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.internal import services
import service_wait_trace
from std_srvs.srv import Trigger


def main():
    """Retain bounded metadata from successful-only calls at normal exit."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['success', 'write', 'flush', 'busy'],
                        default='success')
    args = parser.parse_args()
    trace = service_wait_trace
    if os.environ.get('WEB_LINK_WAIT_TRACE_SOURCE'):
        spec = importlib.util.spec_from_file_location(
            'legacy_final_trace', os.environ['WEB_LINK_WAIT_TRACE_SOURCE'])
        trace = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(trace)
    registered, references, responses = [], [], []

    class Client:
        """Complete a real Future, leaving upstream Event and callbacks intact."""

        def wait_for_service(self, timeout):
            """No native discovery is needed in this controlled check."""
            return True

        def call_async(self, request):
            """Use a real service response but never persist its private value."""
            future = Future()
            future.set_result(Trigger.Response(success=True, message='private-result'))
            references.append(weakref.ref(future))
            return future

    node = SimpleNamespace(
        resolve_service_name=lambda name: name,
        get_service_names_and_types=lambda: [('/rosapi/nodes', ['std_srvs/srv/Trigger'])],
        create_client=lambda *a, **kw: Client())
    capability = object.__new__(CallService)
    capability.protocol = SimpleNamespace(node_handle=node, log=lambda *a: None,
                                          send=responses.append)
    with tempfile.TemporaryDirectory(prefix='ros2-web-wait-final-') as directory:
        root = Path(directory)
        os.environ['WEB_LINK_WAIT_SERVICE'] = '/rosapi/nodes'
        os.environ['WEB_LINK_EVIDENCE'] = str(root)
        busy = Lock()
        lock_patch = patch.object(trace, 'Lock', lambda: busy) if args.case == 'busy' else (
            nullcontext())
        with lock_patch:
            with patch.object(atexit, 'register', lambda fn, *a: registered.append((fn, a))):
                trace.install_service_wait_trace()
        with patch.object(services, 'run_on_executor', lambda owner, create: create()):
            with patch.object(services, '_destroy_client_async', lambda *a: None):
                for index in range(70):
                    capability.call_service({'service': '/rosapi/nodes',
                                             'id': f'completed-{index}', 'timeout': 3})
        assert len(responses) == 70 and all(x['result'] for x in responses)
        gc.collect()
        assert all(ref() is None for ref in references), 'Final trace retained native Future'
        # Simulate atexit's LIFO execution while the controlled output still exists.
        attempts = []
        original_open = Path.open

        class BrokenFinal:
            """Keep final-snapshot IO failure separate from service outcome."""

            def __enter__(self):
                """Return the controlled sink."""
                return self

            def __exit__(self, *values):
                """Model a flush/close failure after write succeeds."""
                if args.case == 'flush':
                    raise OSError('controlled final close/flush failure')

            def write(self, value):
                """Count attempts before reporting a controlled write failure."""
                attempts.append(value)
                if args.case == 'write':
                    raise OSError('controlled final write failure')

        def open_final(path, *values, **kwargs):
            """Replace only the final metadata file, never upstream outputs."""
            if path.name.startswith('service-wait-final-'):
                return BrokenFinal()
            return original_open(path, *values, **kwargs)

        if args.case == 'busy':
            busy.acquire()
        try:
            for fn, values in reversed(registered):
                if args.case in ('success', 'busy'):
                    fn(*values)
                else:
                    with patch.object(Path, 'open', open_final):
                        with patch('sys.stderr', SimpleNamespace(write=lambda *a: (
                                _ for _ in ()).throw(OSError('stderr full')))):
                            fn(*values)
                            fn(*values)  # A final write is attempted at most once.
        finally:
            if args.case == 'busy':
                busy.release()
        if args.case in ('write', 'flush'):
            assert len(attempts) == 1, f'{len(attempts)} final writes exceeded one attempt'
            print(f'PASS final {args.case} + stderr failure isolated; one attempt')
            return
        paths = list(root.glob('service-wait-final-*.json'))
        assert len(paths) == 1, 'Missing final snapshot for successful bridge calls'
        data = paths[0].read_bytes()
        assert len(data) <= 131072, 'Final metadata exceeds byte budget'
        assert b'private-result' not in data, 'Response payload leaked into metadata'
        record = json.loads(data)
        if args.case == 'busy':
            assert record['snapshotAvailable'] is False and record['recent'] == []
            print('PASS final snapshot does not block on a held native lock; marked unavailable')
            return
        assert len(record['recent']) == 64
        assert record['recent'][0]['cid'] == 'completed-6'
        assert record['recent'][-1]['cid'] == 'completed-69'
        for item in record['recent']:
            assert item['waitReturned'] is True
            assert item['callEnteredAt'] <= item['waitEnteredAt'] <= item['waitReturnedAt']
            assert item['waitReturnedAt'] <= item['callReturnedAt']
            assert item['callEnteredWall'] > 0
        assert all(key in record['clockAtSnapshot'] for key in ('wall', 'monotonic'))
        print('PASS successful-only final metadata: 64 calls, byte bound, no payload/Future')


if __name__ == '__main__':
    main()
