#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check bounded post-timeout reader retention without ROS entities."""

import argparse
import atexit
import gc
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch
import weakref


def main():
    """Hold only armed failures, release references and keep cleanup forwarding."""
    import service_late_client as module
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['normal', 'constructor', 'budget', 'exit', 'sink'],
                        default='normal')
    args = parser.parse_args()
    callbacks, destroyed = [], []

    class Client:
        """Represent only lifetime, without simulating a ROS response."""

    node = SimpleNamespace(context=SimpleNamespace(ok=lambda: True))

    def original(owner, client):
        """Record scheduling without retaining the client."""
        assert owner is node
        destroyed.append(id(client))

    with tempfile.TemporaryDirectory(prefix='ros2-web-late-client-') as directory:
        root = Path(directory)
        with patch.object(atexit, 'register', callbacks.append):
            trace = module.LateClientTrace(root, original, hold_seconds=.03)
        normal = Client()
        trace.destroy(node, normal)
        assert destroyed == [id(normal)], 'Normal cleanup was delayed'
        if args.case == 'constructor':
            client = Client()
            assert trace.arm(id(client), 'timer-constructor-failure')
            with patch.object(module, 'Timer', side_effect=RuntimeError('no timer resources')):
                trace.destroy(node, client)
            assert destroyed[-1] == id(client), 'Timer failure prevented original cleanup'
            callbacks[0]()
            print('PASS timer constructor error isolated; original cleanup forwarded')
            return
        if args.case == 'budget':
            clients = [Client() for _ in range(4)]
            assert [trace.arm(id(c), str(i)) for i, c in enumerate(clients)] == [
                True, True, True, False]
            for c in clients:
                trace.destroy(node, c)
            time.sleep(.1)
            callbacks[0]()
            try:
                module.read_late_client_trace(root)
            except ValueError:
                pass
            else:
                raise AssertionError('Exceeded hold budget accepted')
            record = json.loads(next(root.glob('service-late-client-*.json')).read_text())
            assert len(record['records']) == 3 and record['activeTimers'] == 0
            print('PASS at most three holds; overflow flagged; all cleanup forwarded')
            return
        client = Client()
        client_id = id(client)
        reference = weakref.ref(client)
        assert trace.arm(client_id, 'failed-cid')
        trace.destroy(node, client)
        assert client_id not in destroyed
        del client
        assert reference() is not None, 'Reader was not retained after timeout'
        if args.case == 'exit':
            callbacks[0]()
            gc.collect()
            assert reference() is None, 'Cancelled timer retained the reader'
            record = json.loads(next(root.glob('service-late-client-*.json')).read_text())
            assert record['cancelledOnExit'] == 1 and record['activeTimers'] == 0
            try:
                module.read_late_client_trace(root)
            except ValueError:
                pass
            else:
                raise AssertionError('Cancelled observation accepted')
            print('PASS interrupted observation cancels timer/releases refs; evidence incomplete')
            return
        deadline = time.monotonic()+1
        while reference() is not None and time.monotonic() < deadline:
            gc.collect()
            time.sleep(.005)
        assert reference() is None, 'Reader retained beyond timer completion'
        assert destroyed == [id(normal), client_id]
        assert not list(root.glob('service-late-client-*.json')), 'Hot-path file IO'
        if args.case == 'sink':
            attempts = []

            def broken(*args, **kwargs):
                """Fail only diagnostic output and its fallback."""
                attempts.append(True)
                raise OSError('full sink')

            with patch.object(Path, 'open', broken):
                with patch('sys.stderr', SimpleNamespace(write=broken)):
                    callbacks[0]()
                    callbacks[0]()
            assert len(attempts) == 2
            print('PASS final IO/stderr failure isolated; one attempt')
            return
        callbacks[0]()
        callbacks[0]()
        paths = list(root.glob('service-late-client-*.json'))
        assert len(paths) == 1
        record = json.loads(paths[0].read_text())
        assert record['records'][0]['cid'] == 'failed-cid'
        assert record['records'][0]['cleanupSubmittedAt'] >= (
            record['records'][0]['destroyRequestedAt']+.02)
        assert record['activeTimers'] == 0 and record['errors'] == 0
        assert record['snapshotAvailable'] is True
        print('PASS normal cleanup immediate; only armed timeout held; '
              'references released; exit IO')


if __name__ == '__main__':
    main()
