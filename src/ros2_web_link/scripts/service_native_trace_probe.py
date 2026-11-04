#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check native-boundary observation without creating ROS entities."""

import argparse
import atexit
import gc
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import weakref


def main():
    """Require native send/take evidence, original results and bounded retention."""
    import service_native_trace as trace
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['basic', 'budget', 'sink', 'guard'], default='basic')
    args = parser.parse_args()
    if args.case == 'guard':
        native = trace._reviewed_client()  # Import/hash only: no ROS context or entity.
        original = native.take_response
        with patch.object(trace, 'version', lambda name: 'unreviewed'):
            try:
                trace._reviewed_client()
            except RuntimeError:
                pass
            else:
                raise AssertionError('Unreviewed build was accepted')
        assert native.take_response is original
        print('PASS exact local native build accepted; unreviewed build refused before hooks')
        return
    original_error = RuntimeError('private-exception-payload')
    payload = SimpleNamespace(secret='private-response-payload')
    header = SimpleNamespace(request_id=SimpleNamespace(sequence_number=7),
                             source_timestamp=1793249126000000123,
                             received_timestamp=1793249126000000456)
    result = (header, payload)
    active = {'cid': 'native-cid'}
    registered = []

    class Native:
        """Model only the reviewed pybind method/property boundary."""

        service_name = '/rosapi/nodes'
        pointer = 1234

        def send_request(self, request):
            """Return a native sequence or preserve the precise original error."""
            if request is original_error:
                raise request
            return 7

        def take_response(self, response_type):
            """Return the original tuple, including empty native takes."""
            if response_type is original_error:
                raise response_type
            return (None, None) if response_type is None else result

    with tempfile.TemporaryDirectory(prefix='ros2-web-native-boundary-') as directory:
        root = Path(directory)
        original_send, original_take = Native.send_request, Native.take_response
        assert trace.install_native_client_trace('', lambda: active, root) is False
        assert Native.send_request is original_send and Native.take_response is original_take
        with patch.object(trace, '_reviewed_client', lambda: Native):
            with patch.object(atexit, 'register', lambda fn: registered.append(fn)):
                assert trace.install_native_client_trace('/rosapi/nodes', lambda: active, root)
                assert trace.install_native_client_trace('/rosapi/nodes', lambda: active, root)
        assert len(registered) == 1
        try:
            trace.install_native_client_trace('/other', lambda: active, root)
        except RuntimeError:
            pass
        else:
            raise AssertionError('Native trace retargeted an existing hook')
        client = Native()
        reference = weakref.ref(client)
        assert client.send_request(SimpleNamespace(secret='private-request-payload')) == 7
        active = None  # The executor takes responses on a different thread/context.
        assert client.take_response(object()) is result
        assert client.take_response(None) == (None, None)
        header.source_timestamp, header.received_timestamp = 0, -1
        assert client.take_response(object()) is result
        for method in (client.send_request, client.take_response):
            try:
                method(original_error)
            except RuntimeError as error:
                assert error is original_error
            else:
                raise AssertionError('Native exception was swallowed')
        del method
        original_error.__traceback__ = None
        # Accessor errors are diagnostic-only: original native methods still run.
        client.pointer = property(lambda self: None)
        assert client.send_request(object()) == 7
        client.pointer = 1234
        # No filesystem writes in send/take windows.
        assert not list(root.glob('service-native-*.json'))
        del client
        gc.collect()
        assert reference() is None, 'Diagnostic retained a native Client'
        if args.case == 'budget':
            for _ in range(12100):
                Native().send_request(object())
        if args.case == 'sink':
            attempts = []

            def broken_open(*values, **kwargs):
                """Fail both output and fallback without affecting native methods."""
                attempts.append(True)
                raise OSError('controlled native metadata sink failure')

            with patch.object(Path, 'open', broken_open):
                with patch('sys.stderr', SimpleNamespace(write=broken_open)):
                    registered[0]()
                    registered[0]()
            assert len(attempts) == 2  # One file attempt and one isolated stderr attempt.
            print('PASS native final sink/stderr errors isolated; no retry')
            return
        registered[0]()
        registered[0]()  # One exit attempt only.
        paths = list(root.glob('service-native-*.json'))
        assert len(paths) == 1, 'Missing native send/take boundary evidence'
        raw = paths[0].read_bytes()
        assert not any(value in raw for value in (
            b'private-response-payload', b'private-request-payload', b'private-exception-payload'))
        record = json.loads(raw)
        events = record['events']
        assert [e['phase'] for e in events[:6]] == [
            'sendEnter', 'sendReturn', 'takeEnter', 'takeReturn', 'takeEnter', 'takeReturn']
        assert events[0]['cid'] == events[1]['cid'] == 'native-cid'
        assert events[1]['sequence'] == events[3]['sequence'] == 7
        assert events[3]['headerPresent'] is True and events[5]['headerPresent'] is False
        assert events[3]['sourceTimestampNs'] == '1793249126000000123'
        assert events[3]['receivedTimestampNs'] == '1793249126000000456'
        assert events[7]['sourceTimestampNs'] == '0'
        assert events[7]['receivedTimestampNs'] == '-1'
        assert all(e['clientKey'] == events[0]['clientKey'] for e in events[:6])
        assert record['metadataErrors'] >= 1
        if args.case == 'budget':
            assert len(events) <= 24000 and record['encodedEventBytes'] <= 8388608
            assert record['droppedRecords'] > 0 and record['budgetExceeded'] is True
            try:
                trace.read_native_trace(root, '/rosapi/nodes')
            except ValueError:
                pass
            else:
                raise AssertionError('Incomplete metadata was accepted')
            print('PASS native metadata stays bounded; incomplete snapshot rejected')
            return
        assert record['droppedRecords'] == 0 and record['budgetExceeded'] is False
        assert record['snapshotAvailable'] is True
        print('PASS native boundary: CID/sequence, empty takes, identity, exceptions, '
              'privacy, exit IO')


if __name__ == '__main__':
    main()
