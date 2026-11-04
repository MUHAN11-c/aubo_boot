# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""
Observe reviewed rclpy native Client boundaries in an opt-in bridge process.

This is version-limited diagnosis, not a public SDK API or a transport fix.
Do not dereference native memory, retain Clients/headers/payloads, add entities
or Future callbacks, change results, or write files inside native calls.
"""

import atexit
import hashlib
from importlib.metadata import version
import inspect
import json
import os
from pathlib import Path
import sys
from threading import get_ident, Lock
import time

_installed = None
_NATIVE_HASH = '53efd7622c109ae6f9ff7ffafc3bffac4856700d39ca51d8689ecaa617a596bd'
_CALL_HASH = 'e6cd96b1da9eabec6fd3de307db8ecae5cf4262b5fed4e5c0e535b17637219be'
_TAKE_HASH = 'a82a66e17a9efe0adcff747f241711c4fe44c7f41def3514aef781e23273a662'


def _reviewed_client():
    """Refuse private binding hooks outside the exact reviewed local build."""
    from rclpy.client import Client
    from rclpy.executors import Executor
    from rclpy.impl.implementation_singleton import rclpy_implementation
    functions = ((Client.call_async, _CALL_HASH), (Executor._take_client, _TAKE_HASH))
    if (version('rclpy') != '7.1.12' or
            hashlib.sha256(Path(rclpy_implementation.__file__).read_bytes()).hexdigest() !=
            _NATIVE_HASH or any(
                getattr(fn, '__code__', None) is None or
                hashlib.sha256(inspect.getsource(fn.__code__).encode()).hexdigest() != h
                for fn, h in functions)):
        raise RuntimeError('Native diagnostic requires the reviewed local rclpy 7.1.12 build')
    return rclpy_implementation.Client


def install_native_client_trace(endpoint, context_getter, root):
    """Install only from the verified WAIT wrapper and its worker-local CID."""
    global _installed
    if not endpoint:
        return False
    if _installed is not None:
        if _installed != endpoint:
            raise RuntimeError('Cannot retarget an installed native diagnostic')
        return True
    native = _reviewed_client()
    original_send, original_take = native.send_request, native.take_response
    events, guard = [], Lock()
    byte_count = dropped = metadata_errors = 0
    budget_exceeded = final_attempted = False
    record_limit, byte_limit = 24000, 8388608

    def identity(client, with_cid=False):
        """Copy primitive identity only while the upstream handle is acquired."""
        nonlocal metadata_errors
        try:
            if client.service_name != endpoint:
                return None
            # Both values may be reused after destruction. Match to the most
            # recent send within that lifetime/time window, never globally.
            item = {'clientKey': f'{int(client.pointer):x}:{id(client):x}'}
            if with_cid:
                context = context_getter()
                cid = context.get('cid') if isinstance(context, dict) else None
                item['cid'] = cid[:128] if isinstance(cid, str) else None
            return item
        except Exception:
            metadata_errors += 1
            return None

    def retain(phase, item, **values):
        """Append bounded JSON metadata without waiting for another observer."""
        nonlocal byte_count, dropped, metadata_errors, budget_exceeded
        if item is None:
            return
        try:
            record = {'phase': phase, 'wall': time.time(), 'monotonic': time.monotonic(),
                      'thread': get_ident(), **item, **values}
            encoded = json.dumps(record, ensure_ascii=True, separators=(',', ':'))
            size = len(encoded.encode('utf-8')) + 1
            if not guard.acquire(blocking=False):
                dropped += 1
                return
            try:
                if len(events) >= record_limit or byte_count + size > byte_limit:
                    budget_exceeded = True
                    dropped += 1
                    return
                events.append(encoded)
                byte_count += size
            finally:
                guard.release()
        except Exception:
            metadata_errors += 1

    def send_request(self, *args, **kwargs):
        nonlocal metadata_errors
        item = identity(self, with_cid=True)
        retain('sendEnter', item)
        try:
            sequence = original_send(self, *args, **kwargs)
        except Exception as error:
            retain('sendError', item, error=type(error).__name__)
            raise
        try:
            retain('sendReturn', item, sequence=int(sequence))
        except Exception:
            metadata_errors += 1
        return sequence

    def take_response(self, *args, **kwargs):
        nonlocal metadata_errors
        item = identity(self)
        retain('takeEnter', item)
        try:
            result = original_take(self, *args, **kwargs)
        except Exception as error:
            retain('takeError', item, error=type(error).__name__)
            raise
        if item is not None:
            try:
                header = result[0]
                sequence = int(header.request_id.sequence_number) if header is not None else None
                # FastDDS copies its SampleInfo write/history-insertion times
                # into these RMW fields. Preserve nanoseconds as decimal text:
                # epoch ns exceeds JavaScript's exact integer range. Zero or
                # unsupported clocks must not be treated as a delivery time.
                retain('takeReturn', item, headerPresent=header is not None,
                       sequence=sequence,
                       sourceTimestampNs=str(int(header.source_timestamp))
                       if header is not None else None,
                       receivedTimestampNs=str(int(header.received_timestamp))
                       if header is not None else None)
            except Exception:
                metadata_errors += 1
        return result

    def final_snapshot():
        """Attempt one normal-exit write; parent guard bounds filesystem hangs."""
        nonlocal final_attempted
        if final_attempted:
            return
        final_attempted = True
        try:
            acquired = guard.acquire(timeout=.05)
            try:
                snapshot = list(events) if acquired else []
            finally:
                if acquired:
                    guard.release()
            record = {'endpoint': endpoint, 'events': [json.loads(e) for e in snapshot],
                      'snapshotAvailable': acquired, 'recordLimit': record_limit,
                      'byteLimit': byte_limit, 'encodedEventBytes': byte_count,
                      'droppedRecords': dropped, 'metadataErrors': metadata_errors,
                      'budgetExceeded': budget_exceeded, 'nativeBinarySha256': _NATIVE_HASH,
                      'sampling': ('send/take entry and return, not DDS wire arrival; '
                                   'clientKey may be reused after destruction')}
            with (Path(root) / f'service-native-{os.getpid()}.json').open(
                    'x', encoding='utf-8') as output:
                json.dump(record, output, ensure_ascii=True, separators=(',', ':'))
                output.write('\n')
        except Exception as error:
            try:
                print(f'native final diagnostic write failed: {type(error).__name__}',
                      file=sys.stderr)
            except Exception:
                pass

    native.send_request, native.take_response = send_request, take_response
    atexit.register(final_snapshot)
    _installed = endpoint
    return True


def read_native_trace(root, endpoint):
    """Reject missing or incomplete diagnostic evidence after process cleanup."""
    paths = list(Path(root).glob('service-native-*.json'))
    if len(paths) != 1 or paths[0].stat().st_size > 8388608 + 4096:
        raise ValueError('Expected one bounded normal-exit native snapshot')
    record = json.loads(paths[0].read_text())
    if (record['endpoint'] != endpoint or not record['snapshotAvailable'] or
            record['droppedRecords'] or record['metadataErrors'] or record['budgetExceeded'] or
            not 0 < len(record['events']) <= 24000 or record['encodedEventBytes'] > 8388608):
        raise ValueError('Native snapshot has missing, dropped or invalid metadata')
    return {key: record[key] for key in (
        'endpoint', 'snapshotAvailable', 'encodedEventBytes', 'droppedRecords', 'metadataErrors',
        'budgetExceeded')} | {'records': len(record['events']), 'file': paths[0].name}
