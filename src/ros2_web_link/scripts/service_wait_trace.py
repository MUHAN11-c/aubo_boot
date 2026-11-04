# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""
Opt-in, version-checked observation of upstream service wait timeouts.

No added ROS entities, Future callbacks, retry or timeout changes. Inspect the
upstream caller's Future only after its Event.wait returns False and before the
original caller cancels it. The optional late-reader mode retains at most three
timed-out Clients for two seconds after the verdict; ordinary WAIT mode does not.
Never retain frames, Futures or payloads in diagnostic records.
The measurements are sequential observations, not an atomic execution trace.
"""

import atexit
from collections import deque
import hashlib
from importlib.metadata import version
import inspect
import json
import math
import os
from pathlib import Path
import re
import sys
from threading import local, Lock
import time

_installed = None
_INTERNAL_HASH = '6671ca5f152f848a14bd2ed52521b1ea868ff9204dd3fa169ce0b2492425f697'
_CAPABILITY_HASH = 'fa1e1f4921e5b1d8aa7ba986e1e7a03ee76b0644531c2a72fafb0ea402c80a6e'
_DESTROY_HASH = '6b046376b027e2a3de5e5a513673bb9a830ee3cd0c1a4d8d1b708d50afd8f846'
_DISPATCH_HASH = '00de69ff113b2a5c6d30243f22c1d315b8556378db54f00b98dc8dc653759a9d'
_POOL_RESPONSE_HASH = '85e22cf09f9ac4f9565d04348a06d576bf729d4c354a334169608062cfcc953b'


def install_service_wait_trace():
    """Confine diagnostic hooks to this process and one exact Web endpoint."""
    global _installed
    endpoint = os.environ.get('WEB_LINK_WAIT_SERVICE', '')
    native_endpoint = os.environ.get('WEB_LINK_NATIVE_SERVICE', '')
    late_endpoint = os.environ.get('WEB_LINK_LATE_CLIENT_SERVICE', '')
    if native_endpoint and native_endpoint != endpoint:
        raise ValueError('Native diagnostic requires the same explicit WAIT endpoint')
    if late_endpoint and (late_endpoint != endpoint or not native_endpoint):
        raise ValueError('Late-reader diagnostic requires the same native and WAIT endpoint')
    if not endpoint:
        return False
    if (len(endpoint) > 256 or re.fullmatch(
            r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', endpoint) is None):
        raise ValueError('WEB_LINK_WAIT_SERVICE must be an absolute service name <=256 characters')
    if _installed is not None:
        if _installed != (endpoint, bool(native_endpoint), bool(late_endpoint)):
            raise RuntimeError('Cannot retarget or change an installed wait diagnostic mode')
        return True

    from rclpy.task import Future
    from rosbridge_library.capabilities.call_service import CallService
    from rosbridge_library.internal import services
    internal = services.call_service
    upstream_internal = getattr(internal, '_web_link_original', internal)
    response_method = getattr(internal, '_web_link_response', None)
    uncached_method = getattr(internal, '_web_link_uncached_response', None)
    expected = ((upstream_internal, _INTERNAL_HASH),
                (CallService.call_service, _CAPABILITY_HASH))
    if response_method is not None:
        expected += ((internal, _DISPATCH_HASH), (response_method, _POOL_RESPONSE_HASH))
    if uncached_method is not None:
        expected += ((uncached_method,
                      'f47126c68c70ea2d2a48d9497aa6c3e2681c407a3ce0fe0fa7414ae65abdf974'),)
    if late_endpoint:
        expected += ((services._destroy_client_async, _DESTROY_HASH),)
    if (version('rosbridge-library') != '2.7.1' or any(
            getattr(method, '__code__', None) is None or
            hashlib.sha256(inspect.getsource(method.__code__).encode()).hexdigest() != digest
            for method, digest in expected)):
        raise RuntimeError('Wait diagnostic requires the reviewed rosbridge 2.7.1 source layout')
    destination = os.environ.get('WEB_LINK_EVIDENCE', '')
    if not destination or not Path(destination).is_dir():
        raise ValueError('Wait diagnostic requires an existing WEB_LINK_EVIDENCE directory')
    root = Path(destination)
    output = (root / f'service-waits-{os.getpid()}.jsonl').open('x', encoding='utf-8')
    atexit.register(output.close)
    original, event_class = CallService.call_service, services.Event
    service_codes = {upstream_internal.__code__: 'upstream'}
    if response_method is not None:
        service_codes[response_method.__code__] = 'managed-cache'
    if uncached_method is not None:
        service_codes[uncached_method.__code__] = 'managed-uncached'
    active, ring, guard = local(), deque(maxlen=64), Lock()
    if native_endpoint:
        from service_native_trace import install_native_client_trace
        install_native_client_trace(
            native_endpoint, lambda: getattr(active, 'context', None), root)
    late_trace = None
    if late_endpoint:
        from rclpy.client import Client
        from service_late_client import LateClientTrace
        late_trace = LateClientTrace(root, services._destroy_client_async)
        services._destroy_client_async = late_trace.destroy
    written = 0
    final_attempted = False

    def final_snapshot():
        """Keep successful-call timing too, with one bounded normal-exit write."""
        nonlocal final_attempted
        if final_attempted:
            return
        final_attempted = True
        try:
            # A daemon worker may still own the lock during shutdown. Bound this
            # lock wait; filesystem IO still relies on the parent's stop guard.
            acquired = guard.acquire(timeout=.05)
            try:
                recent = list(ring) if acquired else []
            finally:
                if acquired:
                    guard.release()
            record = {'recent': recent, 'snapshotAvailable': acquired,
                      'clockAtSnapshot': {'wall': time.time(), 'monotonic': time.monotonic()},
                      'sampling': ('completed upstream calls; sequential timestamps, '
                                   'not wire delivery')}
            encoded = json.dumps(record, ensure_ascii=True)
            while len(encoded) + 1 > 131072 and recent:
                del recent[:max(1, len(recent) // 2)]
                encoded = json.dumps(record, ensure_ascii=True)
            with (root / f'service-wait-final-{os.getpid()}.json').open(
                    'x', encoding='utf-8') as final:
                final.write(encoded + '\n')
        except Exception as error:
            try:
                print(f'wait final diagnostic write failed: {type(error).__name__}',
                      file=sys.stderr)
            except Exception:
                pass

    def retain(context):
        """Write bounded metadata after upstream cleanup, never inside Event.wait."""
        nonlocal written
        context['callReturnedAt'] = time.monotonic()
        with guard:
            snapshot = dict(context)
            ring.append(snapshot)
            if snapshot.get('waitReturned') is not False or written >= 8:
                return
            recent = list(ring)
            record = {'trigger': snapshot, 'recent': recent,
                      'sampling': 'sequential Future/Event state, before upstream cancellation'}
            encoded = json.dumps(record, ensure_ascii=True)
            while len(encoded) + 1 > 131072 and len(recent) > 1:
                del recent[:max(1, len(recent) // 2)]
                encoded = json.dumps(record, ensure_ascii=True)
            # Count attempts before IO: a failed flush may already have written
            # the record. Disk failure must not permit an unbounded retry log.
            written += 1
            output.write(encoded + '\n')
            output.flush()

    class WaitEvent(event_class):
        """Preserve threading.Event behavior and timestamp notification entry/return."""

        def __init__(self):
            super().__init__()
            self._wait_context = getattr(active, 'context', None)

        def set(self):  # noqa: A003 - preserve the public threading.Event method.
            context = self._wait_context
            if context is not None:
                context['setEnteredAt'] = time.monotonic()
            super().set()
            if context is not None:
                context['setReturnedAt'] = time.monotonic()

        def wait(self, timeout=None):
            context = self._wait_context
            if context is not None:
                context['waitEnteredAt'] = time.monotonic()
            result = super().wait(timeout)
            if context is None:
                return result
            context['waitReturnedAt'] = time.monotonic()
            context['waitReturned'] = result
            if result:
                return result
            frame = inspect.currentframe()
            caller = future = client = None
            try:
                caller = frame.f_back if frame is not None else None
                if caller is None or caller.f_code not in service_codes:
                    context['samplingError'] = 'Unexpected Event.wait caller; not classified'
                    return result
                context['responseCaller'] = service_codes[caller.f_code]
                future = caller.f_locals.get('future')
                if not isinstance(future, Future):
                    context['samplingError'] = 'Expected native rclpy Future; not classified'
                    return result
                context['sampleStartedAt'] = time.monotonic()
                context['waitState'] = {
                    'futureDone': future.done(), 'futureCancelled': future.cancelled(),
                    'eventSet': self.is_set(),
                }
                context['sampleFinishedAt'] = time.monotonic()
                if late_trace is not None and not context['waitState']['futureDone']:
                    client = caller.f_locals.get('client')
                    context['lateReaderArmed'] = late_trace.arm(
                        id(client) if isinstance(client, Client) else None, context['cid'])
            finally:
                # A retained frame would also retain request data and native
                # entities. Only primitive metadata may reach the bounded ring.
                del frame, caller, future, client
            return result

    def call_service(self, message):
        if not isinstance(message, dict) or message.get('service') != endpoint:
            return original(self, message)
        previous = getattr(active, 'context', None)
        cid = message.get('id')
        context = {'cid': cid[:128] if isinstance(cid, str) else None,
                   'service': endpoint, 'callEnteredAt': time.monotonic(),
                   'callEnteredWall': time.time()}
        timeout = message.get('timeout')
        if isinstance(timeout, (int, float)) and not isinstance(timeout, bool):
            try:
                if math.isfinite(timeout):
                    context['requestedTimeoutSeconds'] = timeout
            except OverflowError:
                pass
        active.context = context
        try:
            return original(self, message)
        finally:
            active.context = previous
            try:
                retain(context)
            except Exception as error:
                # A diagnostic sink failure must not change the ROS result.
                try:
                    print(f'wait diagnostic write failed: {type(error).__name__}', file=sys.stderr)
                except Exception:
                    pass

    (root / f'service-wait-ready-{os.getpid()}.json').write_text(json.dumps({
        'pid': os.getpid(), 'endpoint': endpoint, 'ringLimit': 64, 'failureRecordLimit': 8,
        'recordByteLimit': 131072, 'internalSourceSha256': _INTERNAL_HASH,
        'capabilitySourceSha256': _CAPABILITY_HASH,
        'managedDispatcherEnabled': response_method is not None,
        'responseSourceSha256': _POOL_RESPONSE_HASH if response_method else _INTERNAL_HASH,
        'dispatcherSourceSha256': _DISPATCH_HASH if response_method else None,
        'finalSnapshotFile': f'service-wait-final-{os.getpid()}.json',
        'finalSnapshotAttemptLimit': 1,
        'nativeBoundaryEnabled': bool(native_endpoint),
        'lateReaderRetentionEnabled': bool(late_endpoint),
    }, indent=2))
    services.Event, CallService.call_service = WaitEvent, call_service
    atexit.register(final_snapshot)
    _installed = (endpoint, bool(native_endpoint), bool(late_endpoint))
    return True
