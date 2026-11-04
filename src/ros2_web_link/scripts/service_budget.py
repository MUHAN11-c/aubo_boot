# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""
Bound bridge Service work independently of optional DDS client caching.

The upstream capability/conversion helpers keep wire and permission semantics.
Admission happens before starting a worker. Uncached entity creation/destruction
uses public executor.create_task, as in rosbridge 2.7.1's executor_helpers.
An expired but unconsumed task keeps its reservation: a stopped executor cannot
turn repeated timeouts into an unbounded queue of stale creation requests.
"""

from contextvars import ContextVar
from dataclasses import dataclass, field
import math
from threading import Event, Lock, Thread
import time
from typing import Any
import weakref

from rclpy.callback_groups import ReentrantCallbackGroup
from rosbridge_library.internal import services


@dataclass(eq=False)
class _Call:
    """One reservation, retained through queued entity cleanup."""

    ready: Event = field(default_factory=Event)
    response: Event = field(default_factory=Event)
    client: Any = None
    future: Any = None
    creation_error: str | None = None
    abandoned: bool = False
    finished: bool = False
    jobs: int = 0


class ServiceCallBudget:
    """One Node's fail-fast Service admission, across all WebSocket clients."""

    def __init__(self, node, maximum=128, creation_timeout=5.0):
        if type(maximum) is not int or not 1 <= maximum <= 4096:
            raise ValueError('Service call limit must be an integer in 1..4096')
        if (isinstance(creation_timeout, bool) or not math.isfinite(creation_timeout)
                or not 0 < creation_timeout <= 60):
            raise ValueError('Service creation deadline must be finite in (0,60]')
        self._owner = weakref.ref(node)
        self._maximum = maximum
        self._creation_timeout = creation_timeout
        self._guard = Lock()
        self._calls = set()
        self._closed = False
        self._high = self._rejected = self._cleanup_errors = 0
        self._current = ContextVar('ros2_web_service_reservation', default=None)

    def _admit(self):
        with self._guard:
            if self._closed:
                raise RuntimeError('ROS node closing')
            if len(self._calls) >= self._maximum:
                self._rejected += 1
                raise RuntimeError('Bridge Service call budget limit reached; request not queued')
            call = _Call()
            self._calls.add(call)
            self._high = max(self._high, len(self._calls))
            return call

    def _release(self, call):
        """Caller holds the guard; queued work still consumes admission."""
        if call.finished and call.jobs == 0:
            self._calls.discard(call)

    def _retire(self, call):
        with self._guard:
            client, call.client = call.client, None
            if client is None:
                return
            if self._closed:
                # ManagedRosbridgeNode immediately calls Node.destroy_node;
                # its normal ownership cleanup handles every remaining client.
                return
            call.jobs += 1

        def destroy():
            confirmed = False
            try:
                node = self._owner()
                if node is not None and not self._closed:
                    confirmed = node.destroy_client(client)
                elif self._closed:
                    confirmed = True  # Node's normal cleanup owns this handle.
            except Exception:
                pass  # Quarantine rather than grow entities or kill the owner.
            finally:
                with self._guard:
                    if confirmed:
                        call.jobs -= 1
                        self._release(call)
                    else:
                        self._cleanup_errors += 1

        node = self._owner()
        if node is None or node.executor is None:
            destroy()
        else:
            # If wake raises after insertion, keep the slot until consumption.
            try:
                node.executor.create_task(destroy)
            except Exception:
                with self._guard:
                    self._cleanup_errors += 1

    def _finish(self, call):
        try:
            self._retire(call)
        finally:
            with self._guard:
                call.finished = True
                self._release(call)

    def run(self, operation):
        """Also bound internal synchronous callers without double admission."""
        if self._current.get() is not None:
            return operation(self._current.get())
        call = self._admit()
        token = self._current.set(call)
        try:
            return operation(call)
        finally:
            self._current.reset(token)
            self._finish(call)

    def submit(self, operation, failure):
        """Reject before Thread.start; never create an executor/worker queue."""
        call = self._admit()

        def execute():
            token = self._current.set(call)
            try:
                operation()
            except Exception as error:
                failure(error)
            finally:
                self._current.reset(token)
                self._finish(call)

        try:
            Thread(target=execute, name='ros2-web-service', daemon=True).start()
        except BaseException:
            self._finish(call)
            raise

    def _create(self, call, node, service_type, service):
        if node.executor is None:
            raise RuntimeError('Bridge Service requires an attached executor')
        deadline = time.monotonic()+self._creation_timeout
        with self._guard:
            if self._closed:
                raise RuntimeError('ROS node closing')
            call.jobs += 1

        def create():
            confirmed = True
            entered_factory = False
            try:
                with self._guard:
                    if self._closed or call.abandoned or time.monotonic() >= deadline:
                        call.creation_error = 'Service client creation deadline exceeded'
                        return
                owner = self._owner()
                if owner is None:
                    raise RuntimeError('ROS node unavailable')
                entered_factory = True
                client = owner.create_client(service_type, service,
                                             callback_group=ReentrantCallbackGroup())
                with self._guard:
                    late = self._closed or call.abandoned or time.monotonic() >= deadline
                    if not late:
                        call.client = client
                if late:
                    # Creation already entered native code when the wait expired.
                    # Dispose on this owner task before releasing its reservation.
                    confirmed = owner.destroy_client(client)
            except Exception as error:
                call.creation_error = str(error)[:1024]
                # Node.create_client can register a client and then raise while
                # waking its executor, before assigning our local variable.
                # Public APIs cannot prove retirement, so fail closed.
                if entered_factory:
                    confirmed = False
            finally:
                with self._guard:
                    if confirmed:
                        call.jobs -= 1
                        self._release(call)
                    else:
                        self._cleanup_errors += 1
                call.ready.set()

        try:
            node.executor.create_task(create)
        except BaseException:
            with self._guard:
                call.abandoned = True
            raise  # A possibly enqueued task retains its reservation until ack.
        call.ready.wait(max(0, deadline-time.monotonic()))
        with self._guard:
            if self._closed or time.monotonic() >= deadline or call.client is None:
                call.abandoned = True
                raise RuntimeError(
                    call.creation_error or 'Service client creation deadline exceeded')
            return call.client

    def call_uncached(self, call, service, args=None, server_ready_timeout=1.0,
                      server_response_timeout=5.0):
        """
        Use upstream types/conversion and a per-call public rclpy Client.

        As upstream, a nonpositive response timeout means no response deadline;
        admission still bounds such calls and Node shutdown wakes their waits.
        Creation has an independent finite monotonic deployment deadline.
        """
        if not math.isfinite(server_response_timeout) or not math.isfinite(server_ready_timeout):
            raise ValueError('Service timeout must be finite')
        node = self._owner()
        if node is None:
            raise RuntimeError('ROS node unavailable')
        service = node.resolve_service_name(service)
        offered = dict(node.get_service_names_and_types()).get(service)
        if not offered:
            raise services.InvalidServiceException(service)
        service_type = services.get_service_class(offered[0])
        request = services.get_service_request_instance(offered[0])
        services.args_to_service_request_instance(request, args)
        client = self._create(call, node, service_type, service)
        if not client.wait_for_service(server_ready_timeout):
            raise services.InvalidServiceException(service)
        try:
            with self._guard:
                if self._closed:
                    raise RuntimeError('ROS node closing')
                # Construct inside upstream CallService's diagnostic context;
                # normal operation is just threading.Event, without file IO.
                call.response = services.Event()
            deadline = (time.monotonic()+server_response_timeout
                        if server_response_timeout > 0 else None)
            future = call.future = client.call_async(request)
            future.add_done_callback(lambda _: call.response.set())
            call.response.wait(max(0, deadline-time.monotonic()) if deadline else None)
            if self._closed:
                raise RuntimeError('ROS node closing')
            if deadline is not None and time.monotonic() >= deadline:
                raise RuntimeError('Timeout exceeded while waiting for service response')
            result = future.result()
            if result is None:
                raise RuntimeError('Service call exception: '+str(future.exception()))
            return services.extract_values(result)
        finally:
            if call.future is not None:
                if not call.future.done():
                    call.future.cancel()
                client.remove_pending_request(call.future)
                call.future = None

    def close(self):
        """Wake workers immediately before the owning Node's normal cleanup."""
        with self._guard:
            self._closed = True
            calls = list(self._calls)
        for call in calls:
            call.ready.set()
            call.response.set()

    def stats(self):
        """Logical reservations; this is not a native allocator memory bound."""
        with self._guard:
            return {'active': len(self._calls), 'maximum': self._maximum,
                    'highWater': self._high, 'rejected': self._rejected,
                    'cleanupErrors': self._cleanup_errors,
                    'pendingOwnerJobs': sum(call.jobs for call in self._calls),
                    'closed': self._closed}
