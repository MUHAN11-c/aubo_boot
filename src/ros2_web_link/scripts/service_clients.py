# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""
Private Node-owned Service client pool for the reviewed Jazzy bridge.

Protocol parsing, permission checks and message conversion remain rosbridge.
This module uses its existing type/conversion helpers and public rclpy APIs.
Native and CPU contracts are recorded under service-client-design (I60).
The bridge adapter must validate the exact owner-executor layout before use.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from threading import Event, Lock
import time
from typing import Any
import weakref

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import NodeNameNonExistentError
from rosbridge_library.internal import services


def _failure_text(error: BaseException) -> str:
    """Bound retained native error text; never retain its traceback object."""
    text = str(error)
    return text[:1021] + '...' if len(text) > 1024 else text


@dataclass(eq=False)
class _Entry:
    """One reservation; its client reference does not prolong Node ownership."""

    ready: Event = field(default_factory=Event)
    client_ref: Any = None
    # Never cache exception objects: their traceback can retain Node/payload.
    error: str | None = None
    victim_ref: Any = None
    victim_key: tuple[str, type] | None = None
    quarantined: bool = False
    retirement_pending: bool = False
    late_retirement: bool = False
    creating: bool = True
    queued: bool = False
    started: bool = False
    deadline: float = 0.0
    retiring_error: bool = False
    retyping: bool = False
    users: int = 0


@dataclass(eq=False)
class _Request:
    """Only a live call owns its response Future and notification Event."""

    done: Event = field(default_factory=lambda: services.Event())
    entry: _Entry | None = None
    client: Any = None
    future: Any = None
    aborted: bool = False


class ServiceClientPool:
    """Bound client reservations and active calls without owning a new thread."""

    def __init__(self, node: Any, cache_size: int = 64,
                 max_in_flight: int = 128, creation_timeout: float = 5.0):
        if type(cache_size) is not int or not 1 <= cache_size <= 1024:
            raise ValueError('cache_size must be an integer in 1..1024')
        if type(max_in_flight) is not int or not 1 <= max_in_flight <= 4096:
            raise ValueError('max_in_flight must be an integer in 1..4096')
        if (isinstance(creation_timeout, bool) or
                not math.isfinite(creation_timeout) or not 0 < creation_timeout <= 60):
            raise ValueError('creation_timeout must be finite in (0,60]')
        if node.executor is None:
            raise ValueError('Service client pool requires an attached executor')
        self._owner = weakref.ref(node)
        self._executor = weakref.ref(node.executor)
        self._cache_size, self._max_in_flight = cache_size, max_in_flight
        self._creation_timeout = float(creation_timeout)
        self._guard = Lock()
        self._entries: dict[tuple[str, type], _Entry] = {}
        self._requests: set[_Request] = set()
        self._queued_entries: set[_Entry] = set()
        self._closed = False
        self._created = self._evicted = self._late_discarded = self._errors = 0
        self._retired = self._rejected_calls = self._rejected_cache = 0
        self._retyped = self._unconfirmed_creations = 0
        self._native_count = self._native_high = self._entry_high = self._active_high = 0

    def _owner_executor(self, node: Any) -> Any:
        """One stable owner executor is part of this private pool's contract."""
        executor = self._executor()
        if executor is None or node.executor is not executor:
            raise RuntimeError('Service client owner executor changed')
        return executor

    def _prune_empty(self, key: tuple[str, type], entry: _Entry) -> None:
        """Release only a settled empty slot; caller must hold the state guard."""
        client = entry.client_ref() if entry.client_ref is not None else None
        victim = entry.victim_ref() if entry.victim_ref is not None else None
        if (self._entries.get(key) is entry and entry.users == 0 and
                not entry.creating and not entry.queued and not entry.retirement_pending and
                not entry.quarantined and
                entry.error is not None and client is None and victim is None):
            self._entries.pop(key)

    def _settle(self, key: tuple[str, type], entry: _Entry) -> None:
        """Publish one terminal creation result without releasing pending native slots."""
        with self._guard:
            entry.creating = False
            if entry.client_ref is None and entry.error is None:
                entry.error = 'ROS node closing'
            self._prune_empty(key, entry)
        entry.ready.set()

    def _queue_retirement_ack(self, node: Any, key: tuple[str, type], entry: _Entry) -> None:
        """
        Consume the owner's next task dispatch before reusing a native slot.

        This is a reviewed installed-executor-layout boundary, not a public FIFO
        promise. Task2 must gate supported executors before enabling this cache.
        No Task argument owns Node or Client; expired/closed tickets still ack.
        """
        executor = self._owner_executor(node)
        with self._guard:
            entry.queued = True
            self._queued_entries.add(entry)
        try:
            executor.create_task(self._after_retirement, key, entry)
        except BaseException as error:
            # Waking can fail after insertion. Keep this possibly accepted ticket
            # and its slot until actual consumption, even after local close.
            with self._guard:
                entry.error = _failure_text(error)
                self._errors += 1
            entry.ready.set()

    def _retire(self, node: Any, key: tuple[str, type], entry: _Entry, client: Any) -> bool:
        """Exclude late worker takes, request destruction, then defer owner ack."""
        self._owner_executor(node)
        group = client.callback_group
        if not group.beginning_execution(client):
            # No native operation occurred, so an explicit later request may
            # retry this same reserved victim after the callback has finished.
            with self._guard:
                entry.error = 'Service client retirement busy'
            return False
        try:
            if not node.destroy_client(client):
                raise RuntimeError('Client retirement could not be confirmed')
            with self._guard:
                entry.retirement_pending = True
        finally:
            # Destroyable.enter rejects new borrowers after destruction was
            # requested. Existing worker takes were excluded by the group; only
            # the owner's pre-existing WaitSet borrow remains for the next ack.
            group.ending_execution(client)
        self._queue_retirement_ack(node, key, entry)
        return True

    def _after_retirement(self, key: tuple[str, type], entry: _Entry) -> None:
        """Acknowledge native retirement outside the previous owner WaitSet."""
        node = self._owner()
        try:
            if node is None:
                raise RuntimeError('ROS node unavailable')
            self._owner_executor(node)
        except BaseException as error:
            with self._guard:
                self._queued_entries.discard(entry)
                entry.queued = False
                entry.quarantined = True
                entry.error = _failure_text(error)
                self._errors += 1
            self._settle(key, entry)
            return
        with self._guard:
            self._queued_entries.discard(entry)
            entry.queued = False
            if not entry.retirement_pending:
                return
            entry.retirement_pending = False
            self._native_count -= 1
            if entry.late_retirement:
                pass  # lateDiscarded counts results, not replacement evictions.
            elif entry.retiring_error:
                self._retired += 1
            elif entry.retyping:
                self._retyped += 1
            else:
                self._evicted += 1
            entry.victim_ref = entry.victim_key = None
            resume = not self._closed and entry.error is None and not entry.late_retirement
        if resume:
            self._create(key, entry)
        else:
            self._settle(key, entry)

    def _capture_factory_failure(self, node: Any, key: tuple[str, type],
                                 entry: _Entry, group: Any) -> None:
        """
        Never release a slot based on an unreturned public factory result.

        Node.create_client appends before its final executor wake. A unique
        standard group identifies a registered-but-unreturned Client through the
        public clients iterator. If inspection cannot establish ownership, the
        slot remains quarantined anyway; zero returned values is not evidence of
        atomic native failure. Normal Node shutdown is the recovery boundary.
        """
        registered = None
        try:
            matches = [client for client in node.clients if client.callback_group is group]
            if len(matches) == 1:
                registered = matches[0]
        except BaseException:
            pass  # Preserve the original factory error and the unknown slot.
        with self._guard:
            entry.quarantined = True
            self._unconfirmed_creations += 1
            if registered is not None:
                entry.victim_ref = weakref.ref(registered)
                entry.victim_key = key
                self._created += 1
                self._native_count += 1
                self._native_high = max(self._native_high, self._native_count)

    def _create(self, key: tuple[str, type], entry: _Entry) -> None:
        """Run native operations in the owner's executor, always outside guard."""
        client = None
        deferred = False
        try:
            with self._guard:
                self._queued_entries.discard(entry)
                entry.queued = False
                if (self._closed or entry.error is not None or
                        self._entries.get(key) is not entry):
                    return
                if time.monotonic() >= entry.deadline:
                    entry.error = 'Service client creation timed out'
                    return
                entry.started = True
            node = self._owner()
            if node is None:
                raise RuntimeError('ROS node unavailable')
            self._owner_executor(node)
            victim = entry.victim_ref() if entry.victim_ref is not None else None
            if victim is not None:
                deferred = self._retire(node, key, entry, victim)
                return
            with self._guard:
                if self._closed or entry.error is not None:
                    return
                if time.monotonic() >= entry.deadline:
                    entry.error = 'Service client creation timed out'
                    return
            name, service_type = key
            group = MutuallyExclusiveCallbackGroup()
            try:
                client = node.create_client(service_type, name, callback_group=group)
            except BaseException:
                self._capture_factory_failure(node, key, entry, group)
                raise
            with self._guard:
                self._created += 1
                self._native_count += 1
                self._native_high = max(self._native_high, self._native_count)
                if time.monotonic() >= entry.deadline and entry.error is None:
                    entry.error = 'Service client creation timed out'
                if self._closed or entry.error is not None:
                    self._late_discarded += 1
                    entry.victim_ref = weakref.ref(client)
                    entry.victim_key = key
                    entry.late_retirement = True
                    discard = True
                else:
                    entry.client_ref = weakref.ref(client)
                    discard = False
            if discard:
                deferred = self._retire(node, key, entry, client)
        except BaseException as error:
            with self._guard:
                if entry.victim_ref is not None or client is not None:
                    entry.quarantined = True
                entry.error = _failure_text(error)
                self._errors += 1
        finally:
            if not deferred:
                self._settle(key, entry)

    def _admit(self, request: _Request) -> None:
        """Bound admitted work before graph lookup or request expansion."""
        with self._guard:
            if self._closed:
                raise RuntimeError('ROS node closing')
            if len(self._requests) >= self._max_in_flight:
                self._rejected_calls += 1
                raise RuntimeError('Service client calls busy')
            self._requests.add(request)
            self._active_high = max(self._active_high, len(self._requests))

    def _acquire(self, node: Any, key: tuple[str, type], request: _Request) -> Any:
        """Reserve one factory and keep unconsumed tickets inside capacity."""
        with self._guard:
            if self._closed:
                raise RuntimeError('ROS node closing')
            entry = self._entries.get(key)
            replacement = victim_key = None
            retiring_error = False
            conflicts = [(k, e) for k, e in self._entries.items() if k != key and
                         (k[0] == key[0] or
                          (e.victim_key is not None and e.victim_key[0] == key[0]))]
            if conflicts:
                # A repurposed LRU ticket still owns its old endpoint until the
                # native retirement actually completes. Do not hide that owner.
                conflict_key, conflict = conflicts[0]
                if (len(conflicts) != 1 or entry is not None or conflict.users or
                        conflict.creating or conflict.queued or
                        conflict.retirement_pending or conflict.quarantined):
                    self._rejected_cache += 1
                    different_type = (conflict_key[0] == key[0] and
                                      conflict_key[1] is not key[1]) or (
                        conflict.victim_key is not None and
                        conflict.victim_key[0] == key[0] and
                        conflict.victim_key[1] is not key[1])
                    raise RuntimeError('Service client type change busy' if different_type
                                       else 'Service client retirement busy')
                self._entries.pop(conflict_key)
                replacement = conflict.client_ref or conflict.victim_ref
                victim_key = conflict_key if conflict.client_ref else conflict.victim_key
                retiring_error = conflict.error is not None
            if (entry is not None and entry.error is not None and not entry.creating
                    and not entry.queued and not entry.retirement_pending
                    and not entry.quarantined and entry.users == 0):
                # Explicit requests can replace a settled failure; an unconsumed
                # executor ticket is not a settled failure and cannot be retried.
                replacement = entry.client_ref or entry.victim_ref
                victim_key = key if entry.client_ref else entry.victim_key
                retiring_error = replacement is not None
                self._entries.pop(key)
                entry = None
            create = entry is None
            if create:
                victim_ref = replacement
                if len(self._entries) >= self._cache_size:
                    victim = next(((k, e) for k, e in self._entries.items()
                                   if e.users == 0 and not e.creating and not e.queued
                                   and not e.retirement_pending and not e.quarantined), None)
                    if victim is None:
                        self._rejected_cache += 1
                        raise RuntimeError('Service client cache full')
                    previous_key, victim_entry = victim
                    self._entries.pop(previous_key)
                    victim_ref = victim_entry.client_ref or victim_entry.victim_ref
                    victim_key = (previous_key if victim_entry.client_ref
                                  else victim_entry.victim_key)
                    retiring_error = victim_entry.error is not None
                retyping = (victim_key is not None and victim_key[0] == key[0]
                            and victim_key[1] is not key[1])
                entry = _Entry(victim_ref=victim_ref, victim_key=victim_key,
                               deadline=time.monotonic() + self._creation_timeout,
                               retiring_error=retiring_error, retyping=retyping)
                self._entries[key] = entry
                self._entry_high = max(self._entry_high, len(self._entries))
            else:
                self._entries.pop(key)
                self._entries[key] = entry
            entry.users += 1
            request.entry = entry
        if create:
            with self._guard:
                submit = not self._closed and entry.error is None
                if submit:
                    entry.queued = True
                    self._queued_entries.add(entry)
                else:
                    entry.creating = False
            if submit:
                try:
                    executor = self._owner_executor(node)
                    executor.create_task(self._create, key, entry)
                except BaseException as error:
                    # Waking may fail after insertion; do not release its slot
                    # without actual task consumption or final Node teardown.
                    with self._guard:
                        entry.error = _failure_text(error)
                        self._errors += 1
                    entry.ready.set()
            else:
                entry.ready.set()
        remaining = max(0.0, entry.deadline - time.monotonic())
        if not entry.ready.wait(remaining):
            with self._guard:
                if entry.creating and entry.error is None:
                    entry.error = 'Service client creation timed out'
            # Expiry wakes callers; only executor consumption releases a queued
            # reservation. Repeated explicit calls cannot accumulate more tasks.
            entry.ready.set()
            raise RuntimeError('Service client creation timed out')
        with self._guard:
            if self._closed or request.aborted:
                raise RuntimeError('ROS node closing')
            if entry.error is not None:
                raise RuntimeError(entry.error)
            client = entry.client_ref() if entry.client_ref else None
            if client is None:
                raise RuntimeError('Node-owned client unavailable')
            request.client = client
            return client

    def _service_type_name(self, node: Any, name: str) -> str:
        """Disambiguate cached client types using standard server-only graph API."""
        types = dict(node.get_service_names_and_types()).get(name)
        if not types:
            raise services.InvalidServiceException(name)
        if len(types) == 1:
            return types[0]
        server_types = set()
        for node_name, namespace in set(node.get_node_names_and_namespaces()):
            with self._guard:
                if self._closed:
                    raise RuntimeError('ROS node closing')
            try:
                graph = node.get_service_names_and_types_by_node(node_name, namespace)
            except NodeNameNonExistentError:
                continue  # A graph snapshot does not pin a remote node lifetime.
            for service_name, offered_types in graph:
                if service_name == name:
                    server_types.update(offered_types)
        if not server_types:
            raise services.InvalidServiceException(name)
        if len(server_types) != 1:
            raise RuntimeError(f'Service {name} has ambiguous server types')
        return next(iter(server_types))

    @staticmethod
    def _clean_future(client: Any, future: Any) -> None:
        """Remove only this call; cancel never means remote service cancellation."""
        if future is not None:
            if not future.done():
                future.cancel()
            client.remove_pending_request(future)

    def call_service(self, service: str, args: Any = None,
                     server_ready_timeout: float = 1.0,
                     server_response_timeout: float = 5.0) -> dict:
        """Keep the ordinary rosbridge Service conversion and result behavior."""
        request = _Request()
        self._admit(request)
        client = future = None
        try:
            node = self._owner()
            if node is None:
                raise RuntimeError('ROS node unavailable')
            self._owner_executor(node)
            service = node.resolve_service_name(service)
            type_name = self._service_type_name(node, service)
            service_type = services.get_service_class(type_name)
            message = services.get_service_request_instance(type_name)
            services.args_to_service_request_instance(message, args)
            client = self._acquire(node, (service, service_type), request)
            if not client.wait_for_service(server_ready_timeout):
                raise services.InvalidServiceException(service)
            try:
                future = client.call_async(message)
            except BaseException as error:
                with self._guard:
                    if request.entry.error is None:
                        request.entry.error = _failure_text(error)
                    self._errors += 1
                raise
            with self._guard:
                request.future = future
                aborted = request.aborted or self._closed
            if aborted:
                raise RuntimeError('ROS node closing')
            future.add_done_callback(lambda _: request.done.set())
            timeout = server_response_timeout if server_response_timeout > 0 else None
            if not request.done.wait(timeout):
                raise Exception('Timeout exceeded while waiting for service response')
            with self._guard:
                if request.aborted:
                    raise RuntimeError('ROS node closing')
            result = future.result()
            if result is None:
                raise Exception('Service call exception: ' + str(future.exception()))
            return services.extract_values(result)
        finally:
            try:
                self._clean_future(client, future)
            finally:
                with self._guard:
                    self._requests.discard(request)
                    if request.entry is not None:
                        request.entry.users -= 1
                        self._prune_empty((service, service_type), request.entry)
                        request.entry = None
                    request.future = request.client = None

    def close(self) -> None:
        """Wake local waiters; native entities are still destroyed by their Node."""
        with self._guard:
            if self._closed:
                return
            self._closed = True
            requests, entries = list(self._requests), list(self._entries.values())
            pending = [(r.client, r.future) for r in requests if r.future is not None]
            for request in requests:
                request.aborted = True
            for entry in entries:
                if entry.creating:
                    entry.error = 'ROS node closing'
                    entry.creating = False
        for entry in entries:
            entry.ready.set()
        for request in requests:
            request.done.set()
        for client, future in pending:
            self._clean_future(client, future)

    def stats(self) -> dict:
        """Return primitives only; this is not proof of all native pending state."""
        with self._guard:
            return {'active': len(self._requests), 'entries': len(self._entries),
                    'pendingCreations': sum(e.creating for e in self._entries.values()),
                    'pendingRetirements': sum(
                        e.retirement_pending for e in self._entries.values()),
                    'created': self._created, 'evicted': self._evicted,
                    'retyped': self._retyped,
                    'unconfirmedCreations': self._unconfirmed_creations,
                    'quarantinedEntries': sum(e.quarantined for e in self._entries.values()),
                    'unconsumedCreationTickets': len(self._queued_entries),
                    'retired': self._retired,
                    'lateDiscarded': self._late_discarded,
                    'nativeEntityHighWater': self._native_high,
                    'entryHighWater': self._entry_high, 'activeHighWater': self._active_high,
                    'cacheSize': self._cache_size, 'maxInFlight': self._max_in_flight,
                    'rejectedCalls': self._rejected_calls, 'rejectedCache': self._rejected_cache,
                    'errors': self._errors, 'closed': self._closed}
