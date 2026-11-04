#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Exercise public executor boundaries without a ROS context or native entities."""

from threading import Event, Thread
import time
from unittest.mock import patch

from rclpy.task import Future
from service_budget import ServiceCallBudget
from std_srvs.srv import Trigger


class Executor:
    """Control only delivery at the public create_task boundary."""

    def __init__(self):
        self.pending = []
        self.wake_failure = False

    def create_task(self, operation):
        """Keep accepted work even if waking the owner subsequently fails."""
        self.pending.append(operation)
        if self.wake_failure:
            self.wake_failure = False
            raise RuntimeError('wake failed after enqueue')

    def drain(self):
        """Deliver queued public tasks on the controlled owner."""
        while self.pending:
            self.pending.pop(0)()


class Client:
    """Represent an allocated handle and an actual rclpy Future."""

    def __init__(self):
        self.future = Future()
        self.calls = self.removed = 0
        self.sent = Event()

    def wait_for_service(self, timeout):
        """Represent an already discovered fixture server."""
        return True

    def call_async(self, request):
        """Expose request submission without a DDS operation."""
        self.calls += 1
        self.sent.set()
        return self.future

    def remove_pending_request(self, future):
        """Observe cleanup against the same Future."""
        assert future is self.future
        self.removed += 1


class Node:
    """Substitute external entity boundaries, leaving budget logic real."""

    def __init__(self):
        self.executor = Executor()
        self.created, self.destroyed = [], []
        self.destroy_ok = True
        self.create_seconds = 0

    def resolve_service_name(self, service):
        """No remapping in this fixture."""
        return service

    def get_service_names_and_types(self):
        """Use an installed standard service and its real converters."""
        return [('/probe', ['std_srvs/srv/Trigger'])]

    def create_client(self, *args, **kwargs):
        """Delay actual allocation independently of executor task delivery."""
        time.sleep(self.create_seconds)
        client = Client()
        self.created.append(client)
        return client

    def destroy_client(self, client):
        """Model the public success confirmation, including failure."""
        if self.destroy_ok:
            self.destroyed.append(client)
        return self.destroy_ok


def wait(predicate):
    """Bound synchronization and never leave a test worker running."""
    deadline = time.monotonic()+1
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.001)
    assert predicate(), 'fixture synchronization deadline'


def call(budget):
    """Invoke the actual uncached service path and real wire conversion."""
    return budget.run(lambda reservation: budget.call_uncached(
        reservation, '/probe', {}, server_response_timeout=.1))


def main():
    """Check reservations, stale tasks, late allocations and shutdown."""
    node = Node()
    budget = ServiceCallBudget(node, 1, .02)
    try:
        call(budget)
    except RuntimeError as error:
        assert 'creation deadline' in str(error)
    else:
        raise AssertionError('blocked creation became success')
    assert budget.stats()['active'] == 1
    try:
        call(budget)
    except RuntimeError as error:
        assert 'budget limit' in str(error)
    else:
        raise AssertionError('stale queued creation released its reservation')
    node.executor.drain()
    assert not node.created and budget.stats()['active'] == 0
    print('PASS finite creation deadline; expired owner tasks keep admission until consumed')

    node = Node()
    budget = ServiceCallBudget(node, 1, .1)
    node.executor.wake_failure = True
    try:
        call(budget)
    except RuntimeError:
        pass
    node.executor.drain()
    assert not node.created, 'a failed waiter must not create a client after enqueue/wake failure'
    assert budget.stats()['active'] == 0
    print('PASS enqueue-then-wake failure never creates a late orphan client')

    node = Node()
    budget = ServiceCallBudget(node, 1, .1)
    errors = []

    def registered_then_failed(*args, **kwargs):
        node.created.append(Client())
        raise RuntimeError('factory registered a handle before wake failed')

    node.create_client = registered_then_failed
    budget.submit(lambda: call(budget), errors.append)
    wait(lambda: bool(node.executor.pending))
    node.executor.drain()
    wait(lambda: bool(errors))
    assert budget.stats()['active'] == 1 and len(node.created) == 1
    budget.close()
    print('PASS factory failure after possible registration quarantines its reservation')

    node = Node()
    node.create_seconds = .04
    budget = ServiceCallBudget(node, 1, .02)
    errors = []
    budget.submit(lambda: call(budget), errors.append)
    wait(lambda: bool(node.executor.pending))
    owner = Thread(target=node.executor.drain)
    owner.start()
    owner.join(1)
    assert not owner.is_alive()
    wait(lambda: bool(errors))
    assert len(node.created) == len(node.destroyed) == 1
    assert node.created[0].calls == 0 and budget.stats()['active'] == 0
    print('PASS allocation already in progress expires, retires on owner, and sends no request')

    node = Node()
    budget = ServiceCallBudget(node, 1, 1)
    errors = []
    budget.submit(lambda: call(budget), errors.append)
    wait(lambda: bool(node.executor.pending))
    node.executor.drain()
    wait(lambda: bool(node.created) and node.created[0].sent.is_set())
    node.created[0].future.set_result(Trigger.Response(success=True, message='ok'))
    wait(lambda: bool(node.executor.pending))
    node.destroy_ok = False
    node.executor.drain()
    assert budget.stats()['active'] == 1, 'unconfirmed retirement must retain its reservation'
    budget.close()
    print('PASS failed native retirement quarantines its admission slot')

    node = Node()
    budget = ServiceCallBudget(node, 1, 1)
    errors = []
    budget.submit(lambda: call(budget), errors.append)
    wait(lambda: bool(node.executor.pending))
    budget.close()
    wait(lambda: bool(errors))
    node.executor.drain()
    assert not node.created and budget.stats()['active'] == 0
    print('PASS Node shutdown wakes blocked creation and skips queued native work')

    node = Node()
    budget = ServiceCallBudget(node, 1, 1)
    errors = []
    budget.submit(lambda: budget.run(lambda reservation: budget.call_uncached(
        reservation, '/probe', {}, server_response_timeout=0)), errors.append)
    wait(lambda: bool(node.executor.pending))
    node.executor.drain()
    wait(lambda: bool(node.created) and node.created[0].sent.is_set())
    budget.close()
    wait(lambda: bool(errors) and budget.stats()['active'] == 0)
    assert node.created[0].future.cancelled() and node.created[0].removed == 1
    print('PASS Node shutdown cancels local indefinite response wait and pending Future')

    node = Node()
    budget = ServiceCallBudget(node, 1, 1)
    with patch('service_budget.Thread', side_effect=RuntimeError('no thread resources')):
        try:
            budget.submit(lambda: None, lambda _: None)
        except RuntimeError:
            pass
    assert budget.stats()['active'] == 0
    print('PASS thread construction failure releases admission; no ROS/GL/port opened')


if __name__ == '__main__':
    main()
