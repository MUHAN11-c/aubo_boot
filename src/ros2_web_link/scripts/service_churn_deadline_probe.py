#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check absolute response deadlines without opening a ROS context."""

from types import SimpleNamespace
from unittest.mock import patch

from rclpy.task import Future
from service_churn_probe import invoke


class Client:
    """Control send duration and future completion independently of DDS."""

    def __init__(self, clock, send_seconds=0):
        self.clock = clock
        self.send_seconds = send_seconds
        self.future = Future()
        self.removed = 0

    def wait_for_service(self, timeout_sec):
        """Represent an already discovered service."""
        return True

    def call_async(self, request):
        """Make request submission consume an explicit portion of the budget."""
        self.clock[0] += self.send_seconds
        return self.future

    def remove_pending_request(self, future):
        """Observe local timeout cleanup."""
        assert future is self.future
        self.removed += 1


def scenario(send_seconds, response_at):
    """Run the real helper against a controlled executor/monotonic clock."""
    clock = [0.0]
    client = Client(clock, send_seconds)
    waits = []

    def spin(node, future, timeout_sec):
        waits.append(timeout_sec)
        if response_at is not None:
            clock[0] = response_at
            future.set_result(SimpleNamespace(nodes=['fixture']))

    with patch('service_churn_probe.time.monotonic', side_effect=lambda: clock[0]), \
            patch('service_churn_probe.rclpy.spin_until_future_complete', side_effect=spin):
        result = invoke(object(), client)
    return result, client, waits


result, client, waits = scenario(0, 3.1)
assert result['status'] == 'RESPONSE_TIMEOUT', result
assert result['responseMs'] == 3100 and client.future.done()
result, client, waits = scenario(4, None)
assert result['status'] == 'RESPONSE_TIMEOUT' and waits == [0], (result, waits)
assert client.removed == 1 and client.future.cancelled()
result, client, waits = scenario(.5, .52)
assert result['status'] == 'PASS' and waits == [2.5], (result, waits)
print('PASS late completion cannot pass; send uses the same deadline; pending request released')
