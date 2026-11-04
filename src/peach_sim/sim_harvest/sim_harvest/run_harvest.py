"""Submit one explicit simulation harvest action and report its actual result."""

import argparse
import json
import time

from peach_interfaces.action import RunHarvest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def main(args=None):
    """Wait for readiness with a wall deadline and cancel before exiting on interruption."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=420)
    parser.add_argument('--target', action='append', default=[])
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Node('sim_harvest_client')
    client = ActionClient(node, RunHarvest, '/sim_harvest/run_harvest')
    handle, accepted = None, None
    try:
        deadline = time.monotonic() + options.timeout
        while not client.wait_for_server(timeout_sec=.2):
            if time.monotonic() > deadline:
                return 2
        request = RunHarvest.Goal(request_id=f'run_{time.time_ns()}',
                                  target_ids=options.target)
        accepted = client.send_goal_async(request)
        while not accepted.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not accepted.done() or not accepted.result().accepted:
            node.get_logger().error('Mission rejected: wait for clock/controllers/Nav2 readiness')
            return 2
        handle = accepted.result()
        result = handle.get_result_async()
        while not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not result.done():
            return 2
        value = result.result().result
        print(json.dumps({'success': value.success, 'reason': value.termination_reason,
                          'succeeded': value.summary.succeeded}, ensure_ascii=False))
        return 0 if value.success else 1
    except KeyboardInterrupt:
        return 130
    finally:
        confirmed = True
        if rclpy.ok() and accepted is not None and not accepted.done():
            end = time.monotonic() + 8
            while not accepted.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            if accepted.done() and accepted.result().accepted:
                handle = accepted.result()
            elif not accepted.done():
                confirmed = False
        if rclpy.ok() and handle and handle.status not in (4, 5, 6):
            terminal = handle.get_result_async()
            cancel = handle.cancel_goal_async()
            end = time.monotonic() + 15
            while (not cancel.done() or not terminal.done()) and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            confirmed = terminal.done() and terminal.result().status in (4, 5, 6)
        if not confirmed:
            node.get_logger().error('cancel_unconfirmed: stop/restart this simulation session')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not confirmed:
            return 3
