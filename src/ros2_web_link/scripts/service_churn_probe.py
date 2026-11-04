#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""
Compare persistent and short-lived native clients for read-only rosapi Nodes.

Run against an explicitly isolated test domain with an existing rosapi server.
This probe never uses WebSocket, retries a failed call, or changes the server.
Every cycle makes one independent query with each client lifetime policy.
"""

import argparse
import json
from pathlib import Path
import time

import psutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
from rosapi_msgs.srv import Nodes


def invoke(node, client):
    """Use public Jazzy async/future APIs; distinguish discovery from response."""
    at = time.monotonic()
    ready = client.wait_for_service(timeout_sec=1)
    ready_ms = (time.monotonic() - at) * 1000
    if not ready:
        return {'status': 'DISCOVERY_TIMEOUT', 'readyMs': ready_ms}
    future = None
    try:
        at = time.monotonic()
        future = client.call_async(Nodes.Request())
        # Request submission and executor waiting share one deadline. A late
        # executor wake may expose a completed future after the budget expired;
        # that is still a failed latency measurement, never a green response.
        remaining = max(0, 3 - (time.monotonic() - at))
        rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)
        elapsed = (time.monotonic() - at) * 1000
        if not future.done() or elapsed > 3000:
            return {'status': 'RESPONSE_TIMEOUT', 'readyMs': ready_ms, 'responseMs': elapsed,
                    'futureDone': future.done()}
        error = future.exception()
        if error is not None:
            return {'status': 'ERROR', 'readyMs': ready_ms, 'error': str(error)[:2000]}
        response = future.result()
        if response is None or not response.nodes:
            return {'status': 'EMPTY_RESPONSE', 'readyMs': ready_ms}
        return {'status': 'PASS', 'readyMs': ready_ms, 'responseMs': elapsed,
                'nodes': len(response.nodes)}
    finally:
        # Cancelling only releases local waiting. Remove the public pending
        # request explicitly even when no further executor spin will occur.
        if future is not None and not future.done():
            client.remove_pending_request(future)
            future.cancel()


def main():
    """Bound duration, retained latency, log bytes and native process memory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=120)
    parser.add_argument('--period-ms', type=int, default=100)
    args = parser.parse_args()
    if not 2 <= args.seconds <= 3600 or not 100 <= args.period_ms <= 10000:
        parser.error('seconds must be 2..3600 and period-ms 100..10000')
    root = args.out.resolve()
    root.mkdir(parents=True, exist_ok=True)
    if (root / 'service-churn.json').exists():
        parser.error('service churn evidence already exists; choose a fresh output directory')
    try:
        # Claim the output before opening DDS. O_EXCL semantics also serialize
        # accidental concurrent invocations; an exists-then-w check cannot.
        log = (root / 'service-churn.jsonl').open('x')
    except FileExistsError:
        parser.error('service churn evidence already exists; choose a fresh output directory')
    counts = {kind: {'calls': 0, 'completed': 0, 'failed': 0}
              for kind in ('persistent', 'ephemeral')}
    latency = {kind: [] for kind in counts}
    peak_rss = 0
    failure = None
    interrupted = False
    started = time.monotonic()
    node = None
    try:
        rclpy.init(args=[])
        node = Node('web_link_service_churn_probe')
        persistent = node.create_client(Nodes, '/rosapi/nodes')
        if not persistent.wait_for_service(timeout_sec=5):
            raise RuntimeError('read-only /rosapi/nodes is not available in this domain')
        with log:
            cycle = 0
            while rclpy.ok() and time.monotonic() - started < args.seconds:
                cycle_at = time.monotonic()
                # Alternate ordering to avoid always giving one policy the
                # first query after the idle interval.
                kinds = ('ephemeral', 'persistent') if cycle % 2 else counts.keys()
                for kind in kinds:
                    client = persistent if kind == 'persistent' else node.create_client(
                        Nodes, '/rosapi/nodes')
                    try:
                        counts[kind]['calls'] += 1
                        try:
                            result = invoke(node, client)
                        except Exception as error:
                            result = {'status': 'EXCEPTION', 'error': str(error)[:2000]}
                    finally:
                        if kind == 'ephemeral':
                            node.destroy_client(client)
                    record = {'elapsed': time.monotonic() - started, 'mode': kind, **result}
                    line = json.dumps(record) + '\n'
                    if log.tell() + len(line.encode('utf-8')) > 16000000:
                        raise RuntimeError('native service churn log exceeds 16MB')
                    log.write(line)
                    log.flush()
                    if result['status'] != 'PASS':
                        counts[kind]['failed'] += 1
                        raise RuntimeError(f'{kind}: {result}')
                    counts[kind]['completed'] += 1
                    latency[kind].append(result['responseMs'])
                peak_rss = max(peak_rss, psutil.Process().memory_info().rss / 1048576)
                if peak_rss > 256 or psutil.virtual_memory().available < 4 * 1024**3:
                    raise RuntimeError('native service churn memory guard reached')
                cycle += 1
                remaining = args.period_ms / 1000 - (time.monotonic() - cycle_at)
                if remaining > 0:
                    time.sleep(remaining)
            if not rclpy.ok():
                interrupted = True
            if not interrupted and any(not info['completed'] for info in counts.values()):
                raise RuntimeError('No completed comparison for both client lifetime policies')
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    except Exception as error:
        failure = str(error)[:4000]
    finally:
        peak_rss = max(peak_rss, psutil.Process().memory_info().rss / 1048576)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        log.close()
        data = {'status': 'FAIL' if failure else 'INTERRUPTED' if interrupted else 'PASS',
                'failure': failure, 'rmw': get_rmw_implementation_identifier(),
                'endpoint': '/rosapi/nodes', 'durationSeconds': time.monotonic() - started,
                'periodMs': args.period_ms, 'responseTimeoutSeconds': 3,
                'peakRssMiB': peak_rss, 'counts': counts,
                'p95Ms': {kind: sorted(values)[int((len(values) - 1) * .95)]
                          if values else None for kind, values in latency.items()}}
        # Refuse an independently created summary too, rather than overwriting
        # another investigator's result after our ROS resources were released.
        with (root / 'service-churn.json').open('x') as summary:
            summary.write(json.dumps(data, indent=2) + '\n')
        print(json.dumps(data), flush=True)
    return 1 if failure or interrupted else 0


if __name__ == '__main__':
    raise SystemExit(main())
