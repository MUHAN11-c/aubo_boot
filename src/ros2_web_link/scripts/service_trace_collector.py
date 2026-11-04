#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Record standard Nodes service metadata and a persistent native DDS control."""

import argparse
from collections import Counter, deque
import json
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from rosidl_runtime_py.utilities import get_service
from service_trace import install_service_trace


def main():
    """Observe the read-only rosapi query without changing upstream callbacks."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=int, required=True)
    parser.add_argument('--passive', action='store_true',
                        help='Only subscribe to metadata; never create a control client')
    parser.add_argument('--service', default='/rosapi/nodes')
    parser.add_argument('--service-type', default='rosapi_msgs/srv/Nodes')
    args = parser.parse_args()
    if not 2 <= args.seconds <= 3660:
        parser.error('seconds must be 2..3660')
    if not args.passive and args.service_type != 'rosapi_msgs/srv/Nodes':
        parser.error('active control requires rosapi_msgs/srv/Nodes')
    service_type = get_service(args.service_type)
    root = args.out.resolve()
    root.mkdir(parents=True, exist_ok=True)
    counts = Counter()
    latency = deque(maxlen=2000)
    native = {'calls': 0, 'completed': 0, 'timeouts': 0, 'errors': 0}
    rclpy.init(args=[])
    install_service_trace()
    node = Node('web_link_service_trace')
    log = (root / 'service-events.jsonl').open('w', buffering=1)
    started = time.monotonic()

    def record(kind, **values):
        """Bound the trace file and exclude request/response content."""
        if log.tell() > 16000000:
            raise RuntimeError('Service metadata trace exceeds 16MB')
        log.write(json.dumps({'kind': kind, 'wall': time.time(),
                              'elapsed': time.monotonic() - started, **values}) + '\n')

    def event(message):
        """Keep native phase identifiers for GID/sequence correlation."""
        info = message.info
        counts[str(info.event_type)] += 1
        record('service_event', phase=info.event_type, gid=bytes(info.client_gid).hex(),
               sequence=info.sequence_number, stamp=info.stamp.sec + info.stamp.nanosec / 1e9)

    def summary():
        """Persist control latency separately from the browser measurements."""
        ordered = sorted(latency)
        data = {'rmw': get_rmw_implementation_identifier(),
                'mode': 'passive-server' if args.passive else 'metadata+control',
                'service': args.service, 'serviceType': args.service_type,
                'native': native, 'phaseCounts': dict(counts),
                'nativeP95Ms': ordered[int((len(ordered) - 1) * .95)] if ordered else None,
                'elapsed': time.monotonic() - started}
        (root / 'service-trace.json').write_text(json.dumps(data, indent=2))

    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(service_type.Event, args.service + '/_service_event', event, qos)
    client = None if args.passive else node.create_client(service_type, args.service)
    future = None
    try:
        if client is not None and not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('rosapi Nodes service is not ready')
        (root / 'trace.ready').write_text('ready')
        next_call, next_report = time.monotonic(), time.monotonic()
        call_at = 0.0
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if future is not None:
                if future.done():
                    duration = (now - call_at) * 1000
                    if future.exception() is not None or not future.result().nodes:
                        native['errors'] += 1
                        record('native_error', elapsed_ms=duration)
                    else:
                        native['completed'] += 1
                        latency.append(duration)
                        record('native_response', elapsed_ms=duration)
                    future = None
                elif now - call_at > 3:
                    native['timeouts'] += 1
                    record('native_timeout')
                    future.cancel()
                    future = None
            if client is not None and future is None and now >= next_call:
                call_at, next_call = now, now + 2
                future = client.call_async(service_type.Request())
                native['calls'] += 1
                record('native_request')
            if now >= next_report:
                summary()
                next_report = now + 5
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if future is not None:
            future.cancel()
        summary()
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
