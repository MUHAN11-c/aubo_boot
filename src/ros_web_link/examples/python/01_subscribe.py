#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Example 01 — 经 ros_web_link 订阅一个 ROS 2 话题.

先启动自包含 fixture 栈：

    ros2 launch ros_web_link transport.launch.xml fixture:=true

再运行：

    python examples/python/01_subscribe.py --count 3
    python examples/python/01_subscribe.py --topic /rwl_demo/chatter --json
"""
from __future__ import annotations

import argparse
import asyncio
import importlib
import json
import sys

DEFAULT_URL = 'ws://127.0.0.1:8095/ws/foxglove'


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default=DEFAULT_URL,
                   help=f'网关 WebSocket URL（默认 {DEFAULT_URL}）')
    p.add_argument('--topic', default='/rwl_demo/chatter')
    p.add_argument('--type', default='std_msgs/msg/String',
                   help='ROS 2 消息类型，例如 sensor_msgs/msg/JointState')
    p.add_argument('--count', type=int, default=3)
    p.add_argument('--timeout', type=float, default=10.0)
    p.add_argument('--json', action='store_true', help='只输出 JSON（自动化用）')
    return p.parse_args(argv)


def resolve_msg_type(type_name: str):
    """'sensor_msgs/msg/JointState' → message class（延迟 import，--help 不依赖 ROS）."""
    pkg, sep, cls = type_name.partition('/msg/')
    if not sep or not pkg or not cls:
        raise ValueError(f'消息类型格式应为 pkg/msg/Type: {type_name!r}')
    return getattr(importlib.import_module(f'{pkg}.msg'), cls)


async def run(args) -> dict:
    from rosidl_runtime_py.convert import message_to_ordereddict

    from ros_web_link.foxglove_ws_client import FoxgloveWsClient

    msg_type = resolve_msg_type(args.type)
    client = FoxgloveWsClient()
    await client.connect(args.url, timeout=args.timeout)
    sub = await client.subscribe(args.topic, msg_type, timeout=args.timeout)
    try:
        messages = []
        for _ in range(max(1, args.count)):
            msg = await sub.next(timeout=args.timeout)
            messages.append(message_to_ordereddict(msg))
        return {'url': args.url, 'topic': args.topic, 'type': args.type,
                'count': len(messages), 'messages': messages}
    finally:
        await sub.cancel()
        await client.close()


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        result = asyncio.run(run(args))
    except Exception as exc:  # noqa: BLE001 — CLI 边界统一退出码
        print(f'[01_subscribe] {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, ensure_ascii=False, default=str))
    else:
        print(f"topic={result['topic']} type={result['type']} count={result['count']}")
        for i, msg in enumerate(result['messages']):
            print(f'  [{i}] {json.dumps(msg, ensure_ascii=False, default=str)[:400]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
