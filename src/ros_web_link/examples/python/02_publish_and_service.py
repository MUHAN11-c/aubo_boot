#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Example 02 — 经 ros_web_link 发布话题 + 调用服务.

先启动 fixture 栈：

    ros2 launch ros_web_link transport.launch.xml fixture:=true

再运行：

    python examples/python/02_publish_and_service.py --json

演示：
* 订阅 /rwl_demo/echo_reply → 发布 /rwl_demo/echo → 收到 echo:<原串>
* 调用 /rwl_demo/add_two_ints（example_interfaces/srv/AddTwoInts）
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys

DEFAULT_URL = 'ws://127.0.0.1:8095/ws/foxglove'


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default=DEFAULT_URL)
    p.add_argument('--text', default='hello from ros_web_link example')
    p.add_argument('--a', type=int, default=20)
    p.add_argument('--b', type=int, default=22)
    p.add_argument('--timeout', type=float, default=10.0)
    p.add_argument('--json', action='store_true')
    return p.parse_args(argv)


async def run(args) -> dict:
    from example_interfaces.srv import AddTwoInts
    from std_msgs.msg import String

    from ros_web_link.foxglove_ws_client import FoxgloveWsClient

    client = FoxgloveWsClient()
    await client.connect(args.url, timeout=args.timeout)
    try:
        # 先订阅回发话题，避免 fixture 回包早于订阅。
        reply_sub = await client.subscribe(
            '/rwl_demo/echo_reply', String, timeout=args.timeout)
        try:
            await client.publish('/rwl_demo/echo', 'std_msgs/msg/String',
                                 String(data=args.text))
            reply = await reply_sub.next(timeout=args.timeout)
        finally:
            await reply_sub.cancel()

        req = AddTwoInts.Request(a=args.a, b=args.b)
        resp = await client.call_service(
            '/rwl_demo/add_two_ints', req, AddTwoInts.Response,
            timeout=args.timeout)
        return {
            'url': args.url,
            'published': {'topic': '/rwl_demo/echo', 'data': args.text},
            'echo_reply': {'topic': '/rwl_demo/echo_reply', 'data': reply.data},
            'service': {'name': '/rwl_demo/add_two_ints',
                        'request': {'a': args.a, 'b': args.b},
                        'response': {'sum': int(resp.sum)}},
        }
    finally:
        await client.close()


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        result = asyncio.run(run(args))
    except Exception as exc:  # noqa: BLE001
        print(f'[02_publish_and_service] {type(exc).__name__}: {exc}',
              file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, ensure_ascii=False, default=str))
    else:
        print(f"echo: {result['published']['data']} -> {result['echo_reply']['data']}")
        svc = result['service']
        print(f"service {svc['name']}: {svc['request']} -> {svc['response']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
