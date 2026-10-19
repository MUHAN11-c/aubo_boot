#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Example 03 — 经 ros_web_link 读写参数 + 查询连接图.

先启动 fixture 栈：

    ros2 launch ros_web_link transport.launch.xml fixture:=true

再运行：

    python examples/python/03_params_and_graph.py --json

演示 get_params / set_param（标准 ~/get_parameters、~/set_parameters 服务）
与 subscribe_connection_graph / graph_snapshot。
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
    p.add_argument('--node', default='/comm_fixture')
    p.add_argument('--param', default='rwl.answer')
    p.add_argument('--value', type=int, default=43)
    p.add_argument('--timeout', type=float, default=10.0)
    p.add_argument('--json', action='store_true')
    return p.parse_args(argv)


async def run(args) -> dict:
    from ros_web_link.foxglove_ws_client import FoxgloveWsClient

    client = FoxgloveWsClient()
    await client.connect(args.url, timeout=args.timeout)
    try:
        before = await client.get_params(args.node, timeout=args.timeout)
        result = await client.set_param(
            args.node, args.param, args.value, timeout=args.timeout)
        after = await client.get_params(
            args.node, names=[args.param], timeout=args.timeout)

        await client.subscribe_connection_graph()
        deadline = asyncio.get_running_loop().time() + args.timeout
        graph = {}
        while asyncio.get_running_loop().time() < deadline:
            graph = client.graph_snapshot()
            if graph.get('topics'):
                break
            await asyncio.sleep(0.2)
        return {
            'url': args.url,
            'node': args.node,
            'params_before': before,
            'set': {'name': args.param, 'value': args.value,
                    'successful': bool(result.successful),
                    'reason': result.reason},
            'params_after': after,
            'graph': graph,
        }
    finally:
        await client.close()


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        result = asyncio.run(run(args))
    except Exception as exc:  # noqa: BLE001
        print(f'[03_params_and_graph] {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, ensure_ascii=False, default=str))
    else:
        print(f"node {result['node']}")
        print(f"  params: {result['params_before']}")
        print(f"  set {result['set']}")
        print(f"  topics: {len(result['graph'].get('topics', []))} · "
              f"services: {len(result['graph'].get('services', []))} · "
              f"nodes: {len(result['graph'].get('nodes', []))}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
