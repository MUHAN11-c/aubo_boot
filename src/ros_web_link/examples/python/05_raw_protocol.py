#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Example 05 — 直接观察 Foxglove WebSocket v1 的 serverInfo 帧.

不经过 FoxgloveWsClient，便于对照 foxglove ws-protocol 文档：

    python examples/python/05_raw_protocol.py --json

连接后打印 subprotocol、capabilities、channels/services 数量。
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
    p.add_argument('--timeout', type=float, default=10.0)
    p.add_argument('--json', action='store_true')
    return p.parse_args(argv)


async def run(args) -> dict:
    import websockets

    async with websockets.connect(
            args.url, subprotocols=['foxglove.websocket.v1'],
            max_size=128 * 1024 * 1024, ping_interval=20,
            ping_timeout=20) as ws:
        raw = await asyncio.wait_for(ws.recv(), args.timeout)
        info = json.loads(raw)
        return {
            'url': args.url,
            'subprotocol': ws.subprotocol,
            'op': info.get('op'),
            'capabilities': info.get('capabilities', []),
            'channels': len(info.get('channels', [])),
            'services': len(info.get('services', [])),
            'server_info': info,
        }


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        result = asyncio.run(run(args))
    except Exception as exc:  # noqa: BLE001
        print(f'[05_raw_protocol] {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, ensure_ascii=False, default=str))
    else:
        print(f"subprotocol={result['subprotocol']} op={result['op']}")
        print(f"capabilities={result['capabilities']}")
        print(f"channels={result['channels']} services={result['services']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
