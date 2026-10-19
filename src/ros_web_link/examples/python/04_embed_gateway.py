#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Example 04 — 在自己的 Python 应用里组合 ros_web_link 网关.

    python examples/python/04_embed_gateway.py --port 8095

等价于 ``ros_web_link_gateway``，但展示 create_app() 组合点：
* 纯传输模式（仅通信层 JS + console）
* 组合模式（js_roots/html_root/sub_routers 注入自有 UI/BFF）
"""
from __future__ import annotations

import argparse


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=8095)
    p.add_argument('--bind', default='127.0.0.1')
    p.add_argument('--bridge-host', default='127.0.0.1')
    p.add_argument('--bridge-port', type=int, default=18766)
    p.add_argument('--open', action='store_true', help='启动后打开浏览器')
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    # 必须在 create_app 之前把桥地址写进 config（与 CLI 同一条配置链）。
    from ros_web_link import config as cfg
    cfg.init({
        'foxglove_bridge_host': args.bridge_host,
        'foxglove_bridge_port': args.bridge_port,
    })

    import uvicorn
    from ros_web_link.app import create_app

    # 纯传输模式；组合模式示例：
    #   app = create_app(js_roots=['/path/to/ui/js'], html_root='/path/to/ui',
    #                    sub_routers=[my_bff_router])
    app = create_app()
    url = f'http://{args.bind}:{args.port}/'
    print(f'[04_embed_gateway] {url}  (console: /console.html)', flush=True)
    if args.open:
        import threading
        import webbrowser
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    uvicorn.run(app, host=args.bind, port=args.port, log_level='info')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
