# Copyright 2026 wjz
"""
Web 监控台 HTTP 层：静态文件与只读状态 API 的 Handler 与启动函数.

只读设计（2026-08-13 起）：不提供任何 POST/写入口，控制与调试全部
移出 Web（自动全流程由编排器闭环），本层只回答状态快照。Handler 不
闭包引用 ROS 节点，只经 `_DashboardHTTPServer` 上的窄接口
`HttpBackend`（snapshot() 一个方法）取依赖，可用 fake 后端在单元
测试里直接起真实 server 打请求。
"""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Protocol
from urllib.parse import urlparse


class HttpBackend(Protocol):
    """HTTP Handler 依赖的窄接口（由 PeachPerceptionWeb 实现，测试可伪造）."""

    def snapshot(self) -> dict:
        """返回浏览器状态快照（GET /api/state 的载荷）."""
        ...


class DashboardHttpHandler(BaseHTTPRequestHandler):
    """只读 GET handler（依赖全经 server 窄接口）."""

    # 类型注解仅供阅读：实例属性来自 _DashboardHTTPServer
    server: '_DashboardHTTPServer'

    def log_message(self, fmt, *args):
        """访问日志转交后端 debug 日志（ROS 节点或测试 noop）."""
        self.server.log_debug(fmt % args)

    def _send(self, status, content_type, data, cache='no-store'):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', cache)
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header(
            'Content-Security-Policy',
            "default-src 'self'; object-src 'none'; "
            "frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(data)

    def _json(self, value, status=HTTPStatus.OK):
        data = json.dumps(
            value, ensure_ascii=False,
            separators=(',', ':')).encode('utf-8')
        self._send(status, 'application/json; charset=utf-8', data)

    def do_GET(self):
        """只读入口：状态快照与静态资源."""
        parsed = urlparse(self.path)
        path = parsed.path
        if path == '/api/state':
            self._json(self.server.backend.snapshot())
            return
        assets = {
            '/': ('index.html', 'text/html; charset=utf-8'),
            '/index.html': ('index.html', 'text/html; charset=utf-8'),
            '/app.css': ('app.css', 'text/css; charset=utf-8'),
            '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
        }
        asset = assets.get(path)
        if asset is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = (self.server.web_root / asset[0]).read_bytes()
        self._send(
            HTTPStatus.OK, asset[1], data,
            cache='public, max-age=60')

    def do_POST(self):
        """只读监控台：一切写请求统一 405."""
        self._json(
            {'accepted': False, 'message': '只读监控台，无写入口'},
            HTTPStatus.METHOD_NOT_ALLOWED)


class _DashboardHTTPServer(ThreadingHTTPServer):
    """携带窄接口后端与静态根目录的 HTTP 服务（Handler 经 server 取依赖）."""

    def __init__(self, server_address, backend: HttpBackend,
                 web_root: Path, log_debug):
        """记录后端/静态根/日志回调；属性须先于基类 bind 就绪."""
        self.backend = backend
        self.web_root = Path(web_root)
        self.log_debug = log_debug
        super().__init__(server_address, DashboardHttpHandler)


def start_http(host: str, port: int, web_root, backend: HttpBackend,
               log_debug) -> ThreadingHTTPServer:
    """
    构建 HTTP 服务并在后台守护线程启动.

    Args:
        host: 监听地址.
        port: 监听端口；0 表示由内核分配（测试用，实际端口经返回
            server 的 server_address 读取）.
        web_root: 静态文件根目录（index.html/app.css/app.js）.
        backend: HttpBackend 窄接口实现.
        log_debug: debug 日志回调（接单参数字符串）.

    Returns
    -------
        运行中的 ThreadingHTTPServer（调用方负责 shutdown/server_close）.

    """
    server = _DashboardHTTPServer(
        (host, port), backend, Path(web_root), log_debug)
    thread = threading.Thread(
        target=server.serve_forever,
        name='peach-perception-http', daemon=True)
    thread.start()
    return server
