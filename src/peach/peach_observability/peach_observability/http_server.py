"""
Web 监控台 HTTP 层：静态文件、只读状态 API 与单步调试 POST.

GET 仍是只读状态；POST 仅开放 `/api/debug/<action>`。运动门控
（motion_enabled）与审计在后端 `debug_command` 内完成，本层只做解析
与转发。Handler 不闭包引用 ROS 节点，只经 `_ObservabilityHttpServer`
上的窄接口 `HttpBackend`（snapshot / trajectory / debug_command）取依赖。
"""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Protocol
from urllib.parse import parse_qs, urlparse

# 调试请求体上限（字节）：调试载荷只有 ID/枚举/坐标数组，64KB 富余
_MAX_BODY_BYTES = 65536


class HttpBackend(Protocol):
    """HTTP Handler 依赖的窄接口（由 ObservabilityNode 实现，测试可伪造）."""

    def snapshot(self) -> dict:
        """返回浏览器状态快照（GET /api/state 的载荷）."""
        ...

    def trajectory(self) -> dict:
        """返回 TCP 点列、visualization_msgs Marker 字典与路标."""
        ...

    def image(self, stream: str) -> tuple:
        """取已编码 JPEG 与帧元数据；无帧返回 None 与原因."""
        ...

    def debug_command(self, action: str, payload: dict,
                      headers) -> tuple:
        """
        执行一次调试操作（运动门控/审计在后端内完成）.

        Args:
            action: 调试端点键（如 'begin_scene_service'）.
            payload: 已解析的 JSON 请求体.
            headers: 请求头（兼容旧客户端；不再校验令牌）.

        Returns
        -------
            (http_status, 响应 dict).

        """
        ...

    def params_report(self) -> dict:
        """返回白名单节点/参数的当前值（GET /api/params）."""
        ...

    def params_set(self, payload: dict) -> tuple:
        """按 change 列表写参数（POST /api/params）；白名单/门控在后端."""
        ...


class ObservabilityHttpHandler(BaseHTTPRequestHandler):
    """只读 GET handler（依赖全经 server 窄接口）."""

    # 类型注解仅供阅读：实例属性来自 _ObservabilityHttpServer
    server: '_ObservabilityHttpServer'

    def log_message(self, fmt, *args):
        """访问日志转交后端 debug 日志（ROS 节点或测试 noop）."""
        self.server.log_debug(fmt % args)

    def _send(self, status, content_type, data, cache='no-store', headers=None):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', cache)
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header(
            'Content-Security-Policy',
            "default-src 'self'; img-src 'self' blob:; object-src 'none'; "
            "frame-ancestors 'none'")
        for name, value in (headers or {}).items():
            # 帧名来自 ROS 消息，不允许响应头注入.
            safe = str(value).replace('\r', '').replace('\n', '')
            self.send_header(name, safe.encode('ascii', 'replace').decode('ascii'))
        self.end_headers()
        self.wfile.write(data)

    def _json(self, value, status=HTTPStatus.OK):
        data = json.dumps(
            value, ensure_ascii=False,
            separators=(',', ':')).encode('utf-8')
        self._send(status, 'application/json; charset=utf-8', data)

    def do_GET(self):
        """只读入口：状态快照、末端轨迹与静态资源."""
        parsed = urlparse(self.path)
        path = parsed.path
        if path == '/api/state':
            self._json(self.server.backend.snapshot())
            return
        if path == '/api/params':
            getter = getattr(self.server.backend, 'params_report', None)
            self._json(getter() if callable(getter) else {'groups': []})
            return
        if path == '/api/trajectory':
            getter = getattr(self.server.backend, 'trajectory', None)
            self._json(getter() if callable(getter) else {})
            return
        query = parse_qs(parsed.query)
        if path == '/api/analysis/success':
            getter = getattr(self.server.backend, 'analysis_success', None)
            self._json(getter() if callable(getter) else {'today': {}, 'all': {}, 'sessions': []})
            return
        if path == '/api/analysis/records':
            getter = getattr(self.server.backend, 'analysis_records', None)
            self._json(getter() if callable(getter) else {'records': []})
            return
        if path == '/api/analysis':
            getter = getattr(self.server.backend, 'analysis_report', None)
            if not callable(getter):
                self._json({'error': 'analysis_unavailable'}, HTTPStatus.NOT_FOUND)
                return
            try:
                self._json(getter((query.get('path') or [''])[0]))
            except ValueError as error:
                self._json({'error': str(error)}, HTTPStatus.BAD_REQUEST)
            return
        if path == '/api/analysis/image':
            getter = getattr(self.server.backend, 'analysis_image', None)
            data = getter(
                (query.get('path') or [''])[0],
                (query.get('name') or [''])[0],
            ) if callable(getter) else None
            if not data:
                self._json({'error': 'no_image'}, HTTPStatus.NOT_FOUND)
            else:
                kind = 'image/png' if str(
                    (query.get('name') or [''])[0]).lower().endswith('.png') else 'image/jpeg'
                self._send(HTTPStatus.OK, kind, data, cache='no-store')
            return
        images = {'/api/images/perception.jpg': 'perception',
                  '/api/images/raw.jpg': 'raw'}
        if path in images:
            getter = getattr(self.server.backend, 'image', None)
            data, metadata = getter(images[path]) if callable(getter) else (
                None, {'available': False, 'error': 'preview_unavailable'})
            if data is None or metadata.get('error') == 'clock_mismatch':
                self._json(metadata, HTTPStatus.NOT_FOUND)
            else:
                self._send(HTTPStatus.OK, 'image/jpeg', data, headers={
                    'X-Frame-Stamp-Ns': metadata['stamp_ns'],
                    'X-Frame-Id': metadata['frame_id'],
                    'X-Frame-Age-S': metadata['age_s']})
            return
        assets = {
            '/': ('index.html', 'text/html; charset=utf-8'),
            '/index.html': ('index.html', 'text/html; charset=utf-8'),
            '/app.css': ('app.css', 'text/css; charset=utf-8'),
            '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
        }
        for filename in ('vendor/peach-three.js', 'viz3d.js',
                         'visualization.js', 'workbench.js', 'ui-common.js',
                         'monitor-renderers.js', 'trajectory-renderers.js',
                         'request-runtime.js'):
            assets['/' + filename] = (filename, 'text/javascript; charset=utf-8')
        for filename in ('visualization.css', 'workbench.css', 'analysis.css'):
            assets['/' + filename] = (filename, 'text/css; charset=utf-8')
        assets['/analysis.html'] = ('analysis.html', 'text/html; charset=utf-8')
        assets['/analysis.js'] = ('analysis.js', 'text/javascript; charset=utf-8')
        assets['/debug.html'] = ('debug.html', 'text/html; charset=utf-8')
        assets['/debug.js'] = ('debug.js', 'text/javascript; charset=utf-8')
        assets['/debug.css'] = ('debug.css', 'text/css; charset=utf-8')
        asset = assets.get(path)
        if asset is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            data = (self.server.web_root / asset[0]).read_bytes()
        except OSError:
            self._json({'error': 'asset_unavailable'}, HTTPStatus.NOT_FOUND)
            return
        self._send(
            HTTPStatus.OK, asset[1], data,
            cache='public, max-age=60')

    def do_POST(self):
        """调试操作面唯一入口：/api/debug/<action> 与 /api/params；门控在后端."""
        parsed = urlparse(self.path)
        if parsed.path == '/api/params':
            self._post_params()
            return
        if not parsed.path.startswith('/api/debug/'):
            self._json(
                {'accepted': False, 'message': '非调试端点（只读监控）'},
                HTTPStatus.NOT_FOUND)
            return
        action = parsed.path[len('/api/debug/'):].strip('/')
        if not action:
            self._json({'accepted': False, 'message': '缺少调试端点'},
                       HTTPStatus.NOT_FOUND)
            return
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except (TypeError, ValueError):
            self._json({'accepted': False, 'message': 'Content-Length 非法'},
                       HTTPStatus.BAD_REQUEST)
            return
        if length < 0:
            self._json({'accepted': False, 'message': 'Content-Length 非法'},
                       HTTPStatus.BAD_REQUEST)
            return
        if length > _MAX_BODY_BYTES:
            self._json({'accepted': False, 'message': '请求体过大'},
                       HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
            return
        try:
            raw = self.rfile.read(length) if length else b'{}'
            payload = json.loads(raw or b'{}')
        except ValueError:
            self._json({'accepted': False, 'message': '请求体不是合法 JSON'},
                       HTTPStatus.BAD_REQUEST)
            return
        if not isinstance(payload, dict):
            self._json({'accepted': False, 'message': '请求体须为 JSON 对象'},
                       HTTPStatus.BAD_REQUEST)
            return
        submit = getattr(self.server.backend, 'debug_command', None)
        if not callable(submit):
            self._json({'accepted': False, 'message': '调试操作面未启用'},
                       HTTPStatus.SERVICE_UNAVAILABLE)
            return
        try:
            status, response = submit(action, payload, self.headers)
        except (TypeError, ValueError, RuntimeError, OSError, KeyError,
                AttributeError) as exc:
            self._json(
                {'accepted': False, 'message': f'调试处理失败: {exc}'},
                HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        try:
            http_status = HTTPStatus(status)
        except ValueError:
            http_status = HTTPStatus.INTERNAL_SERVER_ERROR
        self._json(response, http_status)


    def _post_params(self):
        """POST /api/params：解析 body → backend.params_set → 审计/门控在后端."""
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except (TypeError, ValueError):
            self._json({'accepted': False, 'message': 'Content-Length 非法'},
                       HTTPStatus.BAD_REQUEST)
            return
        if length < 0 or length > _MAX_BODY_BYTES:
            self._json({'accepted': False, 'message': '请求体大小非法'},
                       HTTPStatus.REQUEST_ENTITY_TOO_LARGE if length > 0 else HTTPStatus.BAD_REQUEST)
            return
        try:
            raw = self.rfile.read(length) if length else b'{}'
            payload = json.loads(raw or b'{}')
        except ValueError:
            self._json({'accepted': False, 'message': '请求体不是合法 JSON'},
                       HTTPStatus.BAD_REQUEST)
            return
        if not isinstance(payload, dict):
            self._json({'accepted': False, 'message': '请求体须为 JSON 对象'},
                       HTTPStatus.BAD_REQUEST)
            return
        submit = getattr(self.server.backend, 'params_set', None)
        if not callable(submit):
            self._json({'accepted': False, 'message': '参数面板未启用'},
                       HTTPStatus.SERVICE_UNAVAILABLE)
            return
        try:
            status, response = submit(payload)
        except (TypeError, ValueError, RuntimeError, OSError) as exc:
            self._json({'accepted': False, 'message': f'参数处理失败: {exc}'},
                       HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        try:
            http_status = HTTPStatus(status)
        except ValueError:
            http_status = HTTPStatus.INTERNAL_SERVER_ERROR
        self._json(response, http_status)


class _ObservabilityHttpServer(ThreadingHTTPServer):
    """携带窄接口后端与静态根目录的 HTTP 服务（Handler 经 server 取依赖）."""

    def __init__(self, server_address, backend: HttpBackend,
                 web_root: Path, log_debug):
        """记录后端/静态根/日志回调；属性须先于基类 bind 就绪."""
        self.backend = backend
        self.web_root = Path(web_root)
        self.log_debug = log_debug
        super().__init__(server_address, ObservabilityHttpHandler)


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
    server = _ObservabilityHttpServer(
        (host, port), backend, Path(web_root), log_debug)
    thread = threading.Thread(
        target=server.serve_forever,
        name='peach-observability-http', daemon=True)
    thread.start()
    return server
