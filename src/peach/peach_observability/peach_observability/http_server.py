"""FastAPI/uvicorn Web API for the Peach observability console."""
from __future__ import annotations

import json
from pathlib import Path
import socket
import threading
from typing import Protocol

from ament_index_python.packages import get_package_share_path, PackageNotFoundError
from fastapi import FastAPI, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, JSONResponse, Response
import uvicorn

_MAX_BODY_BYTES = 65536


class HttpBackend(Protocol):
    """Narrow backend interface implemented by ObservabilityNode."""

    def snapshot(self) -> dict:
        """Return the browser state snapshot."""
        ...

    def trajectory(self) -> dict:
        """Return TCP points, markers and landmarks."""
        ...

    def image(self, stream: str) -> tuple:
        """Return encoded JPEG bytes and frame metadata."""
        ...

    def debug_command(self, action: str, payload: dict, headers) -> tuple:
        """Execute one allowlisted debug action; return (status, body)."""
        ...

    def params_report(self) -> dict:
        """Return allowlisted parameter values."""
        ...

    def params_set(self, payload: dict) -> tuple:
        """Apply one parameter change request; return (status, body)."""
        ...


_ALIASES = {
    '/': 'overview.html',
    '/index.html': 'monitor.html',
    '/monitor.html': 'monitor.html',
}
_WEB_LINK_ASSETS = {
    '/vendor/ros2-web-link.js': ('dist/web/ros2-web-link.js', 'text/javascript'),
    '/vendor/ros2-web-link-LICENSE.txt': ('LICENSE', 'text/plain'),
    '/vendor/THIRD_PARTY_NOTICES.txt': ('dist/web/THIRD_PARTY_NOTICES.txt', 'text/plain'),
}
# Only explicit web asset types are served. No directory listing, no dotfiles,
# no source maps, no user-selected path. New static modules are picked up
# without editing a second hard-coded route table.
_CONTENT_TYPES = {
    '.html': 'text/html',
    '.css': 'text/css',
    '.js': 'text/javascript',
    '.json': 'application/json',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.jpeg': 'image/jpeg',
    '.webp': 'image/webp',
    '.ico': 'image/x-icon',
    '.woff2': 'font/woff2',
    '.txt': 'text/plain',
}
_CSP = (
    "default-src 'self'; img-src 'self' blob:; style-src 'self' "
    "'unsafe-inline'; object-src 'none'; base-uri 'self'; "
    "connect-src 'self' ws://127.0.0.1:* ws://localhost:*; "
    "form-action 'self'; frame-ancestors 'none'"
)


def _status(value) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 500


def _json(status: int, value) -> JSONResponse:
    return JSONResponse(value, status_code=_status(status))


def _headers(request: Request) -> dict:
    return {key.lower(): value for key, value in request.headers.items()}


def _apply_security(response: Response) -> Response:
    response.headers.setdefault('X-Content-Type-Options', 'nosniff')
    response.headers.setdefault('X-Frame-Options', 'DENY')
    response.headers.setdefault('Content-Security-Policy', _CSP)
    response.headers.setdefault('Referrer-Policy', 'no-referrer')
    return response


def _resolve_asset(root: Path, url_path: str):
    """
    Map a same-origin URL path to a regular file under *root*.

    Returns ``(path, media_type)`` or ``None``. Aliases preserve the legacy
    ``/`` (overview) and ``/index.html`` (monitor) entry points. Symlinked
    files are allowed because colcon's symlink install exposes ``share/``
    entries this way; URL traversal and dotfiles are rejected lexically
    before any filesystem lookup.
    """
    relative = _ALIASES.get(url_path)
    if relative is None:
        relative = url_path.lstrip('/')
    parts = Path(relative).parts
    if not relative or any(part in ('..', '') or part.startswith('.') for part in parts):
        return None
    target = root / relative
    if not target.is_file():
        return None
    media_type = _CONTENT_TYPES.get(target.suffix.lower())
    if media_type is None:
        return None
    return target, media_type


def create_app(web_root, backend: HttpBackend) -> FastAPI:
    """Build the same-origin API and static app for one observability node."""
    root = Path(web_root)
    app = FastAPI(title='Peach observability', docs_url=None, redoc_url=None)

    @app.middleware('http')
    async def _security(request: Request, call_next):
        response = await call_next(request)
        response.headers.setdefault('Cache-Control', 'no-store')
        return _apply_security(response)

    @app.get('/api/state')
    def state():
        return _json(200, backend.snapshot())

    @app.get('/api/transport')
    def transport():
        getter = getattr(backend, 'transport_config', None)
        return _json(200, getter() if callable(getter) else {'default': 'http'})

    @app.get('/api/params')
    def params_report():
        getter = getattr(backend, 'params_report', None)
        return _json(200, getter() if callable(getter) else {'groups': []})

    @app.get('/api/force')
    def force():
        getter = getattr(backend, 'force', None)
        return _json(200, getter() if callable(getter) else {'available': False})

    @app.get('/api/trajectory')
    def trajectory():
        getter = getattr(backend, 'trajectory', None)
        return _json(200, getter() if callable(getter) else {})

    @app.get('/api/analysis/success')
    def analysis_success():
        getter = getattr(backend, 'analysis_success', None)
        return _json(200, getter() if callable(getter) else {
            'today': {}, 'all': {}, 'sessions': []})

    @app.get('/api/analysis/records')
    def analysis_records():
        getter = getattr(backend, 'analysis_records', None)
        return _json(200, getter() if callable(getter) else {'records': []})

    @app.get('/api/analysis')
    def analysis_report(path: str = ''):
        getter = getattr(backend, 'analysis_report', None)
        if not callable(getter):
            return _json(404, {'error': 'analysis_unavailable'})
        try:
            return _json(200, getter(path))
        except ValueError as error:
            return _json(400, {'error': str(error)})

    @app.get('/api/analysis/image')
    def analysis_image(path: str = '', name: str = ''):
        getter = getattr(backend, 'analysis_image', None)
        data = getter(path, name) if callable(getter) else None
        if not data:
            return _json(404, {'error': 'no_image'})
        kind = 'image/png' if name.lower().endswith('.png') else 'image/jpeg'
        return Response(data, media_type=kind)

    @app.get('/api/images/perception.jpg')
    def perception_image():
        return _image(backend, 'perception')

    @app.get('/api/images/raw.jpg')
    def raw_image():
        return _image(backend, 'raw')

    @app.post('/api/params')
    async def params_set(request: Request):
        payload, error = await _body(request)
        if error is not None:
            return error
        submit = getattr(backend, 'params_set', None)
        if not callable(submit):
            return _json(503, {'accepted': False, 'message': '参数面板未启用'})
        try:
            status, response = await run_in_threadpool(submit, payload)
        except (TypeError, ValueError, RuntimeError, OSError) as exc:
            return _json(500, {'accepted': False, 'message': f'参数处理失败: {exc}'})
        return _json(status, response)

    @app.post('/api/debug/{action:path}')
    async def debug_action(action: str, request: Request):
        action = action.strip('/')
        if not action:
            return _json(404, {'accepted': False, 'message': '缺少调试端点'})
        payload, error = await _body(request)
        if error is not None:
            return error
        submit = getattr(backend, 'debug_command', None)
        if not callable(submit):
            return _json(503, {'accepted': False, 'message': '调试操作面未启用'})
        try:
            status, response = await run_in_threadpool(
                submit, action, payload, _headers(request))
        except (TypeError, ValueError, RuntimeError, OSError, KeyError, AttributeError) as exc:
            return _json(500, {'accepted': False, 'message': f'调试处理失败: {exc}'})
        return _json(status, response)

    @app.get('/{asset_path:path}')
    def static_asset(asset_path: str):
        path = '/' + asset_path if asset_path else '/'
        if path in _WEB_LINK_ASSETS:
            relative, media_type = _WEB_LINK_ASSETS[path]
            try:
                target = get_package_share_path('ros2_web_link') / relative
            except PackageNotFoundError:
                return _json(503, {'error': 'ros2_web_link_not_installed'})
            if not target.is_file():
                return _json(503, {'error': 'ros2_web_link_asset_not_installed'})
        else:
            asset = _resolve_asset(root, path)
            if asset is None:
                return _json(404, {'error': 'asset_unavailable'})
            target, media_type = asset
        return FileResponse(target, media_type=media_type, headers={'Cache-Control': 'no-cache'})

    return app


def _image(backend: HttpBackend, stream: str) -> Response:
    getter = getattr(backend, 'image', None)
    data, metadata = getter(stream) if callable(getter) else (
        None, {'available': False, 'error': 'preview_unavailable'})
    if data is None or metadata.get('error') == 'clock_mismatch':
        return _json(404, metadata)
    response = Response(data, media_type='image/jpeg')
    response.headers['X-Frame-Stamp-Ns'] = str(metadata['stamp_ns'])
    response.headers['X-Frame-Id'] = str(metadata['frame_id'])
    response.headers['X-Frame-Age-S'] = str(metadata['age_s'])
    return response


async def _body(request: Request):
    raw = bytearray()
    async for chunk in request.stream():
        if len(raw) + len(chunk) > _MAX_BODY_BYTES:
            return None, _json(413, {'accepted': False, 'message': '请求体过大'})
        raw.extend(chunk)
    try:
        payload = json.loads(raw or b'{}')
    except ValueError:
        return None, _json(400, {'accepted': False, 'message': '请求体不是合法 JSON'})
    if not isinstance(payload, dict):
        return None, _json(400, {'accepted': False, 'message': '请求体须为 JSON 对象'})
    return payload, None


class HttpServerHandle:
    """Uvicorn server plus its already-bound listening socket."""

    def __init__(self, server, thread, sock):
        """Store server, daemon thread and bound socket."""
        self._server = server
        self._thread = thread
        self._sock = sock
        self.server_address = sock.getsockname()

    def shutdown(self) -> None:
        """Ask uvicorn to stop and wait briefly for the thread."""
        self._server.should_exit = True
        self._thread.join(timeout=5.0)

    def server_close(self) -> None:
        """Close the bound socket after shutdown."""
        try:
            self._sock.close()
        except OSError:
            pass


def start_http(host: str, port: int, web_root, backend: HttpBackend,
               log_debug) -> HttpServerHandle:
    """Bind a socket synchronously, then run FastAPI/uvicorn in a daemon thread."""
    del log_debug
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(128)
    app = create_app(web_root, backend)
    config = uvicorn.Config(
        app, log_level='warning', access_log=False, lifespan='off')
    server = uvicorn.Server(config)
    thread = threading.Thread(
        target=server.run, kwargs={'sockets': [sock]},
        name='peach-observability-http', daemon=True)
    thread.start()
    return HttpServerHandle(server, thread, sock)
