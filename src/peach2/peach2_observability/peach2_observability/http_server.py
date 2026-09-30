"""
Read-only HTTP monitor (GET only). POST/PUT/DELETE always 405.

No ROS service/action clients — backend exposes snapshot, diagnostics, and ledger read.
"""
from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Protocol
from urllib.parse import unquote, urlparse


class ReadOnlyBackend(Protocol):
    def snapshot(self) -> dict:
        ...

    def diagnostics(self) -> dict:
        ...

    def ledger(self, request_id: str) -> dict:
        ...

    def log_debug(self, message: str) -> None:
        ...


class ObservabilityHttpHandler(BaseHTTPRequestHandler):
    server: '_ObservabilityHttpServer'

    def log_message(self, fmt, *args):
        self.server.backend.log_debug(fmt % args)

    def _send(self, status, content_type, data, cache='no-store'):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', cache)
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header(
            'Content-Security-Policy',
            "default-src 'self'; object-src 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(data)

    def _json(self, value, status=HTTPStatus.OK):
        data = json.dumps(value, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        self._send(status, 'application/json; charset=utf-8', data)

    def _method_not_allowed(self):
        self._json(
            {'error': 'method not allowed', 'read_only': True},
            HTTPStatus.METHOD_NOT_ALLOWED,
        )

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path == '/api/state':
            self._json(self.server.backend.snapshot())
            return
        if path == '/api/diagnostics':
            self._json(self.server.backend.diagnostics())
            return
        if path.startswith('/api/ledger/'):
            request_id = unquote(path[len('/api/ledger/'):].strip('/'))
            self._json(self.server.backend.ledger(request_id))
            return
        assets = {
            '/': ('index.html', 'text/html; charset=utf-8'),
            '/index.html': ('index.html', 'text/html; charset=utf-8'),
        }
        asset = assets.get(path)
        if asset is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = (self.server.web_root / asset[0]).read_bytes()
        self._send(HTTPStatus.OK, asset[1], data, cache='public, max-age=60')

    def do_POST(self):
        self._method_not_allowed()

    def do_PUT(self):
        self._method_not_allowed()

    def do_DELETE(self):
        self._method_not_allowed()


class _ObservabilityHttpServer(ThreadingHTTPServer):
    def __init__(self, server_address, handler_class, backend, web_root: Path):
        self.backend = backend
        self.web_root = web_root
        super().__init__(server_address, handler_class)


class HttpServer:
    """Background ThreadingHTTPServer wrapper."""

    def __init__(self, host: str, port: int, backend: ReadOnlyBackend, web_root: Path):
        self._httpd = _ObservabilityHttpServer(
            (host, int(port)), ObservabilityHttpHandler, backend, Path(web_root))
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name='peach2-observability-http', daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        self._thread.join(timeout=5.0)
