from http import HTTPStatus
import json
from pathlib import Path
import urllib.error
import urllib.request

from peach2_observability.http_server import HttpServer


class _FakeBackend:
    def snapshot(self):
        return {'read_only': True, 'task': None}

    def diagnostics(self):
        return {'statuses': []}

    def ledger(self, request_id: str):
        return {'request_id': request_id}

    def log_debug(self, message: str):
        pass


def test_get_routes_and_post_rejected(tmp_path: Path):
    web = tmp_path / 'web'
    web.mkdir()
    (web / 'index.html').write_text('<html>ok</html>', encoding='utf-8')
    backend = _FakeBackend()
    server = HttpServer('127.0.0.1', 0, backend, web)
    server.start()
    port = server._httpd.server_address[1]  # noqa: SLF001
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state') as resp:
            body = json.loads(resp.read().decode('utf-8'))
        assert body['read_only'] is True

        req = urllib.request.Request(
            f'http://127.0.0.1:{port}/api/debug/run', data=b'{}', method='POST')
        try:
            urllib.request.urlopen(req)
        except urllib.error.HTTPError as error:
            assert error.code == HTTPStatus.METHOD_NOT_ALLOWED
        else:
            raise AssertionError('POST should fail')
    finally:
        server.stop()


def test_ledger_route():
    web = Path('/tmp/peach2_obs_web')
    web.mkdir(exist_ok=True)
    (web / 'index.html').write_text('x', encoding='utf-8')
    backend = _FakeBackend()
    server = HttpServer('127.0.0.1', 0, backend, web)
    server.start()
    port = server._httpd.server_address[1]  # noqa: SLF001
    try:
        with urllib.request.urlopen(
                f'http://127.0.0.1:{port}/api/ledger/batch_x') as resp:
            body = json.loads(resp.read().decode('utf-8'))
        assert body['request_id'] == 'batch_x'
    finally:
        server.stop()
