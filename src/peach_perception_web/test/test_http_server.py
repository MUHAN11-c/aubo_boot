# Copyright 2026 wjz
"""HTTP 层只读行为测试：真实 server + fake 窄接口后端."""

from __future__ import annotations

import http.client
import json

from peach_perception_web.http_server import start_http
import pytest


class _FakeBackend:
    """HttpBackend 内存假实现：只提供状态快照."""

    def __init__(self, revision=7):
        """固定快照内容供断言."""
        self._revision = revision
        self.port = 0

    def snapshot(self):
        """最小状态快照（含 params 镜像区段）."""
        return {
            'system': {'revision': self._revision},
            'params': {'/peach_pose_node': {'yolo_conf': 0.3}},
        }


@pytest.fixture()
def backend(tmp_path):
    """起 127.0.0.1 随机端口的真实 HTTP 服务（fake 后端），用后关闭."""
    for name in ('index.html', 'app.css', 'app.js'):
        (tmp_path / name).write_text('x', encoding='utf-8')
    fake = _FakeBackend()
    server = start_http('127.0.0.1', 0, tmp_path, fake, lambda msg: None)
    fake.port = server.server_address[1]
    yield fake
    server.shutdown()
    server.server_close()


def _request(port, method, path, body=None, headers=None):
    """发一次请求，返回 (status, headers dict, body bytes)."""
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    try:
        if body is None:
            connection.request(method, path, headers=headers or {})
        else:
            merged = {'Content-Type': 'application/json'}
            merged.update(headers or {})
            connection.request(
                method, path, body=json.dumps(body), headers=merged)
        response = connection.getresponse()
        data = response.read()
        return response.status, dict(response.getheaders()), data
    finally:
        connection.close()


def test_get_index_and_assets(backend):
    """首页与静态资源可读，带安全响应头."""
    for path, prefix in (('/', 'text/html'), ('/app.css', 'text/css'),
                         ('/app.js', 'text/javascript')):
        status, headers, data = _request(backend.port, 'GET', path)
        assert status == 200
        assert headers['Content-Type'].startswith(prefix)
        assert headers['X-Frame-Options'] == 'DENY'
        assert data == b'x'


def test_get_state_snapshot(backend):
    """状态快照含系统 revision 与参数镜像区段."""
    status, _headers, data = _request(backend.port, 'GET', '/api/state')
    assert status == 200
    payload = json.loads(data)
    assert payload['system']['revision'] == 7
    assert payload['params']['/peach_pose_node']['yolo_conf'] == 0.3


def test_get_unknown_path_404(backend):
    """未知 GET 路径 404."""
    status, _headers, _data = _request(backend.port, 'GET', '/nope')
    assert status == 404


def test_post_always_method_not_allowed(backend):
    """只读监控台：一切 POST 统一 405（含旧控制/调试/策略路径）."""
    for path in ('/api/control', '/api/debug', '/api/policy',
                 '/api/profiles/save', '/api/whatever'):
        status, _headers, data = _request(
            backend.port, 'POST', path, body={'command': 1})
        assert status == 405
        assert json.loads(data)['accepted'] is False
