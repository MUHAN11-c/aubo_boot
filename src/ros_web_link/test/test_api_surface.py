"""公开 API / 版本 / 入口 / 资产一致性门.

不启动 ROS 图；只校验包自描述与稳定入口，防止文档/实现漂移。
"""
from __future__ import annotations

import inspect
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))


def _pkg_version() -> str:
    root = ET.parse(_PKG_ROOT / 'package.xml').getroot()
    return (root.findtext('version') or '').strip()


def test_version_single_source():
    import ros_web_link

    setup_py = (_PKG_ROOT / 'setup.py').read_text(encoding='utf-8')
    m = re.search(r"version='([^']+)'", setup_py)
    assert m, 'setup.py 缺 version'
    assert ros_web_link.__version__ == _pkg_version() == m.group(1)
    assert re.match(r'^\d+\.\d+\.\d+$', ros_web_link.__version__)


def test_create_app_signature():
    fastapi = pytest.importorskip('fastapi')
    from ros_web_link.app import create_app

    params = inspect.signature(create_app).parameters
    assert list(params) == ['js_roots', 'html_root', 'sub_routers']
    app = create_app()
    paths = {r.path for r in app.routes}
    assert '/health' in paths and '/ws/foxglove' in paths
    assert '/ws/rosbridge' not in paths
    assert fastapi is not None


def test_python_client_public_methods():
    pytest.importorskip('rclpy')
    from ros_web_link.foxglove_ws_client import (
        FoxgloveClientError, FoxgloveWsClient, Subscription)

    for name in ('connect', 'close', 'wait_channel', 'subscribe', 'advertise',
                 'publish', 'wait_service', 'call_service', 'get_params',
                 'set_param', 'subscribe_connection_graph', 'graph_snapshot'):
        assert hasattr(FoxgloveWsClient, name), name
    assert hasattr(Subscription, 'next') and hasattr(Subscription, 'cancel')
    assert issubclass(FoxgloveClientError, RuntimeError)


def test_js_barrel_exports():
    barrel = _PKG_ROOT / 'web' / 'public' / 'js' / 'ros_web_link.js'
    assert barrel.is_file()
    text = barrel.read_text(encoding='utf-8')
    for name in ('foxgloveRosClient', 'createActionClient', 'PARAM_TYPE'):
        assert name in text
    proto = _PKG_ROOT / 'web' / 'public' / 'js' / 'transport' / 'foxglove' / 'index.js'
    assert proto.is_file()


def test_examples_surface():
    examples = _PKG_ROOT / 'examples'
    for rel in ('README.md', 'python/01_subscribe.py',
                'python/02_publish_and_service.py',
                'python/03_params_and_graph.py',
                'python/04_embed_gateway.py',
                'python/05_raw_protocol.py',
                'http/README.md', 'launch/README.md', 'web/README.md'):
        assert (examples / rel).is_file(), rel
    assert (_PKG_ROOT / 'web' / 'public' / 'examples'
            / 'minimal_client.html').is_file()


def test_examples_served_by_gateway(monkeypatch):
    pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    try:
        from fastapi.testclient import TestClient
    except Exception:
        pytest.skip('TestClient 依赖不可用')
    from ros_web_link.app import create_app
    from ros_web_link.routes import health as health_mod

    async def probe_ok(key, host, port):
        return True

    monkeypatch.setattr(health_mod, '_probe_cached', probe_ok)
    with TestClient(create_app()) as client:
        page = client.get('/examples/minimal_client.html')
        assert page.status_code == 200
        assert 'ros_web_link' in page.text
        js = client.get('/examples/minimal_client.js')
        assert js.status_code == 200
        assert '/js/ros_web_link.js' in js.text


def test_health_additive_fields(monkeypatch):
    pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    try:
        from fastapi.testclient import TestClient
    except Exception:
        pytest.skip('TestClient 依赖不可用')
    from ros_web_link.app import create_app
    from ros_web_link.routes import health as health_mod

    async def probe_ok(key, host, port):
        return True

    monkeypatch.setattr(health_mod, '_probe_cached', probe_ok)
    with TestClient(create_app()) as client:
        body = client.get('/health').json()
    assert body['version'] == _pkg_version()
    assert body['protocol'] == 'foxglove.websocket.v1'
    assert body['status'] == 'ok'


def test_python_m_entrypoint_help():
    env = dict(os.environ)
    env['PYTHONPATH'] = str(_PKG_ROOT) + os.pathsep + env.get('PYTHONPATH', '')
    proc = subprocess.run(
        [sys.executable, '-m', 'ros_web_link', '--help'],
        capture_output=True, text=True, timeout=20, env=env)
    assert proc.returncode == 0, proc.stderr
    assert 'ros_web_link_gateway' in proc.stdout
