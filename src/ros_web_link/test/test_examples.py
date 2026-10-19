"""examples/ 可运行性门：--help + comm fixture 上的真实调用.

只用本包 comm fixture（无仿真/无 GPU），验证：
  01 subscribe / 02 publish+service / 03 params+graph / 05 raw serverInfo。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip('websockets')
pytest.importorskip('rclpy')
pytest.importorskip('fastapi')

from .e2e_harness import Stack  # noqa: E402

_PKG_ROOT = Path(__file__).resolve().parent.parent
_EX = _PKG_ROOT / 'examples' / 'python'


def _env() -> dict:
    env = dict(os.environ)
    env['PYTHONPATH'] = str(_PKG_ROOT) + os.pathsep + env.get('PYTHONPATH', '')
    return env


def _run(script: str, *args: str, timeout: float = 40.0):
    proc = subprocess.run(
        [sys.executable, str(_EX / script), *args],
        capture_output=True, text=True, timeout=timeout, env=_env())
    return proc


def _json(proc: subprocess.CompletedProcess) -> dict:
    assert proc.returncode == 0, f'{proc.stderr}\n{proc.stdout}'
    lines = [ln for ln in proc.stdout.splitlines() if ln.strip()]
    return json.loads(lines[-1])


@pytest.fixture(scope='module')
def stack():
    st = Stack(fixtures=('ros_web_link.comm_fixture',))
    try:
        st.start()
    except Exception:
        st.stop()
        raise
    yield st
    st.stop()


@pytest.mark.parametrize('script', [
    '01_subscribe.py', '02_publish_and_service.py',
    '03_params_and_graph.py', '04_embed_gateway.py', '05_raw_protocol.py',
])
def test_example_help(script):
    proc = _run(script, '--help', timeout=15)
    assert proc.returncode == 0, proc.stderr
    assert 'usage:' in proc.stdout


def test_example_subscribe(stack):
    proc = _run('01_subscribe.py', '--url', stack.ws_url,
                '--topic', '/rwl_demo/chatter', '--count', '2', '--json')
    data = _json(proc)
    assert data['topic'] == '/rwl_demo/chatter'
    assert data['count'] == 2
    assert data['messages']


def test_example_publish_and_service(stack):
    proc = _run('02_publish_and_service.py', '--url', stack.ws_url, '--json')
    data = _json(proc)
    assert data['published']['data'] in data['echo_reply']['data']
    assert data['service']['response']['sum'] == 42


def test_example_params_and_graph(stack):
    proc = _run('03_params_and_graph.py', '--url', stack.ws_url, '--json')
    data = _json(proc)
    assert data['set']['successful'] is True
    assert data['params_after'][data['set']['name']] == data['set']['value']
    assert data['graph']['topics'], data['graph']


def test_example_raw_protocol(stack):
    proc = _run('05_raw_protocol.py', '--url', stack.ws_url, '--json')
    data = _json(proc)
    assert data['op'] == 'serverInfo'
    assert data['subprotocol'] == 'foxglove.websocket.v1'
    assert isinstance(data['capabilities'], list)
