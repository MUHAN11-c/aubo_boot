"""FastAPI tests without rosbridge / VPE（TestClient）。"""

from __future__ import annotations

from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from aubo_ros2_web_dashboard.web.app import create_app
from aubo_ros2_web_dashboard.web.dependencies import get_runtime_config
from aubo_ros2_web_dashboard.web.resources import WebPaths


@pytest.fixture(autouse=True)
def _clear_runtime_config_cache():
    """避免 get_runtime_config 的 lru_cache 在多个用例间保留旧环境。"""
    get_runtime_config.cache_clear()
    yield
    get_runtime_config.cache_clear()


@pytest.fixture()
def client(monkeypatch):
    """使用源码树内的 frontend/dist，且不挂载 VPE legacy，便于纯 HTTP 单测。"""
    root = Path(__file__).resolve().parents[1]
    dist = root / "frontend" / "dist"

    def _paths() -> WebPaths:
        return WebPaths(dist_dir=dist)

    monkeypatch.setattr("aubo_ros2_web_dashboard.web.app.resolve_web_paths", _paths)
    monkeypatch.setattr("aubo_ros2_web_dashboard.web.app.resolve_vpe_web_paths", lambda: None)
    app = create_app()
    with TestClient(app, raise_server_exceptions=True) as tc:
        yield tc


def test_gateway_health(client: TestClient) -> None:
    r = client.get("/gateway/health")
    assert r.status_code == 200
    assert r.headers.get("X-Request-ID")
    j = r.json()
    assert j.get("ok") is True
    assert j.get("request_id") == r.headers.get("X-Request-ID")


def test_gateway_ready_probe_disabled(client: TestClient) -> None:
    r = client.get("/gateway/ready")
    assert r.status_code == 200
    assert r.json().get("probe") == "disabled"


def test_api_config(client: TestClient, monkeypatch) -> None:
    monkeypatch.setenv("AUBO_WEB_ROSBRIDGE_PORT", "9091")
    monkeypatch.setenv("AUBO_WEB_PUBLIC_HOST", "")
    monkeypatch.setenv("AUBO_WEB_USE_SIM_TIME", "false")
    monkeypatch.setenv("AUBO_WEB_FIXED_FRAME", "base_link")
    r = client.get("/api/config")
    assert r.status_code == 200
    j = r.json()
    assert j["rosbridge_port"] == 9091
    assert "ivg_presets" in j


def test_root_redirects_to_rwt_when_no_legacy(client: TestClient) -> None:
    r = client.get("/rwt/", follow_redirects=False)
    assert r.status_code == 200
    assert "html" in r.headers.get("content-type", "").lower()
    assert "演示" in r.text and "开发调试" in r.text

    r_demo = client.get("/rwt/demo.html", follow_redirects=False)
    assert r_demo.status_code == 200
    assert "data-rwt-mode" in r_demo.text and "demo" in r_demo.text

    r_dev = client.get("/rwt/dev.html", follow_redirects=False)
    assert r_dev.status_code == 200
    assert 'data-rwt-mode="dev"' in r_dev.text

    r2 = client.get("/", follow_redirects=True)
    assert r2.status_code == 200
    assert "html" in r2.headers.get("content-type", "").lower()


def test_health_proxies_or_502(client: TestClient) -> None:
    """未启动 VPE 时 /health 为 502；若本机已开 8088 则可能为 200。"""
    r = client.get("/health")
    assert r.status_code in (200, 502)
    if r.status_code == 502 and r.content:
        try:
            j = r.json()
        except ValueError:
            pass
        else:
            if isinstance(j, dict) and "error" in j:
                assert j["error"].get("code")
