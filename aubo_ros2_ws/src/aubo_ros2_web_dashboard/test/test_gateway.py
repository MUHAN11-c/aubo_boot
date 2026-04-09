"""FastAPI 网关：健康检查、runtime-config 与非法代理路径。"""
from __future__ import annotations

from urllib.parse import quote

import pytest
from fastapi.testclient import TestClient

from aubo_ros2_web_dashboard.gateway.app import create_app


@pytest.fixture
def web_root(tmp_path):
	d = tmp_path / "web"
	d.mkdir()
	(d / "index.html").write_text("ok", encoding="utf-8")
	return str(d)


@pytest.fixture
def client(web_root):
	return TestClient(create_app(web_root))


def test_health(client):
	r = client.get("/health")
	assert r.status_code == 200
	j = r.json()
	assert j.get("status") == "ok"
	assert "static_root" in j


def test_runtime_config_unified_proxy(client):
	r = client.get("/api/ivg/runtime-config")
	assert r.status_code == 200
	j = r.json()
	assert j.get("unified_proxy") is True
	assert j.get("rosbridge_ws_path") == "/ws/rosbridge"
	assert j.get("web_video_proxy_prefix") == "/api/ivg/proxy/web-video"


def test_web_video_proxy_rejects_path_traversal(client):
	# 避免 TestClient 在发送前规范化掉 ``..``，对路径段做百分号编码
	evil = quote("snapshot/../../../etc/passwd", safe="")
	r = client.get(f"/api/ivg/proxy/web-video/{evil}")
	assert r.status_code == 400


def test_web_video_upstream_unreachable_502(client, monkeypatch):
	monkeypatch.setenv("IVG_WEB_VIDEO_PORT", "1")
	monkeypatch.setenv("IVG_PROXY_VIDEO_CONNECT_TIMEOUT", "3")
	r = client.get("/api/ivg/proxy/web-video/stream")
	assert r.status_code == 502
	assert "unreachable" in (r.json().get("detail") or "").lower()
