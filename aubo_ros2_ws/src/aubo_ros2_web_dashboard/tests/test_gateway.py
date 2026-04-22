"""FastAPI 网关：健康检查、runtime-config 与非法代理路径。"""
from __future__ import annotations

from urllib.parse import quote

import pytest
from ament_index_python.packages import PackageNotFoundError
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


def test_runtime_v1_matches_legacy_config(client):
	r = client.get("/api/v1/runtime")
	assert r.status_code == 200
	j = r.json()
	r2 = client.get("/api/ivg/runtime-config")
	assert r2.status_code == 200
	assert j == r2.json()


def test_robot_mesh_rejects_path_traversal(client):
	# 使用百分号编码的 ``..``，避免 TestClient/Starlette 在匹配前折叠路径段
	r = client.get("/api/ivg/robot-mesh/aubo_description/meshes/%2e%2e/%2e%2e/etc/passwd")
	assert r.status_code == 400


def test_robot_mesh_resolves_case_insensitive_filename(client, monkeypatch, tmp_path):
	"""Linux 上 URDF 经浏览器规范为小写扩展名时，仍能打开 share 内大写 ``.DAE`` 文件。"""
	share = tmp_path / "aubo_description"
	vis = share / "meshes" / "aubo_e5_10" / "visual"
	vis.mkdir(parents=True)
	(vis / "link0.DAE").write_text("<dae/>", encoding="utf-8")

	def _get(pkg: str) -> str:
		if pkg == "aubo_description":
			return str(share)
		raise PackageNotFoundError()

	monkeypatch.setattr(
		"aubo_ros2_web_dashboard.gateway.routes.robot_mesh.get_package_share_directory",
		_get,
	)
	r = client.get(
		"/api/ivg/robot-mesh/aubo_description/meshes/aubo_e5_10/visual/link0.dae"
	)
	assert r.status_code == 200
	assert "collada" in (r.headers.get("content-type") or "").lower()
	assert r.text == "<dae/>"


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
