"""aubo_ros2_web_dashboard 面板 BFF 子路由 — 注入 ros_web_link 网关.

包含 UI 面板专属的 HTTP 路由（不含传输层）：
  GET  /api/v1/runtime          — 前端运行时配置（话题/服务默认值）
  POST /api/v1/settings         — 设置写回
  GET  /api/v1/tool-geometries  — 工具几何信息
"""
from __future__ import annotations

import os
from pathlib import Path

from fastapi import APIRouter
from fastapi.responses import JSONResponse

router = APIRouter(prefix="/api/v1", tags=["dashboard-bff"])


def _get_defaults_yaml_path() -> Path:
    """定位 defaults.yaml（源树或 install）."""
    src = Path(__file__).resolve().parent.parent / "config" / "defaults.yaml"
    if src.is_file():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("aubo_ros2_web_dashboard")) / "config" / "defaults.yaml"
    except Exception:
        return src


def _load_config() -> dict:
    p = _get_defaults_yaml_path()
    if not p.is_file():
        return {}
    try:
        import yaml
        d = yaml.safe_load(p.read_text(encoding="utf-8"))
        return d if isinstance(d, dict) else {}
    except Exception:
        return {}


_cfg = _load_config()


def _g(path: str, default=None):
    c = _cfg
    for k in path.split("."):
        if isinstance(c, dict):
            c = c.get(k)
        else:
            return default
    return c if c is not None else default


@router.get("/runtime")
async def get_runtime():
    """前端运行时配置（网关端口 + 面板话题/服务默认值）."""
    from ros_web_link import config as transport_cfg

    static_root = str(Path(__file__).resolve().parent.parent / "web" / "public")
    return {
        "package": "aubo_ros2_web_dashboard",
        "version": "0.4.0",
        "rosbridge_port": None,  # 已移除
        "foxglove_bridge_port": transport_cfg.foxglove_bridge_port(),
        "static_root": static_root,
        "unified_proxy": True,
        "rosbridge_ws_path": None,  # 已移除
        "foxglove_ws_path": transport_cfg.foxglove_bridge_ws_path(),
        "web_video_port": transport_cfg.web_video_port(),
        "settings_categories": _g("vision_panel", {}),
        "web_video_proxy_prefix": "/api/ivg/proxy/web-video",
    }


@router.post("/settings")
async def post_settings(body: dict):
    """写回设置到 defaults.yaml 的 vision_panel 段."""
    p = _get_defaults_yaml_path()
    try:
        import yaml
        existing = {}
        if p.is_file():
            existing = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        for key, val in (body or {}).items():
            section = existing.setdefault("vision_panel", {})
            if isinstance(val, dict):
                section.setdefault(key, {}).update(val)
            else:
                section[key] = val
        p.write_text(yaml.dump(existing, allow_unicode=True, default_flow_style=False), encoding="utf-8")
        global _cfg
        _cfg = _load_config()
        return {"status": "ok"}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})


@router.get("/tool-geometries")
async def get_tool_geometries():
    """工具几何信息（读 tool_changer 包的 tools.yaml）."""
    try:
        from ament_index_python.packages import get_package_share_directory
        import yaml
        tools_path = Path(get_package_share_directory("tool_changer")) / "config" / "tools.yaml"
        if not tools_path.is_file():
            return {}
        return yaml.safe_load(tools_path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}


def get_bff_router() -> APIRouter:
    """供 ros_web_link 的 create_app(sub_routers=[...]) 注入."""
    return router
