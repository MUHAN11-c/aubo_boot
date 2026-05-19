"""工具几何数据服务 — GET /api/v1/tool-geometries。

从 tool_changer 包的 tools.yaml 读取工具定义，
返回 Web 前端 Robot3dViewer 所需的数据（mesh 代理 URL + attach_offset）。

优先读取 tool_changer 包的 tools.yaml，不可用时使用硬编码回退数据。
回退数据与 tools.yaml 保持同步，消除前端 TOOL_MODELS 硬编码的 DRY 违规喵~
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

try:
    import yaml as _yaml
    _HAS_YAML = True
except ImportError:
    _yaml = None
    _HAS_YAML = False

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from fastapi import APIRouter

router = APIRouter(prefix="/api/v1", tags=["ivg-tool-geometries"])

# ── 硬编码回退数据（与 tools.yaml 保持同步，YAML 不可用时使用）─────────────────
# 格式与 tools.yaml 的 tools 段一致喵~
_FALLBACK_TOOLS: dict[str, Any] = {
    "gripper0": {
        "name": "气动夹爪 φ40",
        "type": "gripper",
        "mesh_visual": "aubo_description/meshes/visual/gripper0_link.stl",
        "attach_offset": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.033},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    },
    "gripper1": {
        "name": "电动夹爪 A",
        "type": "gripper",
        "mesh_visual": "aubo_description/meshes/visual/gripper1_link.stl",
        "attach_offset": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.033},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    },
    "gripper2": {
        "name": "电动夹爪 φ60",
        "type": "gripper",
        "mesh_visual": "aubo_description/meshes/visual/gripper2_link.stl",
        "attach_offset": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.033},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071},
        },
    },
    "gripper1coffeecup": {
        "name": "咖啡杯工具",
        "type": "other",
        "mesh_visual": "aubo_description/meshes/visual/gripper1coffeecup_link.stl",
        "attach_offset": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.033},
            "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0},
        },
    },
    "gripper1milkcup": {
        "name": "牛奶杯工具",
        "type": "other",
        "mesh_visual": "aubo_description/meshes/visual/gripper1milkcup_link.stl",
        "attach_offset": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.033},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.7071, "w": 0.7071},
        },
    },
}


def _load_tools_yaml() -> dict[str, Any] | None:
    """尝试从 tool_changer 包的 share 目录读取 tools.yaml 的 tools 段。"""
    try:
        share = Path(get_package_share_directory("tool_changer"))
        yaml_path = share / "config" / "tools.yaml"
        if yaml_path.is_file() and _HAS_YAML and _yaml is not None:
            raw = yaml_path.read_text(encoding="utf-8")
            data = _yaml.safe_load(raw)
            if isinstance(data, dict) and "tools" in data:
                return data["tools"]
    except (PackageNotFoundError, Exception):
        pass
    return None


def _convert_mesh_url(mesh_visual: str) -> str:
    """将 tools.yaml 的 package:// URI 转换为代理 URL。

    "package://aubo_description/meshes/visual/gripper0_link.stl"
    → "/api/ivg/robot-mesh/aubo_description/meshes/visual/gripper0_link.stl"
    """
    v = mesh_visual.strip()
    if v.startswith("package://"):
        v = v[len("package://"):]
    # 确保不以 / 开头（由代理前缀提供）
    v = v.lstrip("/")
    return f"/api/ivg/robot-mesh/{v}"


def _build_geometries(tools: dict[str, Any]) -> dict[str, Any]:
    """将 tools.yaml 格式转换为前端期望的 JSON 格式。

    返回 { tool_id: { name, type, mesh_url, attach_offset: { position: [x,y,z], orientation: [x,y,z,w] } } }
    """
    result: dict[str, Any] = {}
    for tool_id, info in tools.items():
        if not isinstance(info, dict):
            continue
        attach = info.get("attach_offset", {})
        pos = attach.get("position", {})
        ori = attach.get("orientation", {})
        result[tool_id] = {
            "name": info.get("name", tool_id),
            "type": info.get("type", "gripper"),
            "mesh_url": _convert_mesh_url(info.get("mesh_visual", "")),
            "attach_offset": {
                "position": [
                    pos.get("x", 0.0),
                    pos.get("y", 0.0),
                    pos.get("z", 0.0),
                ],
                "orientation": [
                    ori.get("x", 0.0),
                    ori.get("y", 0.0),
                    ori.get("z", 0.0),
                    ori.get("w", 1.0),
                ],
            },
        }
    return result


@router.get("/tool-geometries")
async def tool_geometries() -> dict[str, Any]:
    """返回所有工具的几何数据（mesh URL + attach_offset）。

    优先从 tool_changer 包的 tools.yaml 读取；
    YAML 不可用时使用硬编码回退数据（与 tools.yaml 当前内容一致）。
    """
    tools = _load_tools_yaml()
    if tools is None:
        tools = _FALLBACK_TOOLS
    return _build_geometries(tools)
