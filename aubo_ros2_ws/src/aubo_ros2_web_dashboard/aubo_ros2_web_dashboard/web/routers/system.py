"""
仅在本网关进程内处理的路由（不转发到 VPE）。

与 VPE 同路径的 /api/config 在代理层被排除：浏览器应始终访问本网关的 GET /api/config 获取 rosbridge 端口等。
"""

from __future__ import annotations

import os

import httpx
from fastapi import APIRouter, Depends, HTTPException, Request

from ..dependencies import WebRuntimeConfig, get_runtime_config

router = APIRouter(tags=["gateway_rwt"])


@router.get("/gateway/health")
def gateway_health(request: Request) -> dict:
    """本进程存活探测（不访问上游）；编排可用 Liveness。"""
    rid = getattr(request.state, "request_id", None)
    return {"ok": True, "service": "aubo_ros2_web_gateway", "request_id": rid}


@router.get("/gateway/ready")
async def gateway_ready(request: Request) -> dict:
    """可选 Readiness：当 `AUBO_WEB_PROBE_VPE=1` 时探测 VPE `/health`，失败返回 503。"""
    rid = getattr(request.state, "request_id", None)
    probe = os.environ.get("AUBO_WEB_PROBE_VPE", "").lower() in ("1", "true", "yes")
    if not probe:
        return {"ok": True, "probe": "disabled", "request_id": rid}

    client: httpx.AsyncClient = request.app.state.http_client
    base = os.environ.get("AUBO_VPE_UPSTREAM", "http://127.0.0.1:8088").rstrip("/")
    try:
        r = await client.get(f"{base}/health", timeout=httpx.Timeout(2.0))
        ok = r.status_code < 500
    except httpx.RequestError:
        ok = False
    if not ok:
        raise HTTPException(
            status_code=503,
            detail=f"上游 VPE 不可达: {base}",
        )
    return {"ok": True, "probe": "ok", "vpe_upstream": base, "request_id": rid}


@router.get("/api/config")
def api_config(cfg: WebRuntimeConfig = Depends(get_runtime_config)) -> dict:
    """
    供 RWT 前端读取：rosbridge 端口、可选公网主机、仿真时间与 TF 提示、IVG 常用话题预设。

    浏览器一般使用 ws://{location.hostname}:{rosbridge_port}；若设置了 public_host 可改用该主机。
    """
    return {
        "rosbridge_port": cfg.rosbridge_port,
        "public_host": cfg.public_host,
        "use_sim_time": cfg.use_sim_time,
        "default_fixed_frame": cfg.default_fixed_frame,
        "ivg_presets": {
            "color_image_topic": "/camera/color/image_raw",
            "depth_image_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "depth_points_topic": "/camera/depth/points",
            "depth_registered_points_topic": "/camera/depth_registered/points",
            "grasp_markers_topic": "/grasp_markers",
            "graspnet_pointcloud_topic": "/graspnet_pointcloud",
            "joint_states_topic": "/joint_states",
            "frames": {"ee_frame": "wrist3_Link", "camera_frame": "camera_frame"},
        },
    }
