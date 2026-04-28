"""BFF 运行时接口 — GET /api/v1/runtime + POST /api/v1/settings。

GET  返回网关端口、视频代理前缀、话题/服务定义（含分类 label）
POST 将前端设置页提交的值写入 config/defaults.yaml 并更新内存配置
"""
from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, Request

from aubo_ros2_web_dashboard import config as cfg

router = APIRouter(prefix="/api/v1", tags=["ivg"])


@router.get("/runtime")
async def runtime_v1(request: Request) -> dict[str, Any]:
    return cfg.runtime_config_dict(request.app.state.static_root)


@router.post("/settings")
async def settings_save(request: Request) -> dict[str, Any]:
    """保存设置到 YAML 配置文件。请求体为 {id: value} 字典。"""
    try:
        body = await request.json()
    except Exception:
        raise HTTPException(status_code=400, detail="请求体必须是 JSON 对象")

    if not isinstance(body, dict) or not body:
        raise HTTPException(status_code=400, detail="请求体不能为空")

    ok = cfg.save_settings_to_yaml(body)
    if not ok:
        raise HTTPException(status_code=500, detail="写入 YAML 失败（文件不可写）")

    return {"status": "ok", "saved": len(body)}
