"""
浏览器运行时配置 BFF。

完整路径：``GET /api/v1/runtime``。
响应体由 ``settings.runtime_config_dict`` 生成，与当前进程环境变量一致（每次请求现读 ``os.environ``）。
"""
from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Request

from aubo_ros2_web_dashboard.gateway import settings as cfg

router = APIRouter(prefix="/api/v1", tags=["ivg"])


@router.get("/runtime")
async def runtime_v1(request: Request) -> dict[str, Any]:
	"""供前端 ``ivg_transport.loadRuntime`` 单次拉取运行时配置。"""
	return cfg.runtime_config_dict(request.app.state.static_root)
