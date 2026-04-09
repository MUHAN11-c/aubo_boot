"""
浏览器运行时配置 BFF。

完整路径：``GET /api/ivg/runtime-config``。
响应体由 ``settings.runtime_config_dict`` 生成，与当前进程环境变量一致（每次请求现读 ``os.environ``）。
"""
from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Request

from aubo_ros2_web_dashboard.gateway import settings as cfg

router = APIRouter(prefix="/api/ivg", tags=["ivg"])


@router.get("/runtime-config")
async def runtime_config(request: Request) -> dict[str, Any]:
	return cfg.runtime_config_dict(request.app.state.static_root)
