"""健康检查 — GET /health，返回服务状态和静态根目录。"""
from __future__ import annotations

from fastapi import APIRouter, Request

router = APIRouter(tags=["ops"])


@router.get("/health")
async def health(request: Request) -> dict[str, str]:
    return {"status": "ok", "static_root": request.app.state.static_root}
