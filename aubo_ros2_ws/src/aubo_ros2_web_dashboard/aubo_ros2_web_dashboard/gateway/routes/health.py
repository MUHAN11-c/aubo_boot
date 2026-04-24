"""
探活与静态根路径回显（运维 / k8s / 监控）。

完整路径：``GET /health``（无 prefix）。
"""
from __future__ import annotations

from fastapi import APIRouter, Request

router = APIRouter(tags=["ops"])


@router.get("/health")
async def health(request: Request) -> dict[str, str]:
	"""返回进程健康状态与当前静态站点根目录（供运维与前端排障对照）。"""
	return {"status": "ok", "static_root": request.app.state.static_root}
