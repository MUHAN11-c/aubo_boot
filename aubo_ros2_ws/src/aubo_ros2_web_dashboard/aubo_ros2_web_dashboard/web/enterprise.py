"""企业级横切能力：请求追踪、安全响应头、耗时、可配置 CORS。"""

from __future__ import annotations

import logging
import os
import time
import uuid
from typing import Callable

from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response

ACCESS_LOG = logging.getLogger("aubo_ros2_web.access")


def cors_allow_origins() -> list[str]:
    """读取 AUBO_WEB_CORS_ORIGINS：逗号分隔源列表，或默认 *。"""
    raw = os.environ.get("AUBO_WEB_CORS_ORIGINS", "*").strip()
    if raw == "*":
        return ["*"]
    return [x.strip() for x in raw.split(",") if x.strip()]


class RequestIdMiddleware(BaseHTTPMiddleware):
    """为每个请求分配或透传 X-Request-ID，便于日志与排障串联。"""

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        rid = request.headers.get("x-request-id") or str(uuid.uuid4())
        request.state.request_id = rid
        response = await call_next(request)
        response.headers["X-Request-ID"] = rid
        return response


class ProcessTimeMiddleware(BaseHTTPMiddleware):
    """响应头 X-Process-Time：服务端处理耗时（秒，近似）。"""

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        t0 = time.perf_counter()
        response = await call_next(request)
        dt = time.perf_counter() - t0
        response.headers["X-Process-Time"] = f"{dt:.4f}"
        if os.environ.get("AUBO_WEB_ACCESS_LOG", "").lower() in ("1", "true", "yes"):
            rid = getattr(request.state, "request_id", None)
            ACCESS_LOG.info(
                "%s %s %s %dms request_id=%s",
                request.method,
                request.url.path,
                response.status_code,
                int(dt * 1000),
                rid,
            )
        return response


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """基础安全响应头（反嗅探、点击劫持、引用策略）。"""

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        response = await call_next(request)
        h = response.headers
        h.setdefault("X-Content-Type-Options", "nosniff")
        h.setdefault("X-Frame-Options", "SAMEORIGIN")
        h.setdefault("Referrer-Policy", "strict-origin-when-cross-origin")
        h.setdefault(
            "Permissions-Policy",
            "accelerometer=(), camera=(), geolocation=(), gyroscope=(), magnetometer=(), microphone=(), payment=(), usb=()",
        )
        return response
