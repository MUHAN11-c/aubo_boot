"""
视觉位姿（VPE）HTTP 反向代理。

上游地址由环境变量 AUBO_VPE_UPSTREAM 指定（launch 默认 http://127.0.0.1:8088）。
请求体与原请求头（除 hop-by-hop 头）会转发；响应按上游状态码与正文原样返回（连接失败时返回 502 JSON）。
"""

from __future__ import annotations

import logging
import os
import httpx
from fastapi import APIRouter, Request, Response

from ..error_handlers import error_json_response

LOGGER = logging.getLogger(__name__)

router = APIRouter(tags=["visual_pose_estimation_proxy"])

# 由 httpx/ASGI 处理的逐跳头，避免重复或非法转发
_SKIP_REQ_HEADERS = frozenset(
    {
        "host",
        "connection",
        "content-length",
        "transfer-encoding",
        "keep-alive",
        "accept-encoding",
    }
)
_SKIP_RESP_HEADERS = frozenset({"connection", "transfer-encoding", "keep-alive"})


def _upstream_base() -> str:
    """VPE FastAPI 根 URL，无尾部斜杠。"""
    return os.environ.get("AUBO_VPE_UPSTREAM", "http://127.0.0.1:8088").rstrip("/")


def _filter_request_headers(request: Request) -> dict[str, str]:
    """复制客户端请求头，去掉不应转发的键。"""
    out: dict[str, str] = {}
    for name, value in request.headers.items():
        if name.lower() in _SKIP_REQ_HEADERS:
            continue
        out[name] = value
    return out


def _filter_response_headers(headers: httpx.Headers) -> dict[str, str]:
    """复制上游响应头，去掉逐跳头。"""
    out: dict[str, str] = {}
    for name, value in headers.multi_items():
        if name.lower() in _SKIP_RESP_HEADERS:
            continue
        out[name] = value
    return out


async def _forward(request: Request, upstream_path: str) -> Response:
    """把当前请求转发到上游的指定路径（含 query string）。"""
    client: httpx.AsyncClient = request.app.state.http_client
    base = _upstream_base()
    url = f"{base}{upstream_path}"
    if request.url.query:
        url = f"{url}?{request.url.query}"
    body = await request.body()
    hdrs = _filter_request_headers(request)
    rid = getattr(request.state, "request_id", None)
    if rid:
        hdrs["X-Request-ID"] = rid
    try:
        r = await client.request(
            request.method,
            url,
            content=body if body else None,
            headers=hdrs,
        )
    except httpx.ConnectError as e:
        LOGGER.warning("VPE upstream unreachable %s: %s", base, e)
        return error_json_response(
            request,
            502,
            "upstream_unreachable",
            f"视觉位姿 Web 不可达: {base}。请先启动 visual_pose_estimation_web（或 launch 设 launch_vpe_web:=true）。",
            upstream=base,
        )
    except httpx.RequestError as e:
        LOGGER.warning("VPE proxy error: %s", e)
        return error_json_response(
            request,
            502,
            "upstream_error",
            str(e),
            upstream=base,
        )

    return Response(
        content=r.content,
        status_code=r.status_code,
        headers=_filter_response_headers(r.headers),
        media_type=r.headers.get("content-type"),
    )


@router.api_route("/health", methods=["GET", "HEAD", "POST", "OPTIONS"])
async def proxy_health(request: Request):
    return await _forward(request, "/health")


@router.api_route("/status", methods=["GET", "HEAD", "POST", "OPTIONS"])
async def proxy_status(request: Request):
    return await _forward(request, "/status")


@router.api_route("/exit", methods=["GET", "POST", "OPTIONS"])
async def proxy_exit(request: Request):
    return await _forward(request, "/exit")


@router.api_route("/api/{path:path}", methods=["GET", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"])
async def proxy_api(request: Request, path: str):
    # /api/config 必须由网关本地 system.router 提供，供 RWT 使用
    if path == "config" or path.startswith("config/"):
        return error_json_response(
            request,
            404,
            "config_local_only",
            "RWT 配置请使用 GET /api/config（网关本地）",
        )
    return await _forward(request, f"/api/{path}")
