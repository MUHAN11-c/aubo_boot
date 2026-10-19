"""健康检查 — GET /health，返回网关与上游组件状态.

组件探测为轻量 TCP 连接（不发 WS/HTTP 握手），结果缓存 2s——
桥挂掉时 /health 仍快速返回 degraded，而不是自身拖到超时。
web_video 是可选辅助组件，down 不影响整体 status。
"""
from __future__ import annotations

import asyncio
import time

from fastapi import APIRouter, Request

from .. import __version__, config as cfg

router = APIRouter(tags=["ops"])

_CACHE_TTL = 2.0
_cache: dict[str, tuple[float, bool]] = {}


async def _tcp_reachable(host: str, port: int, timeout: float = 1.0) -> bool:
    """轻量 TCP 探测（端口可连即认为组件进程在）。"""
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(host, port), timeout=timeout)
        del reader
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass
        return True
    except Exception:
        return False


async def _probe_cached(key: str, host: str, port: int) -> bool:
    now = time.monotonic()
    hit = _cache.get(key)
    if hit and now - hit[0] < _CACHE_TTL:
        return hit[1]
    ok = await _tcp_reachable(host, port)
    _cache[key] = (now, ok)
    return ok


@router.get("/health")
async def health(request: Request) -> dict[str, str]:
    bridge_ok = await _probe_cached(
        "foxglove_bridge", cfg.foxglove_bridge_host(), cfg.foxglove_bridge_port())
    video_ok = await _probe_cached(
        "web_video", cfg.web_video_host(), cfg.web_video_port())
    return {
        "status": "ok" if bridge_ok else "degraded",
        "version": __version__,
        "protocol": "foxglove.websocket.v1",
        "static_root": getattr(request.app.state, "static_root", ""),
        "foxglove_bridge": "ok" if bridge_ok else "down",
        "web_video": "ok" if video_ok else "down",
    }
