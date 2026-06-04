"""反向代理核心 — 浏览器 ↔ 网关 ↔ 上游 ROS 服务。

两条代理通道:
  1. WebSocket 代理 (/ws/rosbridge)
     浏览器 ←→ 网关 ←→ rosbridge (Tornado)
     双向异步转发，任一方向断开即关闭另一方向

  2. HTTP 流代理 (/api/ivg/proxy/web-video/*)
     浏览器 ← 网关 ← web_video_server (MJPEG/快照)
     流式转发，过滤逐跳头
"""
from __future__ import annotations

import asyncio
import time

import httpx
import websockets
from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect
from starlette.responses import JSONResponse, StreamingResponse

import logging

from aubo_ros2_web_dashboard import config as cfg

_logger = logging.getLogger("gateway.proxy")

# 路由定义
ws_router = APIRouter(tags=["proxy"])
http_proxy_router = APIRouter(prefix="/api/ivg", tags=["proxy"])

# HTTP 逐跳头，代理转发时必须剥离
_HOP_BY_HOP = frozenset({
    "connection", "keep-alive", "proxy-authenticate", "proxy-authorization",
    "te", "trailers", "transfer-encoding", "upgrade", "content-encoding",
})


# ── 上游地址 ─────────────────────────────────────────────────────────────────

def _rosbridge_upstream_url() -> str:
    """rosbridge 上游 WebSocket 地址。"""
    return f"ws://{cfg.rosbridge_host()}:{cfg.rosbridge_port()}/"


def _foxglove_bridge_upstream_url() -> str:
    """foxglove_bridge 上游 WebSocket 地址。"""
    return f"ws://{cfg.foxglove_bridge_host()}:{cfg.foxglove_bridge_port()}/"


def _web_video_upstream_base() -> str:
    """web_video_server 上游 HTTP 基地址。"""
    return f"http://{cfg.web_video_host()}:{cfg.web_video_port()}"


def _video_proxy_timeout() -> httpx.Timeout:
    """视频代理的 httpx 超时配置。"""
    return httpx.Timeout(
        connect=cfg.proxy_video_connect_timeout(),
        read=cfg.proxy_video_read_timeout(),
        write=cfg.proxy_video_connect_timeout(),
        pool=cfg.proxy_video_pool_timeout(),
    )


# ── 资源清理工具 ─────────────────────────────────────────────────────────────

async def _aclose_httpx(resp: httpx.Response | None,
                        client: httpx.AsyncClient | None) -> None:
    """安静关闭 httpx 响应和客户端，忽略所有异常。"""
    for obj in (resp, client):
        if obj is not None:
            try:
                await obj.aclose()
            except Exception:
                pass


async def _close_ws(ws) -> None:
    """安静关闭 WebSocket，忽略所有异常。"""
    try:
        await ws.close()
    except Exception:
        pass


def _parse_subprotocols(websocket: WebSocket) -> list[str]:
    """从客户端 WebSocket 握手中提取请求的子协议列表。"""
    raw = websocket.headers.get("sec-websocket-protocol", "")
    if not raw:
        return []
    return [p.strip() for p in raw.split(",") if p.strip()]


# ═══════════════════════════════════════════════════════════════════════════════
# WebSocket 代理: /ws/rosbridge
# ═══════════════════════════════════════════════════════════════════════════════

@ws_router.websocket("/ws/rosbridge")
async def rosbridge_websocket_proxy(websocket: WebSocket) -> None:
    """浏览器 WebSocket ↔ rosbridge 双向代理。

    建立两条 asyncio Task 分别处理上下游方向，
    任一方向断开即取消另一方向并清理资源。

    roslib 2.x 通过代理时首次连接可能立即断开 (内部探测/握手),
    第二次连接稳定。日志已抑制快速断开场景以避免误导喵~
    """
    t0 = time.monotonic()
    await websocket.accept()
    target = _rosbridge_upstream_url()

    try:
        async with websockets.connect(
            target,
            max_size=cfg.rosbridge_max_message_bytes(),
            ping_interval=cfg.rosbridge_ping_interval(),
            ping_timeout=cfg.rosbridge_ping_timeout(),
            close_timeout=cfg.rosbridge_close_timeout(),
        ) as upstream:

            async def client_to_upstream():
                """浏览器 → rosbridge: 接收浏览器消息，转发到上游。"""
                try:
                    while True:
                        try:
                            msg = await websocket.receive()
                        except WebSocketDisconnect:
                            return
                        if msg["type"] == "websocket.disconnect":
                            return
                        if msg["type"] != "websocket.receive":
                            continue
                        if msg.get("text") is not None:
                            await upstream.send(msg["text"])
                        elif msg.get("bytes") is not None:
                            await upstream.send(msg["bytes"])
                finally:
                    await _close_ws(upstream)

            async def upstream_to_client():
                """rosbridge → 浏览器: 接收上游消息，转发到浏览器。"""
                try:
                    async for message in upstream:
                        try:
                            if isinstance(message, str):
                                await websocket.send_text(message)
                            else:
                                await websocket.send_bytes(message)
                        except (WebSocketDisconnect, RuntimeError, OSError):
                            return
                except websockets.exceptions.ConnectionClosed:
                    return
                finally:
                    await _close_ws(websocket)

            # 并发运行两个方向，任一完成则取消另一个
            t1 = asyncio.create_task(client_to_upstream())
            t2 = asyncio.create_task(upstream_to_client())
            try:
                _, pending = await asyncio.wait(
                    {t1, t2}, return_when=asyncio.FIRST_COMPLETED)
                for t in pending:
                    t.cancel()
                await asyncio.gather(t1, t2, return_exceptions=True)
            except asyncio.CancelledError:
                for t in (t1, t2):
                    if not t.done():
                        t.cancel()
                await asyncio.gather(t1, t2, return_exceptions=True)
                raise
    except (websockets.exceptions.ConnectionClosed, websockets.exceptions.InvalidURI,
            websockets.exceptions.InvalidHandshake, ConnectionRefusedError, OSError):
        # 上游连接失败 — 关闭客户端连接
        await _close_ws(websocket)
    except Exception:
        elapsed = time.monotonic() - t0
        if elapsed < 0.5:
            # roslib 探测连接 (首次连接 < 500ms 断开是正常行为, 抑制日志) 喵~
            pass
        else:
            _logger.warning("rosbridge 代理异常 (%.1fs)", elapsed, exc_info=True)
        await _close_ws(websocket)


# ═══════════════════════════════════════════════════════════════════════════════
# WebSocket 代理: /ws/foxglove
# ═══════════════════════════════════════════════════════════════════════════════

@ws_router.websocket("/ws/foxglove")
async def foxglove_bridge_websocket_proxy(websocket: WebSocket) -> None:
    """浏览器 WebSocket ↔ foxglove_bridge 双向代理（CDR 二进制，点云/高频话题专用）。

    与 rosbridge 代理逻辑相同，但连接 foxglove_bridge 上游，
    使用更大的 max_size 以容纳 PointCloud2 等大消息。
    转发客户端请求的 foxglove.sdk.v1 子协议到上游喵~
    """
    t0 = time.monotonic()

    # 提取客户端请求的子协议, 转发到 foxglove_bridge
    client_subprotocols = _parse_subprotocols(websocket)
    fox_sub = [p for p in client_subprotocols if p == "foxglove.sdk.v1"]
    accepted_sub = fox_sub[0] if fox_sub else None

    await websocket.accept(subprotocol=accepted_sub)
    target = _foxglove_bridge_upstream_url()

    try:
        async with websockets.connect(
            target,
            max_size=cfg.foxglove_bridge_max_message_bytes(),
            ping_interval=cfg.foxglove_bridge_ping_interval(),
            ping_timeout=cfg.foxglove_bridge_ping_timeout(),
            close_timeout=cfg.foxglove_bridge_close_timeout(),
            subprotocols=[accepted_sub] if accepted_sub else None,
        ) as upstream:

            async def client_to_upstream():
                try:
                    while True:
                        try:
                            msg = await websocket.receive()
                        except WebSocketDisconnect:
                            return
                        if msg["type"] == "websocket.disconnect":
                            return
                        if msg["type"] != "websocket.receive":
                            continue
                        if msg.get("text") is not None:
                            await upstream.send(msg["text"])
                        elif msg.get("bytes") is not None:
                            await upstream.send(msg["bytes"])
                finally:
                    await _close_ws(upstream)

            async def upstream_to_client():
                try:
                    async for message in upstream:
                        try:
                            if isinstance(message, str):
                                await websocket.send_text(message)
                            else:
                                await websocket.send_bytes(message)
                        except (WebSocketDisconnect, RuntimeError, OSError):
                            return
                except websockets.exceptions.ConnectionClosed:
                    return
                finally:
                    await _close_ws(websocket)

            t1 = asyncio.create_task(client_to_upstream())
            t2 = asyncio.create_task(upstream_to_client())
            try:
                _, pending = await asyncio.wait(
                    {t1, t2}, return_when=asyncio.FIRST_COMPLETED)
                for t in pending:
                    t.cancel()
                await asyncio.gather(t1, t2, return_exceptions=True)
            except asyncio.CancelledError:
                for t in (t1, t2):
                    if not t.done():
                        t.cancel()
                await asyncio.gather(t1, t2, return_exceptions=True)
                raise
    except (websockets.exceptions.ConnectionClosed, websockets.exceptions.InvalidURI,
            websockets.exceptions.InvalidHandshake, ConnectionRefusedError, OSError):
        await _close_ws(websocket)
    except Exception:
        elapsed = time.monotonic() - t0
        if elapsed < 0.5:
            pass
        else:
            _logger.warning("foxglove_bridge 代理异常 (%.1fs)", elapsed, exc_info=True)
        await _close_ws(websocket)


# ═══════════════════════════════════════════════════════════════════════════════
# HTTP 流代理: /api/ivg/proxy/web-video/*
# ═══════════════════════════════════════════════════════════════════════════════

@http_proxy_router.api_route(
    "/proxy/web-video/{path:path}",
    methods=["GET", "HEAD"],
    response_model=None,
)
async def web_video_http_proxy(
    path: str, request: Request
) -> StreamingResponse | JSONResponse:
    """HTTP 流代理 → web_video_server（MJPEG 视频流 / JPEG 快照）。

    路径安全检查 → 拼接上游 URL → 流式转发响应体，
    上游不可达时返回 502/504。
    """
    # 安全检查
    if ".." in path or path.startswith("/"):
        raise HTTPException(status_code=400, detail="非法路径")

    # 拼接上游 URL
    url = f"{_web_video_upstream_base().rstrip('/')}/{path}"
    if request.url.query:
        url = f"{url}?{request.url.query}"

    timeout = _video_proxy_timeout()
    chunk_sz = cfg.proxy_video_chunk_bytes()
    client: httpx.AsyncClient | None = httpx.AsyncClient(
        timeout=timeout, trust_env=False)
    resp: httpx.Response | None = None

    # 发起上游请求
    try:
        req = client.build_request(request.method, url)
        resp = await client.send(req, stream=True)
    except httpx.TimeoutException:
        await _aclose_httpx(None, client)
        return JSONResponse(status_code=504, content={"detail": "视频上游超时"})
    except httpx.RequestError:
        await _aclose_httpx(None, client)
        return JSONResponse(status_code=502, content={"detail": "视频上游不可达"})
    except Exception:
        await _aclose_httpx(None, client)
        raise

    # 过滤逐跳头
    out_h = {k: v for k, v in resp.headers.items()
             if k.lower() not in _HOP_BY_HOP}
    resp_ref, client_ref = resp, client

    async def body_iter():
        """流式读取上游响应体，逐块 yield 给 StreamingResponse。"""
        try:
            async for chunk in resp_ref.aiter_bytes(chunk_size=chunk_sz):
                if chunk:
                    yield chunk
        except asyncio.CancelledError:
            raise
        except (httpx.TimeoutException, httpx.RequestError, httpx.ReadError):
            return
        finally:
            await _aclose_httpx(resp_ref, client_ref)

    return StreamingResponse(
        body_iter(),
        status_code=resp.status_code,
        headers=out_h,
        media_type=resp.headers.get("content-type"),
    )
