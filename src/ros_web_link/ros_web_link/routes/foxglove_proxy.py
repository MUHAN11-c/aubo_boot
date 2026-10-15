"""
反向代理核心 — 浏览器 ↔ 网关 ↔ 上游 ROS 服务.

两条代理通道:
  1. WebSocket 代理 (/ws/foxglove) ← 唯一传输面
     双向异步转发，任一方向断开即关闭另一方向

  2. HTTP 流代理 (/api/ivg/proxy/web-video/*)
     浏览器 ← 网关 ← web_video_server (MJPEG/快照)
     流式转发，过滤逐跳头
"""
from __future__ import annotations

import asyncio
import logging
import random
import time

import httpx
import websockets
from websockets.exceptions import ConnectionClosed, InvalidHandshake, InvalidURI
from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect
from starlette.responses import JSONResponse, StreamingResponse

from .. import config as cfg

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

def _foxglove_bridge_upstream_url() -> str:
    """foxglove_bridge 上游 WebSocket 地址（路径可配，vendored 桥服务于 /）."""
    path = cfg.foxglove_bridge_ws_path() or "/"
    if not path.startswith("/"):
        path = "/" + path
    return f"ws://{cfg.foxglove_bridge_host()}:{cfg.foxglove_bridge_port()}{path}"


def _web_video_upstream_base() -> str:
    """web_video_server 上游 HTTP 基地址."""
    return f"http://{cfg.web_video_host()}:{cfg.web_video_port()}"


def _video_proxy_timeout() -> httpx.Timeout:
    """视频代理的 httpx 超时配置."""
    return httpx.Timeout(
        connect=cfg.proxy_video_connect_timeout(),
        read=cfg.proxy_video_read_timeout(),
        write=cfg.proxy_video_connect_timeout(),
        pool=cfg.proxy_video_pool_timeout(),
    )


# ── 资源清理工具 ─────────────────────────────────────────────────────────────

async def _aclose_httpx(resp: httpx.Response | None,
                        client: httpx.AsyncClient | None) -> None:
    """安静关闭 httpx 响应和客户端，忽略所有异常."""
    for obj in (resp, client):
        if obj is not None:
            try:
                await obj.aclose()
            except Exception:
                pass


async def _close_ws(ws) -> None:
    """安静关闭 WebSocket，忽略所有异常."""
    try:
        await ws.close()
    except Exception:
        pass


def _parse_subprotocols(websocket: WebSocket) -> list[str]:
    """从客户端 WebSocket 握手中提取请求的子协议列表."""
    raw = websocket.headers.get("sec-websocket-protocol", "")
    if not raw:
        return []
    return [p.strip() for p in raw.split(",") if p.strip()]


# ═══════════════════════════════ WebSocket 代理: /ws/foxglove ═══════════════════════════════

_UPSTREAM_CONNECT_ERRORS = (
    ConnectionClosed,
    InvalidURI,
    InvalidHandshake,
    ConnectionRefusedError,
    OSError,
)


async def _connect_upstream_with_retry(target: str, subprotocol: str):
    """连接上游桥；有限次重试 + 抖动退避.

    桥重启/短暂不可达窗口内有限重试，全部失败抛最后一次异常。
    间隔加 ±50% 抖动，避免多客户端同步重试形成重连风暴
    （架构文档已知问题 3 的网关层缓解）。
    """
    attempts = cfg.proxy_upstream_connect_attempts()
    base_ms = cfg.proxy_upstream_connect_retry_ms()
    last_exc: Exception | None = None
    for i in range(attempts):
        try:
            return await websockets.connect(
                target,
                max_size=cfg.foxglove_bridge_max_message_bytes(),
                ping_interval=cfg.foxglove_bridge_ping_interval(),
                ping_timeout=cfg.foxglove_bridge_ping_timeout(),
                close_timeout=cfg.foxglove_bridge_close_timeout(),
                compression="deflate" if cfg.foxglove_bridge_compression() else None,
                subprotocols=[subprotocol],
            )
        except _UPSTREAM_CONNECT_ERRORS as e:
            last_exc = e
            if i + 1 >= attempts:
                break
            delay_s = base_ms * (0.5 + random.random()) / 1000.0  # 抖动非加密用途，random 足够
            _logger.warning("上游 %s 连接失败（第 %d/%d 次），%.0fms 后重试: %s",
                            target, i + 1, attempts, delay_s * 1000, e)
            await asyncio.sleep(delay_s)
    assert last_exc is not None
    raise last_exc


@ws_router.websocket("/ws/foxglove")
async def foxglove_bridge_websocket_proxy(websocket: WebSocket) -> None:
    """浏览器 WebSocket ↔ foxglove_bridge 双向代理（CDR 二进制，点云/高频话题专用）.

    连接 foxglove_bridge 上游（有限重试 + 抖动；全部失败以 1011 关闭客户端，
    而非无 close code 的突然断开——浏览器侧 protocol.js 把 code 透传进重连诊断）。
    使用更大的 max_size 以容纳 PointCloud2 等大消息。
    转发客户端请求的 foxglove 子协议（sdk.v1 或 websocket.v1）到上游。
    websocket.v1 是 vendored ivg_foxglove_bridge（ws-protocol 最终独立库）使用的子协议。
    """
    t0 = time.monotonic()

    client_subprotocols = _parse_subprotocols(websocket)
    _FOXGLOVE_SUBPROTOCOLS = ("foxglove.websocket.v1", "foxglove.sdk.v1")
    fox_sub = [p for p in client_subprotocols if p in _FOXGLOVE_SUBPROTOCOLS]
    # 客户端侧按其声明的 foxglove 子协议回应（浏览器 WS 要求所选子协议 ∈ 客户端请求集）；
    # 未声明者回 websocket.v1。上游恒定连 websocket.v1（vendored ivg_foxglove_bridge 唯一方言），
    # 归一化 sdk.v1 旧客户端与缺省客户端，避免 400 拒绝风暴（2026-10-10 实证）。
    client_accepted_sub = fox_sub[0] if fox_sub else _FOXGLOVE_SUBPROTOCOLS[0]

    await websocket.accept(subprotocol=client_accepted_sub)
    target = _foxglove_bridge_upstream_url()

    upstream = None
    try:
        try:
            upstream = await _connect_upstream_with_retry(
                target, _FOXGLOVE_SUBPROTOCOLS[0])
        except _UPSTREAM_CONNECT_ERRORS as e:
            _logger.warning("上游 %s 不可达（已重试），关闭客户端连接: %s", target, e)
            try:
                await websocket.close(code=1011, reason="foxglove_bridge upstream unavailable")
            except Exception:
                pass
            return

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
            except ConnectionClosed:
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
    except (ConnectionClosed, InvalidURI, InvalidHandshake, ConnectionRefusedError, OSError):
        await _close_ws(websocket)
    except Exception:
        elapsed = time.monotonic() - t0
        if elapsed < 0.5:
            # 短命连接异常多为客户端侧握手后立即离开/上游刚断开的竞态，
            # DEBUG 级留痕即可诊断，不再完全静默
            _logger.debug("foxglove_bridge 代理短命异常 (%.1fs)", elapsed, exc_info=True)
        else:
            _logger.warning("foxglove_bridge 代理异常 (%.1fs)", elapsed, exc_info=True)
        await _close_ws(websocket)
    finally:
        if upstream is not None:
            await _close_ws(upstream)


# ═══════════════════════════════ HTTP 流代理: /api/ivg/proxy/web-video/* ═══════════════════════════════

# 共享 httpx 客户端（连接池复用；MJPEG 流/快照高频请求不再逐请求握手）。
# 生命周期由 app lifespan 管理（create_app 的 lifespan 调 aclose_shared_http_client）。
_shared_video_client: httpx.AsyncClient | None = None


def get_shared_video_client() -> httpx.AsyncClient:
    global _shared_video_client
    if _shared_video_client is None or _shared_video_client.is_closed:
        _shared_video_client = httpx.AsyncClient(
            timeout=_video_proxy_timeout(), trust_env=False)
    return _shared_video_client


async def aclose_shared_video_client() -> None:
    global _shared_video_client
    if _shared_video_client is not None and not _shared_video_client.is_closed:
        try:
            await _shared_video_client.aclose()
        except Exception:
            pass
    _shared_video_client = None


@http_proxy_router.api_route(
    "/proxy/web-video/{path:path}",
    methods=["GET", "HEAD"],
    response_model=None,
)
async def web_video_http_proxy(
    path: str, request: Request
) -> StreamingResponse | JSONResponse:
    """HTTP 流代理 → web_video_server（MJPEG 视频流 / JPEG 快照）.

    路径安全检查 → 拼接上游 URL → 流式转发响应体，
    上游不可达时返回 502/504。
    """
    if ".." in path or path.startswith("/"):
        raise HTTPException(status_code=400, detail="非法路径")

    url = f"{_web_video_upstream_base().rstrip('/')}/{path}"
    if request.url.query:
        url = f"{url}?{request.url.query}"

    client = get_shared_video_client()
    resp: httpx.Response | None = None

    try:
        req = client.build_request(request.method, url)
        resp = await client.send(req, stream=True)
    except httpx.TimeoutException:
        await _aclose_httpx(resp, None)
        return JSONResponse(status_code=504, content={"detail": "视频上游超时"})
    except httpx.RequestError:
        await _aclose_httpx(resp, None)
        return JSONResponse(status_code=502, content={"detail": "视频上游不可达"})
    except Exception:
        await _aclose_httpx(resp, None)
        raise

    out_h = {k: v for k, v in resp.headers.items()
             if k.lower() not in _HOP_BY_HOP}
    resp_ref = resp

    async def body_iter():
        """流式读取上游响应体，逐块 yield 给 StreamingResponse."""
        try:
            async for chunk in resp_ref.aiter_bytes(chunk_size=cfg.proxy_video_chunk_bytes()):
                if chunk:
                    yield chunk
        except asyncio.CancelledError:
            raise
        except (httpx.TimeoutException, httpx.RequestError, httpx.ReadError) as e:
            # 流中断不再静默截断：记 warning，客户端看到的是已转发的部分帧
            _logger.warning("视频上游流中断 %s: %s", url, e)
        finally:
            await _aclose_httpx(resp_ref, None)

    return StreamingResponse(
        body_iter(),
        status_code=resp.status_code,
        headers=out_h,
        media_type=resp.headers.get("content-type"),
    )
