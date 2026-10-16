"""
反向代理核心 — 浏览器 ↔ 网关 ↔ 上游 ROS 服务.

两条代理通道:
  1. WebSocket 代理 (/ws/foxglove) ← 唯一传输面
     双向异步转发；出站经有界数据通道（字节预算 + 丢最旧二进制帧），
     慢客户端不再无限积压延迟（详见 _BoundedForwardLane）
  2. HTTP 流代理 (/api/ivg/proxy/web-video/*)
     浏览器 ← 网关 ← web_video_server (MJPEG/快照)
     流式转发，过滤逐跳头

连接级与累计转发统计暴露在 GET /api/ivg/ws/stats（运维/诊断台用）。
"""
from __future__ import annotations

import asyncio
import collections
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
    max_queue 取小值：慢客户端时背压更早传给桥（桥在 send buffer 超限处
    丢帧），而不是在网关 websockets 接收队列里无界积压大帧。
    """
    attempts = cfg.proxy_upstream_connect_attempts()
    base_ms = cfg.proxy_upstream_connect_retry_ms()
    last_exc: Exception | None = None
    for i in range(attempts):
        try:
            return await websockets.connect(
                target,
                max_size=cfg.foxglove_bridge_max_message_bytes(),
                max_queue=cfg.proxy_upstream_max_queue(),
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


# ═══════════════════════════ 有界出站转发通道 ═══════════════════════════

# 出站丢帧策略对齐三处官方实现（2026-10-16 深读）：
# - rosbridge_suite websocket_handler：write_queue 满丢出站消息、入站 deque(maxlen) 丢最旧；
# - Lichtblick FoxgloveWebSocketPlayer：parsedMessages 字节预算超限丢最旧至 80%（滞回防抖）；
# - vendored 桥 websocket_server.hpp：客户端 send buffer 超限丢新帧（Send buffer limit reached）。
# 差异点：本通道控制帧（JSON op）永不丢、全局保序；只丢二进制数据帧，且按字节预算。
_DROP_HYSTERESIS = 0.8  # 超预算后丢最旧至 80%（Lichtblick 语义：避免逐帧触发丢帧）

# foxglove v1 messageData 帧头（ws-protocol spec）：[op 0x01][subId u32 LE]
# [logTime u64 LE][payload]。logTime = 桥收到该消息的时刻（ros2_foxglove_bridge.cpp:839）。
_MSG_HEADER_LEN = 13


class _BoundedForwardLane:
    """出站转发通道：字节预算内 FIFO，超预算丢最旧二进制帧（数据可丢，控制不可丢）.

    Foxglove v1 的 messageData 本就是 BEST_EFFORT 流（上游桥在 send buffer 超
    限时同样丢帧），丢旧帧只影响丢帧率、不影响正确性；而控制帧（advertise/
    unadvertise/服务响应等 JSON op）丢失会破坏会话语义，永不丢。
    保序：单通道 FIFO，JSON 与数据帧相对顺序不变。
    预算为 0 时退化为无界队列（旧版行为，供对拍）。

    出口年龄门（egress_max_age_ms>0 时启用）：超过 max_age 的数据帧推送前丢弃——
    慢客户端恢复后排空积压时，先读到的是积压期已被内核缓冲的旧帧，此后进来的
    帧全部新鲜；门把「送进内核之前就已陈旧」的帧挡在门外。内核套接字缓冲
    （对端 rcvbuf 自动调优）是 TCP 固有的已提交积压，服务端不可回收——彻底的
    latest-wins 语义由浏览器 player 的 latest 合并模式兜底（Lichtblick 同款）。
    """

    def __init__(self, budget_bytes: int, egress_max_age_s: float = 0.0) -> None:
        self._budget = max(0, int(budget_bytes))
        self._max_age_s = max(0.0, float(egress_max_age_s))
        self._dq: collections.deque = collections.deque()  # [droppable, message, size]
        self._bytes = 0
        self._event = asyncio.Event()
        # 统计（连接生命周期内）
        self.forwarded_json = 0
        self.forwarded_data = 0
        self.forwarded_data_bytes = 0
        self.dropped_data = 0
        self.dropped_data_bytes = 0
        self.dropped_stale = 0  # 出口年龄门丢弃（已陈旧不推送）
        self.peak_bytes = 0

    def put(self, message) -> None:
        if isinstance(message, str):
            self._dq.append((False, message, 0))
            self._event.set()
            return
        size = len(message)
        if self._budget > 0 and self._bytes + size > self._budget:
            target = int(self._budget * _DROP_HYSTERESIS)
            # 丢最旧数据帧至滞回线（控制帧跳过，永不丢）
            while self._bytes + size > target and self._drop_oldest_data():
                pass
            if self._bytes + size > self._budget:
                # 丢光数据帧仍超预算（极端：控制帧洪峰）：丢最新数据帧（rosbridge 语义）
                self.dropped_data += 1
                self.dropped_data_bytes += size
                return
        self._bytes += size
        if self._bytes > self.peak_bytes:
            self.peak_bytes = self._bytes
        self._dq.append((True, message, size))
        self._event.set()

    def _is_stale(self, message: bytes) -> bool:
        if self._max_age_s <= 0 or len(message) < _MSG_HEADER_LEN:
            return False
        if message[0] != 0x01:
            return False
        log_ns = int.from_bytes(message[5:13], 'little')
        return (time.time() - log_ns / 1e9) > self._max_age_s

    def _drop_oldest_data(self) -> bool:
        """从队首找第一个可丢数据帧删除；全控制帧返回 False。"""
        for item in self._dq:
            if item[0]:  # droppable
                self._dq.remove(item)
                self._bytes -= item[2]
                self.dropped_data += 1
                self.dropped_data_bytes += item[2]
                return True
        return False

    def __len__(self) -> int:
        return len(self._dq)

    @property
    def queued_bytes(self) -> int:
        return self._bytes

    async def run_to(self, send_text, send_bytes) -> None:
        """持续把通道内帧发给客户端；send 抛异常即返回（连接已断）。"""
        while True:
            while self._dq:
                droppable, message, size = self._dq.popleft()
                if droppable:
                    self._bytes -= size
                if droppable and self._is_stale(message):
                    # 出口年龄门：推送前已陈旧的帧直接丢（慢恢复期排积压）
                    self.dropped_stale += 1
                    self.dropped_data += 1
                    self.dropped_data_bytes += size
                    continue
                if isinstance(message, str):
                    self.forwarded_json += 1
                    await send_text(message)
                else:
                    self.forwarded_data += 1
                    self.forwarded_data_bytes += size
                    await send_bytes(message)
            self._event.clear()
            await self._event.wait()


# ── 连接统计（诊断面：/api/ivg/ws/stats）────────────────────────────────────

class _WsConnStats:
    """单条 /ws/foxglove 连接的转发统计快照."""

    def __init__(self, remote: str) -> None:
        self.started_at = time.time()
        self.remote = remote
        self.lane: _BoundedForwardLane | None = None
        self.upstream_frames = 0  # 上游来的全部帧（含 JSON）

    def snapshot(self) -> dict:
        d = {
            "startedAt": self.started_at,
            "remote": self.remote,
            "upstreamFrames": self.upstream_frames,
        }
        if self.lane is not None:
            d.update({
                "forwardedJson": self.lane.forwarded_json,
                "forwardedData": self.lane.forwarded_data,
                "forwardedDataBytes": self.lane.forwarded_data_bytes,
                "droppedData": self.lane.dropped_data,
                "droppedDataBytes": self.lane.dropped_data_bytes,
                "droppedStale": self.lane.dropped_stale,
                "queued": len(self.lane),
                "queuedBytes": self.lane.queued_bytes,
                "peakQueuedBytes": self.lane.peak_bytes,
            })
        return d


_ws_active: list[_WsConnStats] = []
_ws_totals = {
    "connections": 0,
    "upstreamFrames": 0,
    "forwardedJson": 0,
    "forwardedData": 0,
    "forwardedDataBytes": 0,
    "droppedData": 0,
    "droppedDataBytes": 0,
    "droppedStale": 0,
}


@ws_router.get("/api/ivg/ws/stats")
async def ws_proxy_stats() -> dict:
    """/ws/foxglove 转发统计：活跃连接逐条 + 进程累计（诊断台/压测证据面）。"""
    return {
        "dataLaneBytes": cfg.proxy_data_lane_bytes(),
        "active": [c.snapshot() for c in list(_ws_active)],
        "totals": dict(_ws_totals),
    }


@ws_router.websocket("/ws/foxglove")
async def foxglove_bridge_websocket_proxy(websocket: WebSocket) -> None:
    """浏览器 WebSocket ↔ foxglove_bridge 双向代理（CDR 二进制，点云/高频话题专用）.

    连接 foxglove_bridge 上游（有限重试 + 抖动；全部失败以 1011 关闭客户端，
    而非无 close code 的突然断开——浏览器侧 protocol.js 把 code 透传进重连诊断）。
    使用更大的 max_size 以容纳 PointCloud2 等大消息。
    转发客户端请求的 foxglove 子协议（sdk.v1 或 websocket.v1）到上游。
    websocket.v1 是 vendored ivg_foxglove_bridge（ws-protocol 最终独立库）使用的子协议。

    上游→浏览器方向经 _BoundedForwardLane：慢客户端丢最旧数据帧而不是积压延迟。
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

    stats = _WsConnStats(str(websocket.client) if websocket.client else "?")
    _ws_active.append(stats)
    _ws_totals["connections"] += 1

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

        lane = _BoundedForwardLane(cfg.proxy_data_lane_bytes(),
                                   cfg.proxy_egress_max_age_s())
        stats.lane = lane

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

        async def upstream_reader():
            """上游读入通道：读得快，积压与丢帧都发生在有界通道里。"""
            try:
                async for message in upstream:
                    stats.upstream_frames += 1
                    lane.put(message)
            except ConnectionClosed:
                pass
            finally:
                # 上游断开：关客户端连接，主协程据 FIRST_COMPLETED 收尾全部任务
                await _close_ws(websocket)

        async def lane_writer():
            """通道排空转发到浏览器；run_to 正常不返回（等待新帧），断连异常退出。"""
            try:
                while True:
                    await lane.run_to(websocket.send_text, websocket.send_bytes)
            except (WebSocketDisconnect, RuntimeError, OSError):
                return
            finally:
                await _close_ws(upstream)

        t1 = asyncio.create_task(client_to_upstream())
        t2 = asyncio.create_task(upstream_reader())
        t3 = asyncio.create_task(lane_writer())
        try:
            _, pending = await asyncio.wait(
                {t1, t2, t3}, return_when=asyncio.FIRST_COMPLETED)
            for t in pending:
                t.cancel()
            await asyncio.gather(t1, t2, t3, return_exceptions=True)
        except asyncio.CancelledError:
            for t in (t1, t2, t3):
                if not t.done():
                    t.cancel()
            await asyncio.gather(t1, t2, t3, return_exceptions=True)
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
        # 累计归账（活跃表摘除 + 总量表累加）
        if stats in _ws_active:
            _ws_active.remove(stats)
        snap = stats.snapshot()
        _ws_totals["upstreamFrames"] += snap.get("upstreamFrames", 0)
        _ws_totals["forwardedJson"] += snap.get("forwardedJson", 0)
        _ws_totals["forwardedData"] += snap.get("forwardedData", 0)
        _ws_totals["forwardedDataBytes"] += snap.get("forwardedDataBytes", 0)
        _ws_totals["droppedData"] += snap.get("droppedData", 0)
        _ws_totals["droppedDataBytes"] += snap.get("droppedDataBytes", 0)
        _ws_totals["droppedStale"] += snap.get("droppedStale", 0)


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
