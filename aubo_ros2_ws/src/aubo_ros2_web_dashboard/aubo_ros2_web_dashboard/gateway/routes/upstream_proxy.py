"""
经 FastAPI 转发 rosbridge WebSocket 与 web_video_server HTTP，浏览器只连页面端口。

上游地址由 IVG_ROSBRIDGE_HOST / IVG_ROSBRIDGE_PORT、IVG_WEB_VIDEO_HOST / IVG_WEB_VIDEO_PORT 决定（默认 127.0.0.1）。

视频 HTTP 代理（MJPEG/snapshot）环境变量：

- IVG_PROXY_VIDEO_CONNECT_TIMEOUT：连接上游秒数，默认 15。
- IVG_PROXY_VIDEO_READ_TIMEOUT：流式读单段超时；未设置或 ≤0 表示不限制（适合长连接 MJPEG）。
- IVG_PROXY_VIDEO_POOL_TIMEOUT：连接池等待秒数，默认 10。
- IVG_PROXY_VIDEO_CHUNK_BYTES：``aiter_bytes`` 块大小，默认 65536。

``httpx.AsyncClient`` 对上游使用 ``trust_env=False``，避免本机 web_video 被系统 ``HTTP_PROXY`` 拐到企业代理。
"""
from __future__ import annotations

import asyncio #异步操作
import os

import httpx #HTTP客户端
import websockets #WebSocket客户端
from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect
from starlette.responses import JSONResponse, StreamingResponse #响应

from aubo_ros2_web_dashboard.gateway import settings as cfg

# 与 launch 中 rosbridge max_message_size 同量级，避免大点云/CBOR 帧被上游库拒收（默认 64MiB）
_WS_MAX_MESSAGE_BYTES = int(os.environ.get("IVG_PROXY_WS_MAX_BYTES", str(64 * 1024 * 1024)))

# WebSocket：完整路径 /ws/rosbridge（无 prefix）
ws_router = APIRouter(tags=["proxy"])

# HTTP：前缀 /api/ivg，故视频代理完整路径为 /api/ivg/proxy/web-video/{path}
http_proxy_router = APIRouter(prefix="/api/ivg", tags=["proxy"])

# 回写给浏览器时去掉逐跳头，避免 Content-Length/Transfer-Encoding 等与 StreamingResponse 冲突
_HOP_BY_HOP = frozenset(
	{
		"connection",
		"keep-alive",
		"proxy-authenticate",
		"proxy-authorization",
		"te",
		"trailers",
		"transfer-encoding",
		"upgrade",
		"content-encoding",
	}
)


# ---------- 上游 ws/http 基址（调用时读 os.environ，与进程启动时注入的 IVG_* 一致）----------


def _rosbridge_upstream_url() -> str:
	host = os.environ.get(cfg.ENV_ROSBRIDGE_HOST, cfg.DEFAULT_ROSBRIDGE_HOST)
	port = int(os.environ.get(cfg.ENV_ROSBRIDGE, str(cfg.DEFAULT_ROSBRIDGE_PORT)))
	return f"ws://{host}:{port}/"


def _web_video_upstream_base() -> str:
	host = os.environ.get(cfg.ENV_WEB_VIDEO_HOST, cfg.DEFAULT_WEB_VIDEO_HOST)
	port = int(os.environ.get(cfg.ENV_WEB_VIDEO, str(cfg.DEFAULT_WEB_VIDEO_PORT)))
	return f"http://{host}:{port}"


# ---------- web_video HTTP 客户端参数 ----------


def _video_proxy_httpx_timeout() -> httpx.Timeout:
	"""流式视频：默认不限制 read（MJPEG 帧间可能很久）；连接与连接池仍限时。"""
	connect = float(os.environ.get("IVG_PROXY_VIDEO_CONNECT_TIMEOUT", "15"))
	pool = float(os.environ.get("IVG_PROXY_VIDEO_POOL_TIMEOUT", "10"))
	raw = os.environ.get("IVG_PROXY_VIDEO_READ_TIMEOUT")
	read_val: float | None
	if raw is None or str(raw).strip() == "":
		read_val = None
	else:
		read_val = float(raw)
		if read_val <= 0:
			read_val = None
	return httpx.Timeout(connect=connect, read=read_val, write=connect, pool=pool)


def _video_proxy_chunk_bytes() -> int:
	return max(4096, int(os.environ.get("IVG_PROXY_VIDEO_CHUNK_BYTES", str(64 * 1024))))


async def _aclose_httpx_stream(resp: httpx.Response | None, client: httpx.AsyncClient | None) -> None:
	if resp is not None:
		try:
			await resp.aclose()
		except Exception:
			pass
	if client is not None:
		try:
			await client.aclose()
		except Exception:
			pass


async def _close_upstream_quietly(upstream) -> None:
	try:
		await upstream.close()
	except Exception:
		pass


async def _close_browser_ws_quietly(websocket: WebSocket) -> None:
	try:
		await websocket.close()
	except Exception:
		pass


# ---------- rosbridge：浏览器 WS ↔ 本机 Tornado WS，双向任务任一结束则收敛 ----------


@ws_router.websocket("/ws/rosbridge")
async def rosbridge_websocket_proxy(websocket: WebSocket) -> None:
	await websocket.accept()
	target = _rosbridge_upstream_url()
	try:
		async with websockets.connect(
			target,
			max_size=_WS_MAX_MESSAGE_BYTES,
			ping_interval=20,
			ping_timeout=60,
			close_timeout=10,
		) as upstream:

			async def client_to_upstream() -> None:
				"""浏览器 → rosbridge；任一端结束须在 finally 里关上游，以唤醒 upstream→client 的 async for。"""
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
					await _close_upstream_quietly(upstream)

			async def upstream_to_client() -> None:
				"""rosbridge → 浏览器；上游断线或浏览器不可写时结束，并尝试关浏览器 WS 释放 receive 侧。"""
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
					await _close_browser_ws_quietly(websocket)

			t1 = asyncio.create_task(client_to_upstream())
			t2 = asyncio.create_task(upstream_to_client())
			try:
				_, pending = await asyncio.wait({t1, t2}, return_when=asyncio.FIRST_COMPLETED)
				for t in pending:
					t.cancel()
				await asyncio.gather(t1, t2, return_exceptions=True)
			except asyncio.CancelledError:
				for t in (t1, t2):
					if not t.done():
						t.cancel()
				await asyncio.gather(t1, t2, return_exceptions=True)
				raise
	except OSError:
		try:
			await websocket.close(code=1011)
		except Exception:
			pass
	except Exception:
		try:
			await websocket.close(code=1011)
		except Exception:
			pass


# ---------- web_video：HTTP 流式转发（MJPEG 等），trust_env=False 防系统代理劫持本机上游 ----------


@http_proxy_router.api_route(
	"/proxy/web-video/{path:path}",
	methods=["GET", "HEAD"],
	response_model=None,
)
async def web_video_http_proxy(path: str, request: Request) -> StreamingResponse | JSONResponse:
	if ".." in path or path.startswith("/"):
		raise HTTPException(status_code=400, detail="invalid path")

	base = _web_video_upstream_base().rstrip("/")
	url = f"{base}/{path}"
	q = request.url.query
	if q:
		url = f"{url}?{q}"

	timeout = _video_proxy_httpx_timeout()
	chunk_sz = _video_proxy_chunk_bytes()
	client: httpx.AsyncClient | None = httpx.AsyncClient(timeout=timeout, trust_env=False)
	resp: httpx.Response | None = None
	try:
		req = client.build_request(request.method, url)
		resp = await client.send(req, stream=True)
	except httpx.TimeoutException:
		await _aclose_httpx_stream(None, client)
		return JSONResponse(status_code=504, content={"detail": "video upstream timeout"})
	except httpx.RequestError:
		await _aclose_httpx_stream(None, client)
		return JSONResponse(status_code=502, content={"detail": "video upstream unreachable"})
	except Exception:
		await _aclose_httpx_stream(None, client)
		raise

	assert resp is not None and client is not None
	out_h = {k: v for k, v in resp.headers.items() if k.lower() not in _HOP_BY_HOP}
	resp_ref = resp
	client_ref = client

	async def body_iter():
		"""分块流式转发；任意退出路径均 ``aclose`` resp + client（头已发出后不再抛 HTTPException）。"""
		try:
			async for chunk in resp_ref.aiter_bytes(chunk_size=chunk_sz):
				if chunk:
					yield chunk
		except asyncio.CancelledError:
			raise
		except (httpx.TimeoutException, httpx.RequestError):
			# 流中途失败：状态码与 Content-Type 已提交，只能掐断 body
			return
		finally:
			await _aclose_httpx_stream(resp_ref, client_ref)

	return StreamingResponse(
		body_iter(),
		status_code=resp.status_code,
		headers=out_h,
		media_type=resp.headers.get("content-type"),
	)
