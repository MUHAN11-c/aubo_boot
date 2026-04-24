"""
FastAPI 应用工厂：``create_app(web_root)`` 返回可交给 Uvicorn 的 ASGI 应用。

组装顺序要点：
  1. 先 ``include_router``（/ws、/api、/health 等显式路径）
  2. 再 ``mount("/", StaticFiles(...))`` 作为兜底，避免静态文件抢占 API
"""
from __future__ import annotations #未来兼容性

import os #路径操作
from contextlib import asynccontextmanager #异步上下文管理器

from fastapi import FastAPI #FastAPI框架
from fastapi.staticfiles import StaticFiles #静态文件
from starlette.datastructures import MutableHeaders
from starlette.middleware.gzip import GZipMiddleware #GZip中间件
from starlette.types import ASGIApp, Message, Receive, Scope, Send

from aubo_ros2_web_dashboard.gateway import settings as cfg #配置
from aubo_ros2_web_dashboard.gateway.routes import health as health_routes
from aubo_ros2_web_dashboard.gateway.routes import ivg_runtime as ivg_runtime_routes
from aubo_ros2_web_dashboard.gateway.routes import robot_mesh as robot_mesh_routes
from aubo_ros2_web_dashboard.gateway.routes.upstream_proxy import http_proxy_router, ws_router


class SecurityHeadersMiddleware:
	"""
	纯 ASGI 中间件：为 HTTP 响应补安全头。

	避免使用 ``BaseHTTPMiddleware``：在客户端提前断开、导航取消请求等场景下，
	``BaseHTTPMiddleware`` 可能误报 ``RuntimeError: No response returned``（Starlette #1769 类问题）。
	"""

	def __init__(self, app: ASGIApp) -> None:
		"""包装内层 ASGI 应用 ``app``，在响应起始阶段注入安全相关 HTTP 头。"""
		self.app = app

	async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
		"""ASGI 调用入口：非 HTTP 请求直通；HTTP 在 ``response.start`` 上补 ``X-Content-Type-Options`` 等。"""
		if scope["type"] != "http":
			await self.app(scope, receive, send)
			return

		async def send_with_headers(message: Message) -> None:
			if message["type"] == "http.response.start":
				h = MutableHeaders(raw=message["headers"])
				h.setdefault("X-Content-Type-Options", "nosniff")
			await send(message)

		await self.app(scope, receive, send_with_headers)


def create_app(web_root: str) -> FastAPI:
	"""
	创建并配置 FastAPI 实例：GZip、安全头、代理与健康检查路由，最后挂载 ``web_root`` 静态目录。
	``web_root`` 须为已解析的绝对路径（与 launch 中 ``share/.../web/public`` 一致）。
	"""
	# 与 launch 传入的 share/.../web/public 一致；须为已解析的目录路径
	root = os.path.abspath(os.path.realpath(web_root))
	if not os.path.isdir(root):
		raise ValueError(f"not a directory: {root}")
	embedded_robotwebtools = os.path.join(root, "js", "robotwebtools")
	robotwebtools_root = os.path.abspath(
		os.path.realpath(os.environ.get(cfg.ENV_RWT_ASSETS_DIR, embedded_robotwebtools))
	)
	if not os.path.isdir(robotwebtools_root):
		raise ValueError(f"robotwebtools assets directory not found: {robotwebtools_root}")

	@asynccontextmanager
	async def lifespan(_app: FastAPI):
		"""应用生命周期：启动时打印 rosbridge/web_video 上游与环境一致信息，便于对照 launch 排障。"""
		# 启动时打印一次有效上游，便于与 launch 参数对照排障
		rb_h = os.environ.get(cfg.ENV_ROSBRIDGE_HOST, cfg.DEFAULT_ROSBRIDGE_HOST)
		wv_h = os.environ.get(cfg.ENV_WEB_VIDEO_HOST, cfg.DEFAULT_WEB_VIDEO_HOST)
		rb_p = os.environ.get(cfg.ENV_ROSBRIDGE, str(cfg.DEFAULT_ROSBRIDGE_PORT))
		wv_p = os.environ.get(cfg.ENV_WEB_VIDEO, str(cfg.DEFAULT_WEB_VIDEO_PORT))
		print(
			f"[ivg] FastAPI static gateway root={root} "
			f"robotwebtools_root={robotwebtools_root} "
			f"rosbridge_upstream={rb_h}:{rb_p} "
			f"web_video_upstream={wv_h}:{wv_p}",
			flush=True,
		)
		yield

	app = FastAPI(
		title="IVG Web Dashboard",
		docs_url=None,
		redoc_url=None,
		openapi_url=None,
		lifespan=lifespan,
	)
	# 供 routes 读取静态根（如 /health、/api/v1/runtime 中的 static_root 字段）
	app.state.static_root = root

	app.add_middleware(GZipMiddleware, minimum_size=512)
	app.add_middleware(SecurityHeadersMiddleware)

	# 以下顺序：更具体的路径先于 StaticFiles；ws /api/ivg 与 /health 互不冲突
	app.include_router(ws_router)
	app.include_router(http_proxy_router)
	app.include_router(health_routes.router)
	app.include_router(ivg_runtime_routes.router)
	app.include_router(robot_mesh_routes.router)
	app.mount(
		"/js/robotwebtools",
		StaticFiles(directory=robotwebtools_root, html=False),
		name="robotwebtools",
	)

	# html=True：访问目录时可解析 index.html
	app.mount(
		"/",
		StaticFiles(directory=root, html=True),
		name="static",
	)
	return app
