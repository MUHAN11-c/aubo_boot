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
from starlette.middleware.gzip import GZipMiddleware #GZip中间件

from aubo_ros2_web_dashboard.gateway import settings as cfg #配置
from aubo_ros2_web_dashboard.gateway.routes import health as health_routes
from aubo_ros2_web_dashboard.gateway.routes import ivg_runtime as ivg_runtime_routes
from aubo_ros2_web_dashboard.gateway.routes import robot_mesh as robot_mesh_routes
from aubo_ros2_web_dashboard.gateway.routes.upstream_proxy import http_proxy_router, ws_router


def create_app(web_root: str) -> FastAPI:
	# 与 launch 传入的 share/.../web/public 一致；须为已解析的目录路径
	root = os.path.abspath(os.path.realpath(web_root))
	if not os.path.isdir(root):
		raise ValueError(f"not a directory: {root}")

	@asynccontextmanager
	async def lifespan(_app: FastAPI):
		# 启动时打印一次有效上游，便于与 launch 参数对照排障
		rb_h = os.environ.get(cfg.ENV_ROSBRIDGE_HOST, cfg.DEFAULT_ROSBRIDGE_HOST)
		wv_h = os.environ.get(cfg.ENV_WEB_VIDEO_HOST, cfg.DEFAULT_WEB_VIDEO_HOST)
		rb_p = os.environ.get(cfg.ENV_ROSBRIDGE, str(cfg.DEFAULT_ROSBRIDGE_PORT))
		wv_p = os.environ.get(cfg.ENV_WEB_VIDEO, str(cfg.DEFAULT_WEB_VIDEO_PORT))
		print(
			f"[ivg] FastAPI static gateway root={root} "
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
	# 供 routes 读取静态根（如 /health、runtime-config 中的 static_root 字段）
	app.state.static_root = root

	app.add_middleware(GZipMiddleware, minimum_size=512)

	@app.middleware("http")
	async def security_headers(request, call_next):
		response = await call_next(request)
		response.headers.setdefault("X-Content-Type-Options", "nosniff")
		return response

	# 以下顺序：更具体的路径先于 StaticFiles；ws /api/ivg 与 /health 互不冲突
	app.include_router(ws_router)
	app.include_router(http_proxy_router)
	app.include_router(health_routes.router)
	app.include_router(ivg_runtime_routes.router)
	app.include_router(ivg_runtime_routes.router_v1)
	app.include_router(robot_mesh_routes.router)

	# html=True：访问目录时可解析 index.html
	app.mount(
		"/",
		StaticFiles(directory=root, html=True),
		name="static",
	)
	return app
