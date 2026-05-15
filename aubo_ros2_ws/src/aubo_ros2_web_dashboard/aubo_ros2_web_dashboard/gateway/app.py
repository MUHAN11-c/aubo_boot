"""FastAPI 应用工厂 — 组装中间件、路由、静态挂载。

路由注册顺序（优先级从高到低）:
  1. /ws/rosbridge              — WebSocket 代理 → rosbridge
  2. /api/ivg/proxy/web-video/* — HTTP 流代理 → web_video_server
  3. /health                    — 健康检查
  4. /api/v1/runtime            — 前端运行时配置 (BFF)
  5. /api/ivg/robot-mesh/*      — 机器人 3D 模型文件
  6. /js/robotwebtools/*        — RobotWebTools 静态资源
  7. /*                         — 前端静态页面 (SPA)
"""
from __future__ import annotations

import os
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from starlette.datastructures import MutableHeaders
from starlette.middleware.gzip import GZipMiddleware
from starlette.types import ASGIApp, Message, Receive, Scope, Send

from aubo_ros2_web_dashboard import config as cfg
from aubo_ros2_web_dashboard.gateway.routes import health, ivg_runtime, robot_mesh
from aubo_ros2_web_dashboard.gateway.routes.upstream_proxy import http_proxy_router, ws_router


class SecurityHeadersMiddleware:
    """安全响应头中间件 — 所有 HTTP 响应注入 X-Content-Type-Options: nosniff。"""

    def __init__(self, app: ASGIApp) -> None:
        self.app = app

    async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        async def send_with_headers(message: Message) -> None:
            if message["type"] == "http.response.start":
                MutableHeaders(raw=message["headers"]).setdefault(
                    "X-Content-Type-Options", "nosniff")
            await send(message)

        await self.app(scope, receive, send_with_headers)


def create_app(web_root: str, *, rwt_override: str | None = None) -> FastAPI:
    """创建 FastAPI 应用。

    Args:
        web_root: 前端静态文件根目录 (share/.../web/public)
        rwt_override: 覆盖 robotwebtools 资产目录（CLI --rwt-assets-dir）
    """
    root = os.path.abspath(os.path.realpath(web_root))
    if not os.path.isdir(root):
        raise ValueError(f"静态目录不存在: {root}")

    # RobotWebTools 资产路径（Vue 3 通过 npm 管理 roslib/ros3d，此目录可选）
    embedded = os.path.join(root, "js", "robotwebtools")
    rwt_root = os.path.abspath(os.path.realpath(rwt_override or embedded))
    rwt_available = os.path.isdir(rwt_root)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        """启动时打印代理目标地址，便于调试。"""
        rb = f"{cfg.rosbridge_host()}:{cfg.rosbridge_port()}"
        wv = f"{cfg.web_video_host()}:{cfg.web_video_port()}"
        print(f"[ivg] 网关启动 root={root} rwt={rwt_root if rwt_available else '(无)'} "
              f"rosbridge→{rb} web_video→{wv}", flush=True)
        yield

    # 创建应用
    app = FastAPI(
        title="IVG Web Dashboard",
        docs_url=None, redoc_url=None, openapi_url=None,  # 不暴露 API 文档
        lifespan=lifespan,
    )
    app.state.static_root = root

    # 中间件（后添加的先执行）
    app.add_middleware(GZipMiddleware, minimum_size=cfg.gateway_gzip_min_size())
    app.add_middleware(SecurityHeadersMiddleware)

    # 路由注册（顺序决定优先级）
    app.include_router(ws_router)           # WebSocket 代理
    app.include_router(http_proxy_router)   # HTTP 视频流代理
    app.include_router(health.router)       # /health
    app.include_router(ivg_runtime.router)  # /api/v1/runtime + /api/v1/settings
    app.include_router(robot_mesh.router)   # /api/ivg/robot-mesh/*

    # 静态文件挂载 — Vite 构建产物 (JS/CSS 在 /assets/ 下)
    asset_dir = os.path.join(root, "assets")
    if os.path.isdir(asset_dir):
        app.mount("/assets",
                  StaticFiles(directory=asset_dir, html=False), name="assets")

    if rwt_available:
        app.mount("/js/robotwebtools",
                  StaticFiles(directory=rwt_root, html=False), name="robotwebtools")

    # SPA fallback: 所有未匹配路径返回 index.html (Vue Router createWebHistory 模式)
    from fastapi.responses import FileResponse
    index_html = os.path.join(root, "index.html")

    @app.get("/{full_path:path}")
    async def spa_fallback(full_path: str):
        # /api/, /ws/, /health 已经由上方路由处理，此处仅处理 SPA 页面路由
        if os.path.isfile(index_html):
            return FileResponse(index_html)
        return {"detail": "index.html not found"}

    # 根路径也走 SPA
    @app.get("/")
    async def root_fallback():
        if os.path.isfile(index_html):
            return FileResponse(index_html)
        return {"detail": "index.html not found"}

    return app
