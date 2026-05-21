"""FastAPI 应用工厂 — 组装中间件、路由、静态挂载。

路由注册顺序（优先级从高到低）:
  1. /ws/rosbridge              — WebSocket 代理 → rosbridge
  2. /api/ivg/proxy/web-video/* — HTTP 流代理 → web_video_server
  3. /health                    — 健康检查
  4. /api/v1/runtime            — 前端运行时配置 (BFF)
  5. /api/v1/tool-geometries     — 工具几何数据 (BFF, 与 tools.yaml 同步)
  6. /api/ivg/robot-mesh/*      — 机器人 3D 模型文件
  7. /js/robotwebtools/*        — RobotWebTools 静态资源
  8. /*                         — MPA 静态文件 (纯 HTML/JS, 零构建)

BFF 纯 HTTP 设计:
  零 ROS 依赖 — 不初始化 rclpy, 不查询 TF, 不 import ROS 模块。
  start_pose 由前端通过 rosbridge 获取后传入, BFF 仅做纯数学轨迹生成喵~
"""
from __future__ import annotations

import logging
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from starlette.datastructures import MutableHeaders
from starlette.middleware.gzip import GZipMiddleware
from starlette.types import ASGIApp, Message, Receive, Scope, Send

from aubo_ros2_web_dashboard import config as cfg
from aubo_ros2_web_dashboard.gateway.routes import health, ivg_runtime, robot_mesh, tool_geometries, latte_trajectory_preview
from aubo_ros2_web_dashboard.gateway.routes.upstream_proxy import http_proxy_router, ws_router

_logger = logging.getLogger("gateway.app")


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

    # RobotWebTools 资产路径 — 通过 importmap 为 MPA 页面提供 ros3d/roslib/three.js 喵~
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
        _logger.info("网关启动 root=%s rwt=%s rosbridge→%s web_video→%s",
                     root, rwt_root if rwt_available else '(无)', rb, wv)
        yield
        # 无需清理 (BFF 零 ROS 依赖) 喵~

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
    app.include_router(tool_geometries.router)  # /api/v1/tool-geometries
    app.include_router(latte_trajectory_preview.router)  # /api/v1/latte/trajectory/*
    app.include_router(robot_mesh.router)   # /api/ivg/robot-mesh/*

    # RobotWebTools 资产 — importmap + ros3d/roslib/three.js 模块（优先级高于根挂载）
    if rwt_available:
        app.mount("/js/robotwebtools",
                  StaticFiles(directory=rwt_root, html=False), name="robotwebtools")

    # MPA 静态文件服务 — 直接提供 web/public/ 下所有 HTML/CSS/JS 文件喵~
    # html=True 使 / 自动解析为 index.html，其他路径按文件名直接提供
    # API 路由（上面注册的）优先级高于此挂载，不会被拦截
    if os.path.isdir(root):
        app.mount("/", StaticFiles(directory=root, html=True), name="public")

    return app
