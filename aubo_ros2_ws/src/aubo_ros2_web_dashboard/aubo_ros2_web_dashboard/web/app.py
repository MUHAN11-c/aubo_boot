"""
FastAPI 统一网关应用工厂（本进程不加载 rclpy）。

职责概览：
- 挂载 RWT 前端静态资源（/rwt）与 visual_pose_estimation_python 的 web_ui（/legacy-ui、/static）。
- 将 /health、/status、/exit、/api/*（除 config）反向代理到上游 VPE（默认 127.0.0.1:8088）。
- 在网关本机提供 /api/config、/gateway/health 等，供浏览器 rosbridge 与运维探活。

路由注册顺序很重要：先注册具体 APIRouter，再 mount 静态目录，避免通配路径抢匹配。
"""

from __future__ import annotations

import logging
import os
from contextlib import asynccontextmanager

import httpx
from fastapi import FastAPI, HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware

from .enterprise import (
    ProcessTimeMiddleware,
    RequestIdMiddleware,
    SecurityHeadersMiddleware,
    cors_allow_origins,
)
from .error_handlers import (
    error_json_response,
    http_exception_handler,
    unhandled_exception_handler,
    validation_exception_handler,
)
from .resources import resolve_web_paths
from .routers import system, vpe_proxy
from .vpe_resources import resolve_vpe_web_paths

LOGGER = logging.getLogger(__name__)


def create_app() -> FastAPI:
    # 已安装的前端与 legacy 资源是否存在（决定 mount 与根路径重定向）
    paths = resolve_web_paths()
    dist_ok = paths.dist_dir.is_dir() and (paths.dist_dir / "index.html").is_file()
    vpe_paths = resolve_vpe_web_paths()
    legacy_ok = bool(vpe_paths and vpe_paths.legacy_ui_dir.is_dir())

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # 全局异步 HTTP 客户端，供 VPE 反向代理复用连接
        # trust_env=False：勿读取 http_proxy/ALL_PROXY 等；本机上游多为 127.0.0.1，且 socks:// 会令 httpx 启动失败
        _trust_env = os.environ.get("AUBO_WEB_HTTPX_TRUST_ENV", "").lower() in ("1", "true", "yes")
        app.state.http_client = httpx.AsyncClient(
            timeout=httpx.Timeout(900.0),
            limits=httpx.Limits(max_keepalive_connections=8, max_connections=32),
            trust_env=_trust_env,
        )
        if not dist_ok:
            LOGGER.warning("RWT dist missing at %s", paths.dist_dir)
        if not legacy_ok:
            LOGGER.warning(
                "visual_pose_estimation_python web_ui 未安装或路径无效；"
                "请 colcon 安装该包以使用 /legacy-ui 与完整 Web UI。"
            )
        yield
        await app.state.http_client.aclose()

    app = FastAPI(
        title="aubo_ros2_web_dashboard",
        version="0.2.0",
        lifespan=lifespan,
        description="IVG 统一 Web：RWT（/rwt 演示与开发调试）+ legacy-ui + 代理 VPE FastAPI（默认 :8088）",
    )

    # Starlette：后添加的中间件更靠近应用内核；CORS 通常靠内层，最外层为 RequestId
    _origins = cors_allow_origins()
    _creds = "*" not in _origins  # 规范要求 allow_origins=["*"] 时不可带 credentials
    app.add_middleware(
        CORSMiddleware,
        allow_origins=_origins,
        allow_credentials=_creds,
        allow_methods=["*"],
        allow_headers=["*"],
        expose_headers=["X-Request-ID", "X-Process-Time"],
    )
    app.add_middleware(SecurityHeadersMiddleware)
    app.add_middleware(ProcessTimeMiddleware)
    app.add_middleware(RequestIdMiddleware)
    # 生产可设 AUBO_WEB_TRUSTED_HOSTS=example.com,192.168.1.1 限制 Host 头
    _trusted = os.environ.get("AUBO_WEB_TRUSTED_HOSTS", "").strip()
    if _trusted:
        hosts = [h.strip() for h in _trusted.split(",") if h.strip()]
        if hosts:
            app.add_middleware(TrustedHostMiddleware, allowed_hosts=hosts)

    app.add_exception_handler(HTTPException, http_exception_handler)
    app.add_exception_handler(RequestValidationError, validation_exception_handler)
    app.add_exception_handler(Exception, unhandled_exception_handler)

    app.include_router(system.router)
    app.include_router(vpe_proxy.router)

    @app.get("/", include_in_schema=False)
    @app.get("/index.html", include_in_schema=False)
    async def root_redirect(request: Request):
        # 优先进入 VPE 同源 legacy 主界面；否则进入 RWT 模式入口（/rwt/index.html）
        if legacy_ok:
            return RedirectResponse(url="/legacy-ui/index.html", status_code=307)
        if dist_ok:
            return RedirectResponse(url="/rwt/", status_code=307)
        return error_json_response(
            request,
            503,
            "service_unavailable",
            "未找到 legacy-ui 与 RWT 前端，请检查 colcon 安装。",
        )

    @app.get("/demo.html", include_in_schema=False)
    async def demo_redirect(request: Request):
        # 根路径 /demo.html：仅在有 legacy-ui 时转到 VPE 自带 demo；无则返回结构化 404
        if legacy_ok:
            return RedirectResponse(url="/legacy-ui/demo.html", status_code=307)
        return error_json_response(
            request,
            404,
            "not_found",
            "demo 页面需要 visual_pose_estimation_python web_ui",
        )

    if legacy_ok and vpe_paths is not None:
        app.mount(
            "/legacy-ui",
            StaticFiles(directory=str(vpe_paths.legacy_ui_dir), html=True),
            name="legacy-ui",
        )
        static_dir = vpe_paths.static_dir if vpe_paths.static_dir.is_dir() else vpe_paths.legacy_ui_dir
        app.mount("/static", StaticFiles(directory=str(static_dir)), name="vpe-static")

    if dist_ok:
        app.mount("/rwt", StaticFiles(directory=str(paths.dist_dir), html=True), name="rwt")

    return app
