"""
组装 FastAPI 应用：生命周期内启停 ROS 桥、注册路由与静态资源、挂载 app.state.


命令行入口 main() 亦在此（setup.py 的 ivg_pose_estimation_web 指向本模块）。
"""

from __future__ import annotations

import argparse
import logging
import os
import secrets
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from starlette.types import ASGIApp, Receive, Scope, Send

from ..path_resolver import resolve_web_paths
from .ros_bridge import RosBridgeManager
from .routers import api_router, router
from .services import NativeWebService

LOGGER = logging.getLogger(__name__)

AUTH_COOKIE = "vpe_auth"


class AuthCookieMiddleware:
    """
    给每个响应种 SameSite=Strict 会话 cookie（写端点守卫的凭证）.

    UI 与 API 同源，浏览器自动携带 cookie；SameSite=Strict 阻断跨站
    CSRF（攻击页无法为我们的源种/带此 cookie）。curl 等本地客户端改用
    启动日志打印的 X-Auth-Token 头。
    """

    def __init__(self, app: ASGIApp, token: str) -> None:
        self.app = app
        self.token = token

    async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        async def send_with_cookie(message):
            if message["type"] == "http.response.start":
                headers = message.setdefault("headers", [])
                headers.append(
                    (
                        b"set-cookie",
                        f"{AUTH_COOKIE}={self.token}; Path=/; SameSite=Strict".encode(),
                    )
                )
            await send(message)

        await self.app(scope, receive, send_with_cookie)


def create_app() -> FastAPI:
    """构建应用实例（Uvicorn factory 模式）；每次调用返回新实例."""
    paths = resolve_web_paths()
    ros_bridge = RosBridgeManager(paths)
    native_service = NativeWebService(ros_bridge)

    # 写端点守卫 token：环境变量覆盖（操作员固定口令），否则每次启动随机
    auth_token = os.environ.get("VPE_WEB_TOKEN") or secrets.token_urlsafe(24)
    LOGGER.info(
        "Web 写端点 token：%s（curl 用 -H 'X-Auth-Token: <token>'；"
        "同源 UI 走 vpe_auth cookie 免填；可用环境变量 VPE_WEB_TOKEN 固定）",
        auth_token,
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        app.state.ros_bridge.start()
        try:
            yield
        finally:
            app.state.ros_bridge.stop()

    app = FastAPI(
        title="ivg_pose_estimation web",
        version="0.1.0",
        lifespan=lifespan,
    )

    app.state.paths = paths
    app.state.ros_bridge = ros_bridge
    app.state.native_service = native_service
    app.state.auth_token = auth_token
    app.add_middleware(AuthCookieMiddleware, token=auth_token)

    # UI 与 API 同源，不开跨域（历史通配 CORS 已移除：跨站页可发匿名
    # POST 打挂 /exit 与调试写端点；写端点现由 token 守卫兜底）

    if paths.static_dir.exists():
        app.mount("/static", StaticFiles(directory=str(paths.static_dir)), name="static")
    else:
        LOGGER.warning("Static directory not found: %s", paths.static_dir)

    if paths.legacy_ui_dir.exists():
        app.mount("/legacy-ui", StaticFiles(directory=str(paths.legacy_ui_dir), html=True), name="legacy-ui")
    else:
        LOGGER.warning("Legacy UI directory not found: %s", paths.legacy_ui_dir)

    app.include_router(router)
    app.include_router(api_router)
    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the FastAPI web service")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host")
    # 8089：与 aubo_hand_eye_calibration 网关（8088）错开，两者可同时运行
    parser.add_argument("--port", type=int, default=8089, help="Bind port")
    parser.add_argument("--reload", action="store_true", help="Enable auto reload")
    args, _unknown_args = parser.parse_known_args()
    os.environ["VPE_WEB_HOST"] = args.host
    os.environ["VPE_WEB_PORT"] = str(args.port)

    # reload 子进程需要可导入路径；factory 与 create_app 一致
    uvicorn.run(
        "ivg_pose_estimation.web.app:create_app",
        factory=True,
        host=args.host,
        port=args.port,
        reload=args.reload,
        reload_excludes=[
            "build/**",
            "install/**",
            "log/**",
            "rosbags/**",
            "**/__pycache__/*",
            "*.pyc",
        ] if args.reload else None,
        reload_includes=["*.py", "*.yaml", "*.yml", "*.json"] if args.reload else None,
    )


if __name__ == "__main__":
    main()
