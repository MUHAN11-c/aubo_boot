"""
组装 FastAPI 应用：生命周期内启停 ROS 桥、注册路由与静态资源、挂载 app.state.


命令行入口 main() 亦在此（setup.py 的 visual_pose_estimation_web 指向本模块）。
"""

from __future__ import annotations

import argparse
import logging
import os
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from ..path_resolver import resolve_web_paths
from .ros_bridge import RosBridgeManager
from .routers import api_router, router
from .services import NativeWebService

LOGGER = logging.getLogger(__name__)


def create_app() -> FastAPI:
    """构建应用实例（Uvicorn factory 模式）；每次调用返回新实例."""
    paths = resolve_web_paths()
    ros_bridge = RosBridgeManager(paths)
    native_service = NativeWebService(ros_bridge)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        app.state.ros_bridge.start()
        try:
            yield
        finally:
            app.state.ros_bridge.stop()

    app = FastAPI(
        title="visual_pose_estimation_python web",
        version="0.1.0",
        lifespan=lifespan,
    )

    app.state.paths = paths
    app.state.ros_bridge = ros_bridge
    app.state.native_service = native_service

    # 浏览器直连或跨端口调试时允许跨域（现场部署可按需收紧 allow_origins）
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

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
    parser.add_argument("--port", type=int, default=8088, help="Bind port")
    parser.add_argument("--reload", action="store_true", help="Enable auto reload")
    args, _unknown_args = parser.parse_known_args()
    os.environ["VPE_WEB_HOST"] = args.host
    os.environ["VPE_WEB_PORT"] = str(args.port)

    # reload 子进程需要可导入路径；factory 与 create_app 一致
    uvicorn.run(
        "visual_pose_estimation_python.web.app:create_app",
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
