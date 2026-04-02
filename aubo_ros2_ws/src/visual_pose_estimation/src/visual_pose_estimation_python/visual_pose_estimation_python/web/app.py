"""组装 FastAPI 应用：生命周期内启停 ROS 桥、注册路由与静态资源、挂载 app.state。"""

from __future__ import annotations

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .resources import resolve_web_paths
from .ros_bridge import RosBridgeManager
from .routers import camera, debug, grasp, pose, robot, system, templates
from .services import NativeWebService
from .ws import WebSocketManager


LOGGER = logging.getLogger(__name__)


def create_app() -> FastAPI:
    """构建应用实例（Uvicorn factory 模式）；每次调用返回新实例。"""
    paths = resolve_web_paths()
    ros_bridge = RosBridgeManager(paths)
    ws_manager = WebSocketManager()
    native_service = NativeWebService(ros_bridge)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # 启动 ROS2 桥接线程，并向已连接 WebSocket 广播生命周期事件
        app.state.ros_bridge.start()
        await app.state.ws_manager.broadcast_json(
            {
                "type": "lifecycle",
                "payload": {"event": "startup", "ros_bridge": app.state.ros_bridge.status()},
            }
        )
        try:
            yield
        finally:
            await app.state.ws_manager.broadcast_json({"type": "lifecycle", "payload": {"event": "shutdown"}})
            app.state.ros_bridge.stop()

    app = FastAPI(
        title="visual_pose_estimation_python web",
        version="0.1.0",
        lifespan=lifespan,
    )

    app.state.paths = paths
    app.state.ros_bridge = ros_bridge
    app.state.ws_manager = ws_manager
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

    app.include_router(system.router)
    app.include_router(camera.router)
    app.include_router(pose.router)
    app.include_router(templates.router)
    app.include_router(robot.router)
    app.include_router(grasp.router)
    app.include_router(debug.router)
    return app
