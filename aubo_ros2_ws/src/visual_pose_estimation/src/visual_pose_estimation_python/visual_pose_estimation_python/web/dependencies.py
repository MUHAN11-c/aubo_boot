"""FastAPI Depends：从 request.app.state 注入路径、ROS 桥、业务服务、WS 管理器。"""

from __future__ import annotations

from fastapi import Request

from .resources import WebPaths
from .ros_bridge import RosBridgeManager
from .services import NativeWebService
from .ws import WebSocketManager


def get_paths(request: Request) -> WebPaths:
    return request.app.state.paths


def get_ros_bridge(request: Request) -> RosBridgeManager:
    return request.app.state.ros_bridge


def get_native_service(request: Request) -> NativeWebService:
    return request.app.state.native_service


def get_ws_manager(request: Request) -> WebSocketManager:
    return request.app.state.ws_manager
