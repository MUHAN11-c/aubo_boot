from __future__ import annotations

import logging
import os
import signal
import threading
import time

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

from ..dependencies import get_paths, get_ros_bridge
from ..resources import WebPaths
from ..ros_bridge import RosBridgeManager
from ..ws import WebSocketManager


router = APIRouter(tags=["system"])
LOGGER = logging.getLogger(__name__)


def schedule_exit(delay_seconds: float = 1.0) -> None:
    def _shutdown_later():
        threading.Event().wait(delay_seconds)
        os.kill(os.getpid(), signal.SIGTERM)

    thread = threading.Thread(target=_shutdown_later, name="fastapi-exit", daemon=True)
    thread.start()
    LOGGER.info("Scheduled FastAPI process shutdown in %.1fs", delay_seconds)


@router.get("/", response_class=FileResponse)
@router.get("/index.html", response_class=FileResponse)
def index(paths: WebPaths = Depends(get_paths)):
    return FileResponse(paths.index_file)


@router.get("/status")
def status(ros_bridge: RosBridgeManager = Depends(get_ros_bridge)):
    payload = {
        "status": "online",
        "port": int(os.environ.get("VPE_WEB_PORT", "8088")),
        "service": "visual_pose_estimation_fastapi_web",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "ros_bridge": ros_bridge.status(),
    }
    return JSONResponse(payload)


@router.get("/health")
def health(ros_bridge: RosBridgeManager = Depends(get_ros_bridge)):
    return JSONResponse(
        {
            "ok": True,
            "ros_bridge_ready": ros_bridge.is_ready,
            "startup_error": ros_bridge.startup_error,
        }
    )


@router.post("/exit")
async def exit_service():
    schedule_exit()
    return JSONResponse({"status": "success", "message": "服务正在退出..."})


@router.websocket("/ws")
async def websocket_endpoint(
    websocket: WebSocket,
):
    ws_manager: WebSocketManager = websocket.app.state.ws_manager
    ros_bridge: RosBridgeManager = websocket.app.state.ros_bridge
    await ws_manager.connect(websocket)
    await ws_manager.send_json(
        websocket,
        {
            "type": "status",
            "payload": {
                "service": "visual_pose_estimation_fastapi_web",
                "ros_bridge": ros_bridge.status(),
            },
        },
    )

    try:
        while True:
            message = await websocket.receive_json()
            if message.get("action") == "ping":
                await ws_manager.send_json(websocket, {"type": "pong"})
            elif message.get("action") == "status":
                await ws_manager.send_json(
                    websocket,
                    {"type": "status", "payload": {"ros_bridge": ros_bridge.status()}},
                )
    except WebSocketDisconnect:
        await ws_manager.disconnect(websocket)
