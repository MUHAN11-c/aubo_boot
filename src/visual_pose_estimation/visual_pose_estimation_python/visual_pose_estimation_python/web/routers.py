"""
HTTP 路由层：system / camera / pose / templates / robot / grasp / debug 全部端点.


端点与路径保持与旧 routers/ 包一致；本区不发运动/IO（robot/grasp 相关端点
由服务层返回 501）。Depends 注入函数亦集中于此。
"""

from __future__ import annotations

import logging
import os
import signal
import threading
import time
from typing import Optional

from fastapi import APIRouter, Body, Depends, Query, Request
from fastapi.responses import JSONResponse, RedirectResponse, Response

from .ros_bridge import RosBridgeManager
from .services import NativeWebService


router = APIRouter(tags=["system"])
api_router = APIRouter(prefix="/api")
LOGGER = logging.getLogger(__name__)


# ---------- Depends ----------
def get_ros_bridge(request: Request) -> RosBridgeManager:
    return request.app.state.ros_bridge


def get_native_service(request: Request) -> NativeWebService:
    return request.app.state.native_service


# ---------- system ----------
def schedule_exit(delay_seconds: float = 1.0) -> None:
    def _shutdown_later():
        threading.Event().wait(delay_seconds)
        os.kill(os.getpid(), signal.SIGTERM)

    thread = threading.Thread(target=_shutdown_later, name="fastapi-exit", daemon=True)
    thread.start()
    LOGGER.info("Scheduled FastAPI process shutdown in %.1fs", delay_seconds)


# 主界面在 /legacy-ui/ 下，相对路径的 assets、scripts 才能正确加载。
# 若直接返回 web_ui/index.html，浏览器地址为 / 时会请求 /scripts/app.js（404），界面残缺。
_LEGACY_UI_ENTRY = "/legacy-ui/index.html"
_LEGACY_UI_DEMO = "/legacy-ui/demo.html"


@router.get("/")
@router.get("/index.html")
def index():
    return RedirectResponse(url=_LEGACY_UI_ENTRY, status_code=307)


@router.get("/demo.html")
def demo_page():
    """根路径 /demo.html → 正式演示页（/demo 由 StaticFiles 挂载占用，请用 /demo/index.html 或本路由）."""
    return RedirectResponse(url=_LEGACY_UI_DEMO, status_code=307)


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


# ---------- camera ----------
@api_router.post("/capture_image")
def capture_image(
    payload: Optional[dict] = Body(default=None),
    service: NativeWebService = Depends(get_native_service),
):
    return service.capture_image(payload)


@api_router.post("/capture_template_image")
def capture_template_image(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.capture_template_image(payload)


# ---------- pose ----------
@api_router.post("/estimate_pose")
def estimate_pose(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.estimate_pose(payload)


@api_router.post("/estimate_pose_2d")
def estimate_pose_2d(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.estimate_pose_2d(payload)


# ---------- templates ----------
@api_router.post("/save_template_pose")
def save_template_pose(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.save_template_pose(payload)


@api_router.post("/list_templates")
def list_templates(
    payload: Optional[dict] = Body(default=None),
    service: NativeWebService = Depends(get_native_service),
):
    return service.list_templates(payload)


@api_router.post("/list_workpiece_ids")
def list_workpiece_ids(
    service: NativeWebService = Depends(get_native_service),
):
    return service.list_workpiece_ids()


@api_router.post("/read_template_pose")
def read_template_pose(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.read_template_pose(payload)


@api_router.post("/standardize_template")
def standardize_template(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.standardize_template(payload)


@api_router.get("/get_template_image")
def get_template_image(
    workpiece_id: str = Query(...),
    pose_id: str = Query(...),
    image_name: str = Query("gripper_visualization.jpg"),
    service: NativeWebService = Depends(get_native_service),
):
    image_data, media_type = service.get_template_image(workpiece_id, pose_id, image_name)
    return Response(content=image_data, media_type=media_type)


# ---------- robot（本区不发运动/IO，服务层 501） ----------
@api_router.post("/get_robot_status")
def get_robot_status(service: NativeWebService = Depends(get_native_service)):
    return service.get_robot_status()


@api_router.post("/set_robot_pose")
def set_robot_pose(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.set_robot_pose(payload)


@api_router.post("/set_robot_io")
def set_robot_io(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.set_robot_io(payload)


@api_router.post("/execute_pose_sequence")
def execute_pose_sequence(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.execute_pose_sequence(payload)


# ---------- grasp（本区不执行，服务层 501） ----------
@api_router.post("/execute_single_grasp")
def execute_single_grasp(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.execute_single_grasp(payload)


@api_router.post("/loop_grasp_control")
def loop_grasp_control(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.loop_grasp_control(payload)


@api_router.post("/publish_grasps_loop_control")
def publish_grasps_loop_control(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.publish_grasps_loop_control(payload)


@api_router.post("/run_gripper_swap")
def run_gripper_swap(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.run_gripper_swap(payload)


# ---------- debug ----------
@api_router.post("/save_debug_features")
def save_debug_features(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.save_debug_features(payload)


@api_router.post("/debug/capture")
def debug_capture(
    payload: Optional[dict] = Body(default=None),
    service: NativeWebService = Depends(get_native_service),
):
    return service.debug_capture(payload)


@api_router.post("/debug/get_images")
def debug_get_images(service: NativeWebService = Depends(get_native_service)):
    return service.debug_get_images()


@api_router.post("/debug/update_params")
def debug_update_params(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.debug_update_params(payload)


@api_router.post("/debug/get_params")
def debug_get_params(service: NativeWebService = Depends(get_native_service)):
    return service.debug_get_params()


@api_router.post("/debug/save_thresholds")
def debug_save_thresholds(service: NativeWebService = Depends(get_native_service)):
    return service.debug_save_thresholds()
