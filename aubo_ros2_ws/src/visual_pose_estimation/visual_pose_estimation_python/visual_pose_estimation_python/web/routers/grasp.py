"""抓取与夹爪 HTTP：单次抓取、循环控制、快换等。"""

from __future__ import annotations

from fastapi import APIRouter, Body, Depends

from ..dependencies import get_native_service
from ..services import NativeWebService


router = APIRouter(prefix="/api", tags=["grasp"])


@router.post("/execute_single_grasp")
def execute_single_grasp(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.execute_single_grasp(payload)


@router.post("/loop_grasp_control")
def loop_grasp_control(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.loop_grasp_control(payload)


@router.post("/publish_grasps_loop_control")
def publish_grasps_loop_control(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.publish_grasps_loop_control(payload)


@router.post("/run_gripper_swap")
def run_gripper_swap(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.run_gripper_swap(payload)
