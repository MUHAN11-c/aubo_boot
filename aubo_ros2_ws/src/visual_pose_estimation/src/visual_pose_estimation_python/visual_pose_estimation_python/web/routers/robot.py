from __future__ import annotations

from fastapi import APIRouter, Body, Depends

from ..dependencies import get_native_service
from ..services import NativeWebService


router = APIRouter(prefix="/api", tags=["robot"])


@router.post("/get_robot_status")
def get_robot_status(
    service: NativeWebService = Depends(get_native_service),
):
    return service.get_robot_status()


@router.post("/set_robot_pose")
def set_robot_pose(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.set_robot_pose(payload)


@router.post("/set_robot_io")
def set_robot_io(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.set_robot_io(payload)


@router.post("/execute_pose_sequence")
def execute_pose_sequence(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.execute_pose_sequence(payload)
