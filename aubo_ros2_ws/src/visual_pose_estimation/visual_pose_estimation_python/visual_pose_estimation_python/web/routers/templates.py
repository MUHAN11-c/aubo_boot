"""模板工件 HTTP：列表、读写姿态、标准化、模板图下载。"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Body, Depends, Query
from fastapi.responses import Response

from ..dependencies import get_native_service
from ..services import NativeWebService


router = APIRouter(prefix="/api", tags=["templates"])


@router.post("/save_template_pose")
def save_template_pose(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.save_template_pose(payload)


@router.post("/list_templates")
def list_templates(
    payload: Optional[dict] = Body(default=None),
    service: NativeWebService = Depends(get_native_service),
):
    return service.list_templates(payload)


@router.post("/list_workpiece_ids")
def list_workpiece_ids(
    service: NativeWebService = Depends(get_native_service),
):
    return service.list_workpiece_ids()


@router.post("/read_template_pose")
def read_template_pose(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.read_template_pose(payload)


@router.post("/standardize_template")
def standardize_template(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.standardize_template(payload)


@router.get("/get_template_image")
def get_template_image(
    workpiece_id: str = Query(...),
    pose_id: str = Query(...),
    image_name: str = Query("gripper_visualization.jpg"),
    service: NativeWebService = Depends(get_native_service),
):
    image_data, media_type = service.get_template_image(workpiece_id, pose_id, image_name)
    return Response(content=image_data, media_type=media_type)
