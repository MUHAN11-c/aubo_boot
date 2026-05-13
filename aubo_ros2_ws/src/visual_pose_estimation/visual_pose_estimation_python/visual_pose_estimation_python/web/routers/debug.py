"""调试可视化 HTTP：多路图像、参数滑块、阈值保存等。"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Body, Depends

from ..dependencies import get_native_service
from ..services import NativeWebService


router = APIRouter(prefix="/api", tags=["debug"])


@router.post("/save_debug_features")
def save_debug_features(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.save_debug_features(payload)


@router.post("/debug/capture")
def debug_capture(
    payload: Optional[dict] = Body(default=None),
    service: NativeWebService = Depends(get_native_service),
):
    return service.debug_capture(payload)


@router.post("/debug/get_images")
def debug_get_images(
    service: NativeWebService = Depends(get_native_service),
):
    return service.debug_get_images()


@router.post("/debug/update_params")
def debug_update_params(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.debug_update_params(payload)


@router.post("/debug/get_params")
def debug_get_params(
    service: NativeWebService = Depends(get_native_service),
):
    return service.debug_get_params()


@router.post("/debug/save_thresholds")
def debug_save_thresholds(
    service: NativeWebService = Depends(get_native_service),
):
    return service.debug_save_thresholds()
