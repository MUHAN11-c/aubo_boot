"""相机相关 HTTP：采集现场图、模板图。"""

from __future__ import annotations

from fastapi import APIRouter, Body, Depends

from ..dependencies import get_native_service
from ..services import NativeWebService


router = APIRouter(prefix="/api", tags=["camera"])


@router.post("/capture_image")
def capture_image(
    payload: dict | None = Body(default=None),
    service: NativeWebService = Depends(get_native_service),
):
    return service.capture_image(payload)


@router.post("/capture_template_image")
def capture_template_image(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.capture_template_image(payload)
