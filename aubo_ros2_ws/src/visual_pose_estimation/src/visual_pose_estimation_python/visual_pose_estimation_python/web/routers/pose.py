from __future__ import annotations

from fastapi import APIRouter, Body, Depends

from ..dependencies import get_native_service
from ..services import NativeWebService


router = APIRouter(prefix="/api", tags=["pose"])


@router.post("/estimate_pose")
def estimate_pose(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.estimate_pose(payload)
