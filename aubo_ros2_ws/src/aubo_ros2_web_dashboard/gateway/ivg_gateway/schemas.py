"""Pydantic 请求/响应模型：与 REST 路由及 OpenAPI 文档对应。"""
from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


class Token(BaseModel):
    access_token: str
    token_type: str = "bearer"


class TokenPayload(BaseModel):
    sub: str
    role: str = "operator"


class UserRead(BaseModel):
    id: str
    username: str
    role: str
    is_active: bool
    created_at: datetime

    model_config = {"from_attributes": True}


class TaskCreate(BaseModel):
    name: str = Field(..., min_length=1, max_length=256)
    payload_json: Optional[str] = None


class TaskRead(BaseModel):
    id: str
    name: str
    status: str
    payload_json: Optional[str] = None
    created_at: datetime

    model_config = {"from_attributes": True}


class TrajectoryCreate(BaseModel):
    name: str = Field(..., min_length=1, max_length=256)
    data_json: Optional[str] = None
    task_id: Optional[str] = None


class TrajectoryRead(BaseModel):
    id: str
    name: str
    data_json: Optional[str] = None
    task_id: Optional[str] = None
    created_at: datetime

    model_config = {"from_attributes": True}


class VisionInferenceRequest(BaseModel):
    """占位：后续接视觉推理 / 大模型流水线。"""

    prompt: Optional[str] = None
    image_base64: Optional[str] = None


class VisionInferenceResponse(BaseModel):
    ok: bool = False
    message: str = "未实现：请在此接入模型服务"
