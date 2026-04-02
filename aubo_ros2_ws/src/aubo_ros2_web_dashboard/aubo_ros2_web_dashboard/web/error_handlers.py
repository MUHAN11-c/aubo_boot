"""统一 JSON 错误体（含 request_id），便于监控与前端解析。"""

from __future__ import annotations

import logging
from typing import Any

from fastapi import HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

LOGGER = logging.getLogger(__name__)


def _rid(request: Request) -> str | None:
    """取自 RequestIdMiddleware 写入的 request.state.request_id。"""
    return getattr(request.state, "request_id", None)


def _error_body(code: str, message: Any, request_id: str | None, extra: dict | None = None) -> dict:
    """构造统一错误 JSON 的 body（不含 HTTP 状态码）。"""
    msg = message if isinstance(message, str) else str(message)
    body: dict = {
        "error": {
            "code": code,
            "message": msg,
            "request_id": request_id,
        }
    }
    if extra:
        body["error"].update(extra)
    return body


def error_json_response(request: Request, status: int, code: str, message: str, **extra: Any) -> JSONResponse:
    """代理层与根路由返回与全局异常一致的结构。"""
    b = _error_body(code, message, _rid(request))
    if extra:
        b["error"].update(extra)
    return JSONResponse(status_code=status, content=b)


async def http_exception_handler(request: Request, exc: HTTPException) -> JSONResponse:
    """将路由中 raise 的 HTTPException 转为统一 error 结构。"""
    detail = exc.detail
    msg = detail if isinstance(detail, str) else str(detail)
    return JSONResponse(
        status_code=exc.status_code,
        content=_error_body(f"http_{exc.status_code}", msg, _rid(request)),
        headers=dict(exc.headers) if exc.headers else {},
    )


async def validation_exception_handler(request: Request, exc: RequestValidationError) -> JSONResponse:
    """Pydantic 请求体验证失败（422）。"""
    return JSONResponse(
        status_code=422,
        content=_error_body(
            "validation_error",
            "请求参数校验失败",
            _rid(request),
            {"details": exc.errors()},
        ),
    )


async def unhandled_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """兜底：未捕获异常记日志并返回 500，不向客户端泄露堆栈。"""
    LOGGER.exception("未处理异常 request_id=%s", _rid(request))
    return JSONResponse(
        status_code=500,
        content=_error_body("internal_error", "服务器内部错误", _rid(request)),
    )
