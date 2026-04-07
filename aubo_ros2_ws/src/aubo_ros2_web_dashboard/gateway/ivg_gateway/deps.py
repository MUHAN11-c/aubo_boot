"""
FastAPI 依赖注入：数据库会话、当前用户、角色门槛。
HTTP API 使用 Authorization: Bearer；WebSocket 使用 Query token（见 main.rosbridge_proxy）。
"""
from collections.abc import AsyncGenerator
from typing import Annotated

from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ivg_gateway.auth_core import decode_token
from ivg_gateway.database import SessionLocal
from ivg_gateway.models import User, UserRole

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/token")


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """请求级会话，yield 后由 FastAPI 关闭。"""
    async with SessionLocal() as session:
        yield session


async def get_current_user(
    token: Annotated[str, Depends(oauth2_scheme)],
    session: Annotated[AsyncSession, Depends(get_db)],
) -> User:
    """解析 Bearer JWT，再查库确认用户仍存在且未禁用。"""
    payload = decode_token(token)
    if not payload or "sub" not in payload:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="无效令牌")
    username = str(payload["sub"])
    r = await session.execute(select(User).where(User.username == username))
    user = r.scalar_one_or_none()
    if user is None or not user.is_active:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="用户无效")
    return user


def require_operator(user: Annotated[User, Depends(get_current_user)]) -> User:
    """viewer 不可写任务/轨迹/急停等。"""
    if user.role not in (UserRole.operator.value, UserRole.admin.value):
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="需要 operator 或 admin 角色")
    return user


def require_admin(user: Annotated[User, Depends(get_current_user)]) -> User:
    """预留：仅管理员可调的管理接口。"""
    if user.role != UserRole.admin.value:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="需要 admin 角色")
    return user
