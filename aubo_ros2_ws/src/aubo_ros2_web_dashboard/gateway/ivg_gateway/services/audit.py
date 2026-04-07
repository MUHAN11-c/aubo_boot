"""审计落库：单行插入并 commit；调用方勿在同一 session 上期望未 flush 的读。"""
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from ivg_gateway.models import AuditLog, User


async def write_audit(
    session: AsyncSession,
    *,
    username: str,
    action: str,
    detail: str | None = None,
    user_id: str | None = None,
) -> None:
    """写入一条审计；user_id 缺省时按 username 补查。"""
    if user_id is None:
        r = await session.execute(select(User.id).where(User.username == username))
        user_id = r.scalar_one_or_none()
    row = AuditLog(username=username, action=action, detail=detail, user_id=user_id)
    session.add(row)
    await session.commit()
