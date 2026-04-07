"""异步 SQLAlchemy：引擎、会话工厂、建表。"""
from collections.abc import AsyncGenerator

from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine

from ivg_gateway.config import get_settings
from ivg_gateway.models import Base

settings = get_settings()
engine = create_async_engine(
    settings.database_url,
    echo=False,  # True 时打印 SQL，调试用
)
# 每个请求/任务应 async with SessionLocal() 短生命周期使用
SessionLocal = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)


async def get_session() -> AsyncGenerator[AsyncSession, None]:
    """可选依赖生成器；路由中更常用 deps.get_db。"""
    async with SessionLocal() as session:
        yield session


async def init_db() -> None:
    """根据 models.Base.metadata 创建缺失的表（不会删改已有表结构）。"""
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
