"""视觉位姿 Web：FastAPI 应用入口（工厂函数 create_app，供 Uvicorn / 测试加载）。"""

from .app import create_app

__all__ = ["create_app"]
