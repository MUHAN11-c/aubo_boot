"""
FastAPI 统一静态网关（实现子包）。

阅读顺序建议：
  1. ``settings`` — 环境变量键名、默认值、``runtime-config`` 字典
  2. ``routes/*`` — 各 URL 行为（健康、配置、上游代理）
  3. ``app.create_app`` — 组装 FastAPI：中间件 → 路由 → 最后挂 StaticFiles
  4. ``cli.main`` — 解析命令行并用 Uvicorn 监听端口

对外常用：``from aubo_ros2_web_dashboard.gateway import create_app``（测试里工厂注入）。
"""

from aubo_ros2_web_dashboard.gateway.app import create_app

__all__ = ["create_app"]
