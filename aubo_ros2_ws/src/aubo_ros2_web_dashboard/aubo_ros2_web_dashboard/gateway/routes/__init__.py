"""
BFF 与代理路由（各文件一个或若干 ``APIRouter``）。

注册约定（见 ``gateway.app.create_app``）：
  必须在 ``app.mount("/", StaticFiles(...))`` **之前** ``include_router``，
  否则 ``/`` 静态挂载会先吞掉尚未匹配的 API/WebSocket 路径。
"""
