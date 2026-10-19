"""ros_web_link 应用工厂 — 可组合式 FastAPI 网关.

create_app() 支持三种扩展点（UI 层按需注入）：
  js_roots:      JS 静态根列表（通信层 + UI 层，按序挂载到 /js/）
  html_root:     HTML 静态根（UI 层独有，挂载到 /）
  sub_routers:   UI 层 BFF 子路由（FastAPI APIRouter 列表）

独立启动（纯传输，无 UI）：
  ros2 launch ros_web_link transport.launch.xml
  # 等价 rosbridge: 只起 WS 桥 + 网关，无页面

UI 组合启动：
  ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
  # 内部调 create_app(js_roots=[...], html_root=..., sub_routers=[...])
"""
from __future__ import annotations

import os
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.routing import WebSocketRoute

from . import config as cfg
from .routes.health import router as health_router
from .routes.foxglove_proxy import ws_router, http_proxy_router
from .routes.robot_mesh import router as mesh_router

__all__ = ['create_app']


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """添加安全响应头."""

    async def dispatch(self, request: Request, call_next):
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "SAMEORIGIN"
        return response


class NoCacheHtmlJsMiddleware(BaseHTTPMiddleware):
    """JS/HTML 响应强制协商缓存（no-cache）.

    Starlette StaticFiles 默认不发 Cache-Control，浏览器按启发式缓存且 IAB 实测
    不回源——源码更新后旧模块滞留（app.js 改动引用已删元素 → 空引用崩溃）。
    no-cache 保留 etag/last-modified 协商，304 代价极小。
    """

    async def dispatch(self, request: Request, call_next):
        response = await call_next(request)
        p = request.url.path
        if p.startswith("/js/") or p.endswith(".html") or p == "/":
            response.headers["Cache-Control"] = "no-cache"
        return response


class MultiStaticFiles(StaticFiles):
    """多根静态文件：按目录优先级查找（先通信层后 UI 层）."""

    def __init__(self, directories: list[str], **kwargs):
        self._directories = [d for d in directories if os.path.isdir(d)]
        if not self._directories:
            raise ValueError("MultiStaticFiles: at least one valid directory required")
        # 用第一个目录初始化父类（Starlette 要求）
        super().__init__(directory=self._directories[0], **kwargs)

    def lookup_path(self, path: str):
        """按序尝试每个目录，找到第一个匹配的文件.

        契约对齐 StaticFiles.lookup_path：返回 (full_path, stat_result)，
        stat_result 必须是 os.stat 结果（Starlette 0.27 会读 .st_mode 判文件类型，
        返回 mtime 浮点会 500）。
        """
        for d in self._directories:
            full_path = os.path.join(d, path)
            if os.path.isfile(full_path):
                return full_path, os.stat(full_path)
        # 未找到，返回 None
        return None, None


class SymlinkStaticFiles(StaticFiles):
    """符号链接树可用的静态文件（colcon --symlink-install 形态）.

    Starlette 0.27 的 lookup_path 用 realpath 做 containment 检查——install 树
    里的文件都是指向 build/ 的符号链接，realpath 解析出目录之外 → 全部 404。
    这里改用 normpath 的 .. 段检查防穿越，其余语义不变（返回 os.stat 结果）。
    """

    def lookup_path(self, path: str):
        if not path or ".." in path.split("/"):
            return None, None
        full_path = os.path.join(self.directory, path)
        if os.path.isfile(full_path):
            return full_path, os.stat(full_path)
        # 目录也要回 stat：get_response 据此走 html 模式的 index.html 回退
        if os.path.isdir(full_path):
            return full_path, os.stat(full_path)
        return None, None


def create_app(
    js_roots: list[str] | None = None,
    html_root: str | None = None,
    sub_routers: list | None = None,
) -> FastAPI:
    """创建 FastAPI 应用.

    Args:
        js_roots: JS 静态根目录列表（按序优先）。通信层自动包含在前，
                  UI 层可追加。每个根目录下的文件按相对路径挂载到 /js/。
        html_root: HTML/CSS 静态根（UI 层），挂载到 /（html=True）。
                   纯传输模式下为 None（不挂 HTML）。
        sub_routers: UI 层 BFF 子路由（APIRouter），include 优先于静态挂载。
    """
    # 默认 JS 根：本包 share 目录下的 web/public/js/
    own_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    own_js = os.path.join(own_share, "web", "public", "js")
    if not os.path.isdir(own_js):
        try:
            from ament_index_python.packages import get_package_share_directory
            own_js = os.path.join(get_package_share_directory("ros_web_link"), "web", "public", "js")
        except Exception:
            pass

    all_js_roots = [own_js] + [r for r in (js_roots or []) if r and os.path.isdir(r)]

    # 本包静态根（console.html 诊断页所在；纯传输模式兼作 HTML 根）
    own_public = os.path.dirname(own_js) if os.path.isdir(own_js) else ""

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        fb = f"{cfg.foxglove_bridge_host()}:{cfg.foxglove_bridge_port()}"
        print(f"[ros_web_link] 网关启动 js_roots={all_js_roots} foxglove→{fb}", flush=True)
        yield
        # 共享 httpx 客户端（视频代理连接池）随应用关闭
        from .routes.foxglove_proxy import aclose_shared_video_client
        await aclose_shared_video_client()

    app = FastAPI(
        title="ROS Web Link",
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
        lifespan=lifespan,
    )
    app.state.static_root = html_root or own_public

    # 中间件（后添加先执行）
    app.add_middleware(GZipMiddleware, minimum_size=cfg.gateway_gzip_min_size())
    app.add_middleware(NoCacheHtmlJsMiddleware)
    app.add_middleware(SecurityHeadersMiddleware)

    # 1) 传输路由（最优先）
    app.include_router(health_router)
    app.include_router(ws_router)
    app.include_router(http_proxy_router)
    app.include_router(mesh_router)

    # 2) UI 层 BFF 子路由
    for router in (sub_routers or []):
        app.include_router(router)

    # 3) JS 静态（仅纯传输模式挂载；组合模式下 UI 根挂载 / 已覆盖 JS 路径，
    #    通信层 JS 经 launch_gateway 的 symlink 合并进 UI 目录）
    if not html_root and all_js_roots:
        app.mount("/js", MultiStaticFiles(directories=all_js_roots), name="js")

    # 4) 纯传输模式入口：/ → 诊断台（操作员只记一个地址；组合模式 / 归 UI 层）
    if not html_root:
        @app.get("/", include_in_schema=False)
        async def _root_redirect():
            return RedirectResponse(url="/console.html", status_code=307)

    # 5) HTML 静态
    if html_root and os.path.isdir(html_root):
        # 组合模式：UI 层独占 /；本包诊断页挂 /rwl（console.html 仍可从 UI 域访问）。
        # 用 SymlinkStaticFiles：install 树符号链接形态下原生 StaticFiles 全 404
        if own_public:
            app.mount("/rwl", SymlinkStaticFiles(directory=own_public, html=True), name="rwl")
        app.mount("/", SymlinkStaticFiles(directory=html_root, html=True), name="html")
    elif own_public:
        # 纯传输模式：本包 web/public 作 HTML 根（/console.html 诊断台，等价
        # rosbridge_suite 自带 roslibjs 示例页的定位）
        app.mount("/", StaticFiles(directory=own_public, html=True), name="html")

    return app
