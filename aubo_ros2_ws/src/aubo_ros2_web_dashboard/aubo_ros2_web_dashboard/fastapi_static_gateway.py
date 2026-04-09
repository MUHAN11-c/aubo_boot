"""
FastAPI 统一静态网关：托管 ``web/public``，并作为 **HTTP 边界**（静态 + 轻量 BFF）。

实现按常见 FastAPI 项目习惯拆在 ``aubo_ros2_web_dashboard.gateway``：

- ``gateway.settings``：环境变量、版本、``runtime-config`` 载荷；
- ``gateway.routes``：``APIRouter``（``/health``、``/api/ivg/runtime-config``、``/ws/rosbridge``、``/api/ivg/proxy/web-video/…``）；
- ``gateway.app``：``create_app(web_root)``（``include_router``、中间件、``StaticFiles``）；
- ``gateway.cli``：``main()`` → Uvicorn。

环境变量（由 ``web_dashboard.launch.py`` 的 ``ExecuteProcess.additional_env`` 传入 ``IVG_ROSBRIDGE_HOST``/``PORT`` 与 ``IVG_WEB_VIDEO_HOST``/``PORT``，亦可手工 export）：

- ``IVG_ROSBRIDGE_HOST`` / ``IVG_ROSBRIDGE_PORT``（默认 ``127.0.0.1`` / ``9090``）
- ``IVG_WEB_VIDEO_HOST`` / ``IVG_WEB_VIDEO_PORT``（默认 ``127.0.0.1`` / ``8089``）

浏览器始终经同源 ``/ws/rosbridge`` 与 ``/api/ivg/proxy/web-video`` 访问上述上游（无环境变量开关）。

运行：``python3 -m aubo_ros2_web_dashboard.fastapi_static_gateway [PORT] --bind HOST --directory DIR``（``PORT`` 缺省为 8090；``--directory`` **必填**，指向已安装的 ``share/.../web/public`` 或开发时包内同名目录的绝对路径。）
"""
from __future__ import annotations

from aubo_ros2_web_dashboard.gateway.app import create_app
from aubo_ros2_web_dashboard.gateway.cli import main

__all__ = ["create_app", "main"]


if __name__ == "__main__":
	main()
