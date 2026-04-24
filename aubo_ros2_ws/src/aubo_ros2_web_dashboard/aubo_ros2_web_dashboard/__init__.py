"""
aubo_ros2_web_dashboard — IVG Web 仪表板 ament_python 包根命名空间。

目录职责概要：
- ``launch/``：一键拉起 rosbridge、tf2_web_republisher、可选 web_video_server、FastAPI 静态网关。
- ``web/public/``：浏览器静态资源（HTML/CSS/JS），安装到 ``share/<pkg>/web/public``。
- ``aubo_ros2_web_dashboard.gateway``：FastAPI 应用（静态托管 + BFF + rosbridge/web_video 同源代理）。
- ``fastapi_static_gateway`` 模块：Uvicorn 可执行入口（``python -m`` / 控制台脚本）。

子模块请从 ``aubo_ros2_web_dashboard.gateway`` 或 ``fastapi_static_gateway`` 导入；本文件仅作包标识。
"""
