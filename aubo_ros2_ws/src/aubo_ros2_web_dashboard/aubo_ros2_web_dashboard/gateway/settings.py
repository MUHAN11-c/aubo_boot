"""
环境变量名、默认值，以及 ``GET /api/ivg/runtime-config`` 的 JSON 构造。

说明：
  - 具体主机/端口由**进程环境**提供（launch ``additional_env`` 或 shell ``export``）；
  - 本模块**不写入**环境，只 ``os.environ.get`` 读取；
  - 浏览器始终经网关同源 ``/ws/rosbridge`` 与 ``/api/ivg/proxy/web-video`` 访问上游（无关闭开关）。
"""
from __future__ import annotations

import os
from typing import Any

_PKG_FALLBACK = "0.4.0"

# 无环境变量时的默认上游（本机 rosbridge / web_video）
DEFAULT_ROSBRIDGE_PORT = 9090
DEFAULT_WEB_VIDEO_PORT = 8089
DEFAULT_ROSBRIDGE_HOST = "127.0.0.1"
DEFAULT_WEB_VIDEO_HOST = "127.0.0.1"

# 与 launch/web_dashboard.launch.py 中 additional_env 的键一致
ENV_ROSBRIDGE = "IVG_ROSBRIDGE_PORT"
ENV_WEB_VIDEO = "IVG_WEB_VIDEO_PORT"
ENV_ROSBRIDGE_HOST = "IVG_ROSBRIDGE_HOST"
ENV_WEB_VIDEO_HOST = "IVG_WEB_VIDEO_HOST"


def package_version() -> str:
	try:
		from importlib.metadata import version

		return version("aubo_ros2_web_dashboard")
	except Exception:
		return _PKG_FALLBACK


def runtime_config_dict(static_root: str) -> dict[str, Any]:
	# 供前端 ivg_runtime.js：上游端口（展示/覆盖）、同源代理路径
	rb_port = int(os.environ.get(ENV_ROSBRIDGE, str(DEFAULT_ROSBRIDGE_PORT)))
	wv_port = int(os.environ.get(ENV_WEB_VIDEO, str(DEFAULT_WEB_VIDEO_PORT)))
	return {
		"package": "aubo_ros2_web_dashboard",
		"version": package_version(),
		"rosbridge_port": rb_port,
		"web_video_port": wv_port,
		"static_root": static_root,
		"unified_proxy": True,
		"rosbridge_ws_path": "/ws/rosbridge",
		"web_video_proxy_prefix": "/api/ivg/proxy/web-video",
		"hint": "浏览器只连当前页面端口；rosbridge→/ws/rosbridge，视频→/api/ivg/proxy/web-video/…；上游在网关本机 IVG_*_HOST",
	}
