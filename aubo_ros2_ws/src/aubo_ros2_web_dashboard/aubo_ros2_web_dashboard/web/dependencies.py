"""
FastAPI 依赖注入：从环境变量读取 RWT/网关运行时配置（不依赖 rclpy）。

launch 文件在启动 web_dashboard 进程前会写入 AUBO_WEB_*，供此处与 /api/config 使用。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from functools import lru_cache


@dataclass(frozen=True)
class WebRuntimeConfig:
    """浏览器与运维可见的运行时参数（由环境变量推导）。"""

    rosbridge_port: int
    public_host: str | None
    use_sim_time: bool
    default_fixed_frame: str


@lru_cache
def get_runtime_config() -> WebRuntimeConfig:
    """进程级缓存；单测中需 cache_clear() 以免环境变量串单测。"""
    port_s = os.environ.get("AUBO_WEB_ROSBRIDGE_PORT", "9090")
    try:
        port = int(port_s)
    except ValueError:
        port = 9090
    public = os.environ.get("AUBO_WEB_PUBLIC_HOST", "").strip() or None
    sim = os.environ.get("AUBO_WEB_USE_SIM_TIME", "").lower() in ("1", "true", "yes")
    fixed = os.environ.get("AUBO_WEB_FIXED_FRAME", "base_link")
    return WebRuntimeConfig(
        rosbridge_port=port,
        public_host=public,
        use_sim_time=sim,
        default_fixed_frame=fixed,
    )
