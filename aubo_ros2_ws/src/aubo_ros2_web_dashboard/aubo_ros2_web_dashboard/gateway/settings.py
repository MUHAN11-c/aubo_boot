"""兼容重导出 — 所有配置访问请直接用 aubo_ros2_web_dashboard.config。"""
from aubo_ros2_web_dashboard.config import (  # noqa: F401
    rosbridge_host,
    rosbridge_port,
    web_video_host,
    web_video_port,
    runtime_config_dict,
)
