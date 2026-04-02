"""
解析本包安装后的 share 路径，定位 RWT 静态资源目录（web/dist）。

colcon install 后路径形如：install/.../share/aubo_ros2_web_dashboard/web/dist/
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class WebPaths:
    """RWT 前端构建产物根目录（内含 index.html、demo.html、dev.html 等）。"""

    dist_dir: Path


def resolve_web_paths() -> WebPaths:
    share = Path(get_package_share_directory("aubo_ros2_web_dashboard"))
    dist = share / "web" / "dist"
    return WebPaths(dist_dir=dist)
