"""
解析 visual_pose_estimation_python 包安装后的 web_ui 路径。

用于挂载 /legacy-ui 与 /static；若包未安装或路径不存在，网关仍启动但无 legacy 能力。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


@dataclass(frozen=True)
class VpeWebPaths:
    """legacy 页面目录与静态资源目录（static 可回退到 web_ui 根）。"""

    legacy_ui_dir: Path
    static_dir: Path


def resolve_vpe_web_paths() -> VpeWebPaths | None:
    try:
        share = Path(get_package_share_directory("visual_pose_estimation_python"))
    except PackageNotFoundError:
        return None
    legacy = share / "web_ui"
    static = legacy / "static"
    if not legacy.is_dir():
        return None
    return VpeWebPaths(legacy_ui_dir=legacy, static_dir=static if static.is_dir() else legacy)
