"""
路径解析模块

统一解析模板目录、标定文件、相机内参、debug阈值文件等路径。
优先使用源码头路径，回退到 ament_index 安装路径。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


@dataclass
class WebPaths:
    """Web 相关路径集合"""
    configs_dir: Path
    debug_thresholds_file: Path
    rembg_subprocess_script: Path


def _find_package_src_root() -> Path:
    """定位包源码根目录（visual_pose_estimation_python/）。"""
    this_file = Path(__file__).resolve()
    pkg_dir = this_file.parent
    for _ in range(10):
        if (pkg_dir / "package.xml").exists():
            return pkg_dir
        pkg_dir = pkg_dir.parent
    return Path(os.getcwd())


def _find_web_ui_dir() -> Path:
    """定位 web_ui/ 目录。先查源码树，再查 ament 安装路径。"""
    pkg_src = _find_package_src_root()
    web_ui = pkg_src / "web_ui"
    if web_ui.exists():
        return web_ui
    try:
        from ament_index_python import get_package_share_directory
        share = Path(get_package_share_directory("visual_pose_estimation_python"))
        installed = share / "web_ui"
        if installed.exists():
            return installed
    except Exception:
        pass
    return web_ui


def _find_templates_root() -> Path:
    """定位模板根目录。"""
    # 1. 环境变量
    env_val = os.environ.get("VPE_TEMPLATE_ROOT")
    if env_val:
        return Path(env_val)
    # 2. 源码树中的 visual_pose_estimation/templates
    pkg_src = _find_package_src_root()
    templates = pkg_src.parent / "templates"
    if templates.exists():
        return templates
    # 3. 工作空间中的 templates
    ws_templates = pkg_src.parent.parent / "templates"
    if ws_templates.exists():
        return ws_templates
    return templates


def _find_repo_root() -> Path:
    """查找仓库根目录（aubo_ros2_ws）。"""
    pkg_src = _find_package_src_root()
    for parent in pkg_src.parents:
        if (parent / "src").exists() and (parent / "install").exists():
            return parent
    return pkg_src.parents[1]


def resolve_web_paths() -> WebPaths:
    """返回 Web 资源路径集合。

    Returns:
        WebPaths 实例，包含 configs_dir, debug_thresholds_file, rembg_subprocess_script
    """
    web_ui = _find_web_ui_dir()
    configs_dir = web_ui / "configs"

    # debug_thresholds.json
    debug_thresholds = configs_dir / "debug_thresholds.json"
    if not debug_thresholds.exists():
        # 回退到 web_ui 根目录
        debug_thresholds = web_ui / "debug_thresholds.json"

    # rembg 子进程脚本
    rembg_script = web_ui / "scripts" / "rembg_subprocess.py"
    if not rembg_script.exists():
        rembg_script = Path("scripts/rembg_subprocess.py")

    return WebPaths(
        configs_dir=configs_dir,
        debug_thresholds_file=debug_thresholds,
        rembg_subprocess_script=rembg_script,
    )


def resolve_templates_root(web_paths: Optional[WebPaths] = None) -> Path:
    """解析模板根目录路径。

    优先级: 环境变量 VPE_TEMPLATE_ROOT > 源码树 > 工作空间默认路径

    Args:
        web_paths: WebPaths 实例（未使用，保留接口兼容）

    Returns:
        模板根目录的 Path
    """
    return _find_templates_root()


def resolve_camera_intrinsics_candidates(web_paths: WebPaths) -> List[Path]:
    """返回相机内参保选文件路径列表。

    优先级: camera_intrinsics.yaml > ost.yaml

    Args:
        web_paths: WebPaths 实例

    Returns:
        Path 候选列表
    """
    candidates = [
        web_paths.configs_dir / "camera_intrinsics.yaml",
        web_paths.configs_dir / "ost.yaml",
    ]
    # 也检查 hand_eye_calibration 包的 share 路径
    try:
        from ament_index_python import get_package_share_directory
        share = Path(get_package_share_directory("hand_eye_calibration"))
        candidates.append(share / "config" / "calibration_results" / "ost.yaml")
    except Exception:
        pass
    return candidates


def resolve_hand_eye_calibration_candidates(web_paths: WebPaths) -> List[Path]:
    """返回手眼标定候选文件路径列表。

    优先级: hand_eye_calibration.yaml > 通配符匹配 > hand_eye_calibration 包结果

    Args:
        web_paths: WebPaths 实例

    Returns:
        Path 候选列表
    """
    candidates = [
        web_paths.configs_dir / "hand_eye_calibration.yaml",
    ]
    # 查找 hand_eye_calibration*.yaml 的所有文件
    if web_paths.configs_dir.exists():
        for f in sorted(web_paths.configs_dir.glob("hand_eye_calibration*.yaml")):
            p = web_paths.configs_dir / f.name
            if p not in candidates:
                candidates.append(p)
    # 也从 hand_eye_calibration 包查找
    try:
        from ament_index_python import get_package_share_directory
        share = Path(get_package_share_directory("hand_eye_calibration"))
        calib_dir = share / "config" / "calibration_results"
        if calib_dir.exists():
            for f in sorted(calib_dir.glob("hand_eye_calibration*.yaml")):
                if f not in candidates:
                    candidates.append(f)
    except Exception:
        pass
    return candidates
