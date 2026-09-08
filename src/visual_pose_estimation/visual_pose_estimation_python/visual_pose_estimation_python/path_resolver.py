"""
路径解析模块（唯一事实源：ROS 节点与 Web 层共用）.

统一解析 web_ui 目录、模板目录、标定文件、相机内参等路径。
优先使用源码树路径，回退到 ament_index 安装路径。
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import List, Optional


PACKAGE_NAME = "visual_pose_estimation_python"


@dataclass(frozen=True)
class WebPaths:
    """前端资源、配置、模板、模型等目录的快照（开发源码树 vs ament share）."""

    source_root: Path
    package_share_dir: Optional[Path]
    legacy_ui_dir: Path
    static_dir: Path
    configs_dir: Path
    docs_dir: Path
    legacy_scripts_dir: Path
    index_file: Path
    workspace_templates_dir: Path
    models_dir: Path

    @property
    def app_config_file(self) -> Path:
        return self.configs_dir / "app_config.json"

    @property
    def debug_thresholds_file(self) -> Path:
        return self.configs_dir / "debug_thresholds.json"

    @property
    def pose_list_dir(self) -> Path:
        return self.configs_dir / "pose_list"


def _get_package_share_directory(package_name: str = PACKAGE_NAME) -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:
        return None

    try:
        return Path(get_package_share_directory(package_name))
    except Exception:
        return None


def _first_existing_path(*candidates: Path) -> Optional[Path]:
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def _find_package_src_root() -> Path:
    """定位包源码根目录（visual_pose_estimation_python/，含 package.xml）."""
    this_file = Path(__file__).resolve()
    pkg_dir = this_file.parent
    for _ in range(10):
        if (pkg_dir / "package.xml").exists():
            return pkg_dir
        pkg_dir = pkg_dir.parent
    return Path(os.getcwd())


def _find_repo_root() -> Path:
    """查找本工作区根目录（含 src/ 与 install/ 的 aubo_e5_jazzy_ws）."""
    pkg_src = _find_package_src_root()
    for parent in pkg_src.parents:
        if (parent / "src").exists() and (parent / "install").exists():
            return parent
    return pkg_src.parents[1]


def _resolve_workspace_templates_dir(source_root: Path, package_share_dir: Optional[Path]) -> Path:
    source_candidate = source_root.parent / "templates"
    if source_candidate.exists():
        return source_candidate

    if package_share_dir is not None:
        install_candidate = package_share_dir.parents[3] / "src" / "visual_pose_estimation" / "templates"
        if install_candidate.exists():
            return install_candidate

    return source_candidate


@lru_cache(maxsize=16)
def load_json_file(path: Path) -> dict:
    """读取 JSON 文件；不存在或解析失败返回空字典."""
    if not path.exists():
        return {}

    try:
        with open(path, "r", encoding="utf-8") as file_obj:
            return json.load(file_obj)
    except Exception:
        return {}


def resolve_web_paths() -> WebPaths:
    """根据源码或 ament share 定位 web_ui、模板、模型与配置文件路径."""
    source_root = _find_package_src_root()
    package_share_dir = _get_package_share_directory(PACKAGE_NAME)

    source_web_ui_dir = source_root / "web_ui"
    share_web_ui_dir = package_share_dir / "web_ui" if package_share_dir else None

    legacy_ui_dir = share_web_ui_dir if share_web_ui_dir and share_web_ui_dir.exists() else source_web_ui_dir
    static_dir = legacy_ui_dir / "static"
    configs_dir = legacy_ui_dir / "configs"
    docs_dir = legacy_ui_dir / "docs"
    legacy_scripts_dir = legacy_ui_dir / "scripts"

    # 逻辑入口始终指向主 index（含工作流程、演示）；根路径由 system 路由重定向到 /legacy-ui/
    primary_index = legacy_ui_dir / "index.html"
    if primary_index.exists():
        index_file = primary_index
    elif static_dir.exists():
        index_file = static_dir / "index.html"
    else:
        index_file = primary_index

    workspace_templates_dir = _resolve_workspace_templates_dir(source_root, package_share_dir)

    # 运行时模型（u2net.onnx）：源码树 models/ 优先，ament share models/ 兜底
    source_models_dir = source_root / "models"
    share_models_dir = package_share_dir / "models" if package_share_dir else None
    models_dir = _first_existing_path(source_models_dir, share_models_dir) or source_models_dir

    return WebPaths(
        source_root=source_root,
        package_share_dir=package_share_dir,
        legacy_ui_dir=legacy_ui_dir,
        static_dir=static_dir,
        configs_dir=configs_dir,
        docs_dir=docs_dir,
        legacy_scripts_dir=legacy_scripts_dir,
        index_file=index_file,
        workspace_templates_dir=workspace_templates_dir,
        models_dir=models_dir,
    )


def ensure_u2net_model_home() -> None:
    """
    把 rembg 的模型目录（U2NET_HOME）指到包内 models/，保证运行文件自包含.


    必须在首次 import rembg / new_session 之前调用；已设置的 env 不覆盖。
    """
    os.environ.setdefault("U2NET_HOME", str(resolve_web_paths().models_dir))


def get_app_config(paths: WebPaths) -> dict:
    """读取 web_ui/configs/app_config.json（不存在时为空配置）."""
    return load_json_file(paths.app_config_file)


def resolve_templates_root(web_paths: Optional[WebPaths] = None,
                           explicit_root: str | Path | None = None) -> Path:
    """
    解析模板根目录.


    优先级：显式指定 > 环境变量 VPE_TEMPLATE_ROOT > app_config.json（仅 Web 侧）
    > 源码树 templates/。ROS 节点以无参调用，Web 层传 paths。
    """
    candidates: List[Path] = []
    if explicit_root:
        candidates.append(Path(explicit_root).expanduser())

    env_val = os.environ.get("VPE_TEMPLATE_ROOT", "").strip()
    if env_val:
        candidates.append(Path(env_val).expanduser())

    if web_paths is not None:
        app_config_root = str(get_app_config(web_paths).get("template_root", "")).strip()
        if app_config_root:
            candidates.append(Path(app_config_root).expanduser())
        candidates.append(web_paths.workspace_templates_dir)

    # 无 WebPaths 时（ROS 节点）：源码树/工作区 templates 兜底
    pkg_src = _find_package_src_root()
    candidates.append(pkg_src.parent / "templates")

    existing = _first_existing_path(*candidates)
    return existing if existing is not None else candidates[-1]


def resolve_camera_intrinsics_candidates(web_paths: WebPaths) -> List[Path]:
    """返回相机内参保选文件路径列表（camera_intrinsics.yaml > ost.yaml）."""
    candidates = [
        web_paths.configs_dir / "camera_intrinsics.yaml",
        web_paths.configs_dir / "ost.yaml",
    ]
    # 也检查手眼标定包的 share 路径：本工作区 aubo_hand_eye_calibration 优先，legacy hand_eye_calibration 兜底
    try:
        from ament_index_python import get_package_share_directory
        for pkg_name in ("aubo_hand_eye_calibration", "hand_eye_calibration"):
            try:
                share = Path(get_package_share_directory(pkg_name))
            except Exception:
                continue
            candidate = share / "config" / "calibration_results" / "ost.yaml"
            if candidate not in candidates:
                candidates.append(candidate)
    except Exception:
        pass
    return candidates


def resolve_hand_eye_calibration_candidates(web_paths: WebPaths) -> List[Path]:
    """
    返回手眼标定候选文件路径列表.


    优先级：configs/hand_eye_calibration*.yaml > 工作区 hand_eye/active.yaml
    > 手眼标定包 calibration_results。
    """
    candidates = [
        web_paths.configs_dir / "hand_eye_calibration.yaml",
        _find_repo_root() / "src" / "aubo_hand_eye_calibration" / "hand_eye" / "active.yaml",
    ]
    # 查找 hand_eye_calibration*.yaml 的所有文件
    if web_paths.configs_dir.exists():
        for f in sorted(web_paths.configs_dir.glob("hand_eye_calibration*.yaml")):
            p = web_paths.configs_dir / f.name
            if p not in candidates:
                candidates.append(p)
    # 也从手眼标定包查找：本工作区 aubo_hand_eye_calibration 优先，legacy hand_eye_calibration 兜底
    try:
        from ament_index_python import get_package_share_directory
        for pkg_name in ("aubo_hand_eye_calibration", "hand_eye_calibration"):
            try:
                share = Path(get_package_share_directory(pkg_name))
            except Exception:
                continue
            calib_dir = share / "config" / "calibration_results"
            if calib_dir.exists():
                for f in sorted(calib_dir.glob("hand_eye_calibration*.yaml")):
                    if f not in candidates:
                        candidates.append(f)
            for extra in (
                share / "config" / "hand_eye" / "active.yaml",
                share / "hand_eye" / "active.yaml",
            ):
                if extra not in candidates:
                    candidates.append(extra)
    except Exception:
        pass
    return candidates
