from __future__ import annotations

import json
import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Optional


PACKAGE_NAME = "visual_pose_estimation_python"
HAND_EYE_PACKAGE_NAME = "hand_eye_calibration"


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


def _resolve_workspace_templates_dir(source_root: Path, package_share_dir: Optional[Path]) -> Path:
    source_candidate = source_root.parents[1] / "templates"
    if source_candidate.exists():
        return source_candidate

    if package_share_dir is not None:
        install_workspace_candidate = package_share_dir.parents[3] / "src" / "visual_pose_estimation" / "templates"
        if install_workspace_candidate.exists():
            return install_workspace_candidate

    return source_candidate


@lru_cache(maxsize=16)
def load_json_file(path: Path) -> dict:
    if not path.exists():
        return {}

    try:
        with open(path, "r", encoding="utf-8") as file_obj:
            return json.load(file_obj)
    except Exception:
        return {}


@dataclass(frozen=True)
class WebPaths:
    source_root: Path
    package_share_dir: Optional[Path]
    hand_eye_package_share_dir: Optional[Path]
    legacy_ui_dir: Path
    static_dir: Path
    configs_dir: Path
    docs_dir: Path
    legacy_scripts_dir: Path
    index_file: Path
    workspace_templates_dir: Path
    rembg_subprocess_script: Path

    @property
    def app_config_file(self) -> Path:
        return self.configs_dir / "app_config.json"

    @property
    def debug_thresholds_file(self) -> Path:
        return self.configs_dir / "debug_thresholds.json"

    @property
    def pose_list_dir(self) -> Path:
        return self.configs_dir / "pose_list"


def resolve_web_paths() -> WebPaths:
    source_root = Path(__file__).resolve().parents[2]
    package_share_dir = _get_package_share_directory(PACKAGE_NAME)
    hand_eye_package_share_dir = _get_package_share_directory(HAND_EYE_PACKAGE_NAME)

    source_web_ui_dir = source_root / "web_ui"
    share_web_ui_dir = package_share_dir / "web_ui" if package_share_dir else None

    legacy_ui_dir = share_web_ui_dir if share_web_ui_dir and share_web_ui_dir.exists() else source_web_ui_dir
    static_dir = legacy_ui_dir / "static"
    configs_dir = legacy_ui_dir / "configs"
    docs_dir = legacy_ui_dir / "docs"
    legacy_scripts_dir = legacy_ui_dir / "scripts"

    if static_dir.exists():
        index_file = static_dir / "index.html"
    else:
        index_file = legacy_ui_dir / "index.html"

    workspace_templates_dir = _resolve_workspace_templates_dir(source_root, package_share_dir)
    rembg_subprocess_script = _first_existing_path(
        legacy_scripts_dir / "rembg_subprocess.py",
        source_web_ui_dir / "scripts" / "rembg_subprocess.py",
    ) or legacy_scripts_dir / "rembg_subprocess.py"

    return WebPaths(
        source_root=source_root,
        package_share_dir=package_share_dir,
        hand_eye_package_share_dir=hand_eye_package_share_dir,
        legacy_ui_dir=legacy_ui_dir,
        static_dir=static_dir,
        configs_dir=configs_dir,
        docs_dir=docs_dir,
        legacy_scripts_dir=legacy_scripts_dir,
        index_file=index_file,
        workspace_templates_dir=workspace_templates_dir,
        rembg_subprocess_script=rembg_subprocess_script,
    )


def get_app_config(paths: WebPaths) -> dict:
    return load_json_file(paths.app_config_file)


def resolve_templates_root(paths: WebPaths, explicit_root: str | Path | None = None) -> Path:
    candidates: list[Path] = []
    if explicit_root:
        candidates.append(Path(explicit_root).expanduser())

    env_template_root = os.environ.get("VPE_TEMPLATE_ROOT", "").strip()
    if env_template_root:
        candidates.append(Path(env_template_root).expanduser())

    app_config_root = str(get_app_config(paths).get("template_root", "")).strip()
    if app_config_root:
        candidates.append(Path(app_config_root).expanduser())

    candidates.append(paths.workspace_templates_dir)

    existing = _first_existing_path(*candidates)
    return existing if existing is not None else candidates[-1]


def resolve_camera_intrinsics_candidates(paths: WebPaths) -> list[Path]:
    candidates = [
        paths.configs_dir / "camera_intrinsics.yaml",
        paths.configs_dir / "ost.yaml",
    ]

    if paths.hand_eye_package_share_dir:
        candidates.append(paths.hand_eye_package_share_dir / "config" / "calibrationdata" / "ost.yaml")
    return candidates


def resolve_hand_eye_calibration_candidates(paths: WebPaths) -> list[Path]:
    candidates: list[Path] = []
    standard_file = paths.configs_dir / "hand_eye_calibration.yaml"
    if standard_file.exists():
        candidates.append(standard_file)

    if paths.configs_dir.exists():
        generated_files = [path for path in paths.configs_dir.glob("hand_eye_calibration*.yaml") if path != standard_file]
        if not generated_files:
            generated_files = [
                path
                for path in paths.configs_dir.glob("*.yaml")
                if path.name not in {"ost.yaml", "camera_intrinsics.yaml"}
            ]
        candidates.extend(sorted(generated_files, key=lambda path: path.stat().st_mtime, reverse=True))

    if paths.hand_eye_package_share_dir:
        calibration_results_dir = paths.hand_eye_package_share_dir / "config" / "calibration_results"
        if calibration_results_dir.exists():
            candidates.extend(
                sorted(calibration_results_dir.glob("*.yaml"), key=lambda path: path.stat().st_mtime, reverse=True)
            )

    return candidates
