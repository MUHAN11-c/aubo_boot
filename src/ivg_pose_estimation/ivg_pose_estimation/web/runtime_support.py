"""Web 层通用工具：RemBG 可用性、姿态/图像辅助函数等."""

from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path

from ..path_resolver import WebPaths, get_app_config, resolve_templates_root


CAMERA_POSE_FIXED_ORIENTATION = {
    "orientation": {"x": -0.7071067811865476, "y": -0.7071067811865476, "z": 0.0, "w": 0.0},
    "euler_orientation_rpy_rad": [-3.141592653589793, 0.0, 1.5707963267948966],
    "euler_orientation_rpy_deg": [-180.0, 0.0, 90.0],
}

# RemBG 进程内可用性（venv 已装 rembg/onnxruntime 时为 True）
try:
    import onnxruntime  # noqa: F401
    from ..rembg_processor import RemBGProcessor

    REMBG_AVAILABLE = True
except ImportError:
    RemBGProcessor = None
    REMBG_AVAILABLE = False

_REMBG_PROCESSOR = None


def get_rembg_processor():
    """按需构造进程内 RemBG 处理器；不可用时返回 None（调用方跳过抠图）."""
    global _REMBG_PROCESSOR
    if _REMBG_PROCESSOR is None and REMBG_AVAILABLE:
        _REMBG_PROCESSOR = RemBGProcessor(prefer_cuda=True)
    return _REMBG_PROCESSOR


def quaternion_to_euler_rpy(x: float, y: float, z: float, w: float) -> list[float]:
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


@lru_cache(maxsize=8)
def _app_config_cached(config_path: Path) -> dict:
    return get_app_config(config_path)


def get_camera_pose_fixed_orientation(paths: WebPaths) -> dict:
    config = get_app_config(paths)
    return config.get("camera_pose_fixed_orientation") or CAMERA_POSE_FIXED_ORIENTATION


def get_templates_dir(paths: WebPaths) -> Path:
    return resolve_templates_root(paths)


def get_pose_list_dir(paths: WebPaths) -> Path:
    return paths.pose_list_dir


def normalize_pose_rotation(robot_status):
    if not robot_status or "cartesian_position" not in robot_status:
        return robot_status

    cartesian_position = robot_status["cartesian_position"]
    position = cartesian_position.get("position", {})
    orientation = cartesian_position.get("orientation", {})
    ox = float(orientation.get("x", 0.0))
    oy = float(orientation.get("y", 0.0))
    oz = float(orientation.get("z", 0.0))
    ow = float(orientation.get("w", 1.0))

    euler_rad = cartesian_position.get("euler_orientation_rpy_rad")
    euler_deg = cartesian_position.get("euler_orientation_rpy_deg")
    if not isinstance(euler_rad, (list, tuple)) or len(euler_rad) != 3:
        euler_rad = quaternion_to_euler_rpy(ox, oy, oz, ow)
    else:
        euler_rad = [float(value) for value in euler_rad]

    if not isinstance(euler_deg, (list, tuple)) or len(euler_deg) != 3:
        euler_deg = [math.degrees(value) for value in euler_rad]
    else:
        euler_deg = [float(value) for value in euler_deg]

    robot_status["cartesian_position"] = {
        "position": position,
        "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
        "euler_orientation_rpy_rad": euler_rad,
        "euler_orientation_rpy_deg": euler_deg,
    }
    return robot_status
