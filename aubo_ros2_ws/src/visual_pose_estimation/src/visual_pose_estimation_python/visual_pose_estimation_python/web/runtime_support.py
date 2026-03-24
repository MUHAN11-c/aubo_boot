from __future__ import annotations

import base64
import io
import math
import os
from functools import lru_cache
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image

from .resources import WebPaths, get_app_config, load_json_file, resolve_templates_root, resolve_web_paths


CAMERA_POSE_FIXED_ORIENTATION = {
    "orientation": {"x": -0.7071067811865476, "y": -0.7071067811865476, "z": 0.0, "w": 0.0},
    "euler_orientation_rpy_rad": [-3.141592653589793, 0.0, 1.5707963267948966],
    "euler_orientation_rpy_deg": [-180.0, 0.0, 90.0],
}

REMBG_AVAILABLE = False
_RemBGProcessorClass = None
try:
    import onnxruntime  # noqa: F401
    from visual_pose_estimation_python.rembg_processor import RemBGProcessor as _ImportedRemBGProcessorClass

    _RemBGProcessorClass = _ImportedRemBGProcessorClass
    REMBG_AVAILABLE = True
except ImportError:
    _RemBGProcessorClass = None
    REMBG_AVAILABLE = False

SUBPROCESS_REMBG_AVAILABLE = False
if not REMBG_AVAILABLE:
    _conda_python = Path(os.environ.get("VPE_REMBG_PYTHON", str(Path.home() / "miniconda3" / "envs" / "ros2_env" / "bin" / "python"))).expanduser()
    _script_path = resolve_web_paths().rembg_subprocess_script
    if _conda_python.exists() and _script_path.exists():
        try:
            import subprocess

            test_result = subprocess.run(
                [str(_conda_python), "-c", "import rembg; import onnxruntime; print('OK')"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if test_result.returncode == 0:
                SUBPROCESS_REMBG_AVAILABLE = True
                REMBG_AVAILABLE = True
        except Exception:
            SUBPROCESS_REMBG_AVAILABLE = False


class SubprocessRemBGProcessor:
    """Use a conda environment to run rembg when current Python cannot import it."""

    def __init__(self, script_path: Path, model: str = "u2net", prefer_cuda: bool = True):
        self.model = model
        self.prefer_cuda = prefer_cuda
        self._conda_python = str(
            Path(
                os.environ.get(
                    "VPE_REMBG_PYTHON",
                    str(Path.home() / "miniconda3" / "envs" / "ros2_env" / "bin" / "python"),
                )
            ).expanduser()
        )
        self._script_path = script_path
        self._providers = None

    @property
    def providers(self):
        return self._providers

    def process_roi(self, color_bgr: np.ndarray, bbox: tuple[int, int, int, int]):
        import subprocess
        import tempfile

        x0, y0, x1, y1 = bbox
        input_path = ""
        bbox_path = ""
        try:
            img_pil = Image.fromarray(color_bgr[:, :, ::-1])
            with tempfile.NamedTemporaryFile(mode="wb", suffix=".png", delete=False) as input_file:
                img_pil.save(input_file, format="PNG")
                input_path = input_file.name

            bbox_data = {"x0": x0, "y0": y0, "x1": x1, "y1": y1}
            with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="utf-8") as bbox_file:
                json.dump(bbox_data, bbox_file)
                bbox_path = bbox_file.name

            result = subprocess.run(
                [self._conda_python, str(self._script_path), input_path, bbox_path, self.model],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0:
                return None, None

            output = json.loads(result.stdout)
            if not output.get("success"):
                return None, None

            self._providers = output.get("providers", ["CPUExecutionProvider"])

            mask_data = base64.b64decode(output["mask_b64"])
            mask_img = Image.open(io.BytesIO(mask_data))
            mask = np.array(mask_img.convert("L"))

            cutout_data = base64.b64decode(output["cutout_b64"])
            cutout_img = Image.open(io.BytesIO(cutout_data))
            cutout_rgb = np.array(cutout_img.convert("RGB"))
            cutout_bgr = cutout_rgb[:, :, ::-1]
            return mask, cutout_bgr
        except Exception:
            return None, None
        finally:
            for temp_path in (input_path, bbox_path):
                if temp_path and os.path.exists(temp_path):
                    try:
                        os.unlink(temp_path)
                    except Exception:
                        pass


_REMBG_PROCESSOR = None


def get_rembg_processor():
    global _REMBG_PROCESSOR
    if _REMBG_PROCESSOR is not None:
        return _REMBG_PROCESSOR

    if _RemBGProcessorClass is not None:
        _REMBG_PROCESSOR = _RemBGProcessorClass(prefer_cuda=True)
    elif SUBPROCESS_REMBG_AVAILABLE:
        script_path = resolve_web_paths().rembg_subprocess_script
        _REMBG_PROCESSOR = SubprocessRemBGProcessor(script_path=script_path, prefer_cuda=True)
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
def load_app_config(config_path: Path) -> dict:
    return load_json_file(config_path)


def get_paths_app_config(paths: WebPaths) -> dict:
    return get_app_config(paths)


def get_camera_pose_fixed_orientation(paths: WebPaths) -> dict:
    config = get_paths_app_config(paths)
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
