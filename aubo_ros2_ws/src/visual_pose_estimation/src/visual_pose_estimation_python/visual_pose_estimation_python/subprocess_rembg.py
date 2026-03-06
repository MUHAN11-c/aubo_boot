#!/usr/bin/env python3
"""
Subprocess-based RemBG processor for ROS2 side.

目的：
- 复用 web_ui 中的 rembg_subprocess.py（运行在 conda GPU 环境）
- 避免在系统 Python 3.8 环境中安装 onnxruntime / rembg
- 对 ROS2 侧暴露与 RemBGProcessor 相同的接口：process_roi(color_bgr, bbox) -> (mask, cutout_bgr)
"""

from __future__ import annotations

import base64
import io
import json
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Optional, Tuple, List

import numpy as np
from PIL import Image

class SubprocessRemBGProcessor:
    """通过 conda 子进程调用 rembg 的处理器。

    接口兼容 RemBGProcessor：
        process_roi(color_bgr, bbox) -> (full_mask, full_cutout_bgr)
    """

    def __init__(self, model: str = "u2net", prefer_cuda: bool = True) -> None:
        self.model = model
        self.prefer_cuda = prefer_cuda
        # 与 web_ui 相同的 conda Python 路径
        self._conda_python = "/home/mu/miniconda3/envs/ros2_env/bin/python"
        # 计算脚本路径（同时兼容源码运行和 install 运行）
        pkg_root = Path(__file__).resolve().parent.parent
        # 优先：源码布局下的 web_ui/scripts/rembg_subprocess.py
        script_candidates = [
            pkg_root / "web_ui" / "scripts" / "rembg_subprocess.py",
            # 兼容当前工作空间的源码绝对路径（防止安装后找不到脚本）
            Path("/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/scripts/rembg_subprocess.py"),
        ]
        chosen = None
        for cand in script_candidates:
            if cand.exists():
                chosen = cand
                break
        self._script_path = chosen if chosen is not None else script_candidates[0]

        self._providers: Optional[List[str]] = None

    @property
    def providers(self) -> Optional[List[str]]:
        return self._providers

    def process_roi(
        self,
        color_bgr: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """通过子进程处理 ROI，返回与 RemBGProcessor 相同格式的结果。"""
        x, y, w, h = bbox

        # 基本合法性检查
        if color_bgr is None or color_bgr.size == 0:
            return None, None

        img_h, img_w = color_bgr.shape[:2]
        if w <= 0 or h <= 0:
            return None, None

        # 在图像范围内裁剪 bbox（与 rembg_subprocess.py 中的逻辑保持一致）
        x0 = max(0, int(x))
        y0 = max(0, int(y))
        x1 = min(img_w, int(x + w))
        y1 = min(img_h, int(y + h))
        if x1 <= x0 or y1 <= y0:
            return None, None

        temp_dir = tempfile.gettempdir()
        input_file = None
        bbox_file = None
        input_path = ""
        bbox_path = ""

        try:
            # 保存整幅 BGR 图像为 PNG（子进程内部再按 bbox 裁剪 ROI）
            img_rgb = color_bgr[:, :, ::-1]
            img_pil = Image.fromarray(img_rgb)
            input_file = tempfile.NamedTemporaryFile(
                mode="wb", suffix=".png", delete=False, dir=temp_dir
            )
            img_pil.save(input_file, format="PNG")
            input_file.close()
            input_path = input_file.name

            # 保存 bbox JSON（注意：这里按 rembg_subprocess 的约定传 x0,y0,w,h）
            bbox_data = {"x0": x0, "y0": y0, "x1": x1 - x0, "y1": y1 - y0}
            bbox_file = tempfile.NamedTemporaryFile(
                mode="w", suffix=".json", delete=False, dir=temp_dir
            )
            json.dump(bbox_data, bbox_file)
            bbox_file.close()
            bbox_path = bbox_file.name

            if not Path(self._conda_python).exists() or not self._script_path.exists():
                return None, None

            # 调用子进程
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

            # 解码 mask
            mask_b64 = output.get("mask_b64")
            cutout_b64 = output.get("cutout_b64")
            if not mask_b64 or not cutout_b64:
                _debug_log(
                    "subprocess_rembg.py:process_roi",
                    "子进程输出缺少 mask 或 cutout",
                    {"has_mask_b64": bool(mask_b64), "has_cutout_b64": bool(cutout_b64)},
                    "SP-R3",
                )
                return None, None

            mask_data = base64.b64decode(mask_b64)
            mask_img = Image.open(io.BytesIO(mask_data))
            mask = np.array(mask_img.convert("L"))

            cutout_data = base64.b64decode(cutout_b64)
            cutout_img = Image.open(io.BytesIO(cutout_data))
            cutout_rgb = np.array(cutout_img.convert("RGB"))
            cutout_bgr = cutout_rgb[:, :, ::-1]

            return mask, cutout_bgr

        except subprocess.TimeoutExpired:
            return None, None
        except Exception:
            return None, None
        finally:
            # 清理临时文件
            try:
                if input_file and input_path and Path(input_path).exists():
                    Path(input_path).unlink()
            except Exception:
                pass
            try:
                if bbox_file and bbox_path and Path(bbox_path).exists():
                    Path(bbox_path).unlink()
            except Exception:
                pass

