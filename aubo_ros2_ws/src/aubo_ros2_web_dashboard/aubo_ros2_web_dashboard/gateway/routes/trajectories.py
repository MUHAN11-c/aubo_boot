"""轨迹数据 API — 列出/加载已录制的轨迹 NPZ 文件。

端点:
  GET /api/v1/trajectories           → 列出所有录制
  GET /api/v1/trajectories/{ts}/full  → 完整轨迹 JSON
  GET /api/v1/trajectories/{ts}/latte/{n} → 拉花段 JSON
"""
from __future__ import annotations

import os
import re
from pathlib import Path

import numpy as np
from fastapi import APIRouter, HTTPException

router = APIRouter(prefix="/api/v1/trajectories", tags=["trajectories"])

_TRAJECTORY_ROOT = Path.home() / "robot_trajectories"
_TS_RE = re.compile(r"^\d{8}_\d{6}$")  # YYYYMMDD_HHMMSS


def _load_npz(path: Path) -> dict:
    """加载 NPZ 文件, 返回 {key: array_list}。"""
    if not path.is_file():
        raise HTTPException(status_code=404, detail=f"文件不存在: {path.name}")
    # allow_pickle=False: 纯数值数组无需 pickle, 且防止恶意 NPZ 反序列化攻击
    # Ref: https://numpy.org/doc/stable/reference/generated/numpy.load.html
    data = dict(np.load(path, allow_pickle=False))
    return data


def _arrays_to_json(arr: dict, max_points: int = 5000) -> dict:
    """将 numpy 数组 dict 转为 JSON 兼容格式。超过 max_points 时降采样。"""
    n = len(next(iter(arr.values())))
    # 校验所有数组长度一致, 防止损坏的 NPZ 导致前端数组越界
    for key, values in arr.items():
        if len(values) != n:
            raise HTTPException(
                status_code=500,
                detail=f"数据损坏: '{key}' 长度 ({len(values)}) 与预期 ({n}) 不一致"
            )
    step = max(1, n // max_points)
    result = {}
    for key, values in arr.items():
        a = values[::step]
        # 四舍五入到合理精度 (位置 5 位小数, 角度 3 位)
        if key in ("x", "y", "z"):
            a = np.round(a, 5)
        elif key in ("roll", "pitch", "yaw"):
            a = np.round(a, 3)
        elif key.startswith("j"):
            a = np.round(a, 2)
        result[key] = a.tolist()
    result["_original_n"] = n
    result["_downsampled"] = n > max_points
    return result


def _ts_dir(ts: str) -> Path:
    """校验时间戳并返回对应目录路径。"""
    if not _TS_RE.match(ts):
        raise HTTPException(status_code=400, detail=f"无效时间戳格式: {ts}")
    d = _TRAJECTORY_ROOT / ts
    if not d.is_dir():
        raise HTTPException(status_code=404, detail=f"录制目录不存在: {ts}")
    return d


@router.get("")
def list_trajectories():
    """列出所有录制的轨迹, 按时间倒序。"""
    if not _TRAJECTORY_ROOT.is_dir():
        return {"trajectories": []}

    items = []
    for d in sorted(_TRAJECTORY_ROOT.iterdir(), reverse=True):
        if not d.is_dir() or not _TS_RE.match(d.name):
            continue
        full_npz = d / "trajectory_full.npz"
        if not full_npz.is_file():
            continue

        try:
            data = dict(np.load(full_npz, allow_pickle=False))
            t = data.get("t")
            if t is None or len(t) == 0:
                continue
            dur = float(t[-1] - t[0])
            n_points = len(t)

            # 计数拉花段
            latte_count = 0
            latte_n = 0
            i = 1
            while (d / f"trajectory_latte_{i}.npz").is_file():
                latte_count += 1
                try:
                    ld = dict(np.load(d / f"trajectory_latte_{i}.npz", allow_pickle=False))
                    latte_n += len(ld.get("t", []))
                except Exception:
                    pass
                i += 1

            items.append({
                "ts": d.name,
                "n_points": n_points,
                "duration": round(dur, 1),
                "frequency": round(n_points / dur, 1) if dur > 0 else 0,
                "latte_segments": latte_count,
                "latte_points": latte_n,
                "x_range": [round(float(data["x"].min()), 4), round(float(data["x"].max()), 4)],
                "y_range": [round(float(data["y"].min()), 4), round(float(data["y"].max()), 4)],
                "z_range": [round(float(data["z"].min()), 4), round(float(data["z"].max()), 4)],
            })
        except Exception:
            continue

    return {"trajectories": items}


@router.get("/{ts}/full")
def get_full_trajectory(ts: str):
    """获取完整轨迹数据 (JSON)。"""
    d = _ts_dir(ts)
    arr = _load_npz(d / "trajectory_full.npz")
    if len(arr) == 0:
        raise HTTPException(status_code=404, detail="轨迹数据为空")
    return _arrays_to_json(arr)


@router.get("/{ts}/latte/{n}")
def get_latte_trajectory(ts: str, n: int):
    """获取拉花段轨迹数据 (JSON)。n 从 1 开始。"""
    d = _ts_dir(ts)
    path = d / f"trajectory_latte_{n}.npz"
    arr = _load_npz(path)
    if len(arr) == 0:
        raise HTTPException(status_code=404, detail="拉花段数据为空")
    return _arrays_to_json(arr)
