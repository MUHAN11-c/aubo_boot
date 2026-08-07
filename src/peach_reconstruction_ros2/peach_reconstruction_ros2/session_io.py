"""
重建 session 落盘 — 每帧原始数据 + 位姿 + 参数快照.

目录结构（root_dir/session_YYYYMMDD_HHMMSS/）：
  frame_XX_rgb.png            BGR 彩图（cv_bridge bgr8 即 BGR，cv2 直写无色差）
  frame_XX_depth.npy          uint16 深度 [mm]
  frame_XX_camera_info.yaml   内参 K + 时间戳
  frame_XX_T_base_camera.yaml base←camera 4×4 位姿与诊断标记
  result/tsdf_cloud.ply        finalize 的 TSDF 云（xyz+rgb，可选）
  metadata.yaml                参数快照与帧级摘要（离线复现用）
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import List

import cv2
import numpy as np
import yaml


def _dump_yaml(data: dict, path) -> None:
    """
    把 dict 写入 yaml 文件（utf-8，不排序保持可读顺序）.

    Args:
        data: 可 yaml 序列化的 dict（numpy 类型须已转原生类型）.
        path: 输出路径.

    Returns
    -------
        无返回值（None）；文件写入 path.

    """
    with open(str(path), 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def _write_ply_xyzrgb(path, xyz: np.ndarray,
                      colors_bgr: np.ndarray) -> None:
    """
    写 ASCII PLY（xyz + uchar red/green/blue；注意文件内转回 RGB 序）.

    Args:
        path: 输出 ply 路径.
        xyz: (N, 3) 点 [m].
        colors_bgr: (N, 3) uint8 BGR（OpenCV 排列）.

    Returns
    -------
        无返回值（None）；文件写入 path.

    """
    n = int(xyz.shape[0])
    rgb = np.asarray(colors_bgr, dtype=np.uint8)[:, ::-1]  # BGR → RGB
    rows = np.column_stack([xyz, rgb])
    with open(str(path), 'w', encoding='utf-8') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {n}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\n'
                'property uchar blue\n')
        f.write('end_header\n')
        np.savetxt(f, rows, fmt='%.6f %.6f %.6f %d %d %d')


def save_session(root_dir, frames: List, metadata: dict,
                 tsdf_cloud=None) -> Path:
    """
    把一次重建的全部帧写到 root_dir/session_<时间戳>/.

    Args:
        root_dir: session 根目录（不存在自动创建）.
        frames: CapturedFrame 列表（按采集顺序编号 frame_00, frame_01, ...）.
        metadata: 参数快照等元信息（写入 metadata.yaml）.
        tsdf_cloud: 可选 (xyz, colors_bgr) 元组；给出时写
            result/tsdf_cloud.ply（xyz+rgb）.

    Returns
    -------
        创建成功的 session 目录 Path.

    """
    root = Path(root_dir)
    session_dir = root / f'session_{datetime.now():%Y%m%d_%H%M%S}'
    session_dir.mkdir(parents=True, exist_ok=True)
    for i, frame in enumerate(frames):
        stem = str(session_dir / f'frame_{i:02d}')
        cv2.imwrite(stem + '_rgb.png', frame.rgb)
        np.save(stem + '_depth.npy', frame.depth_mm)
        _dump_yaml({
            'stamp_sec': float(frame.stamp),
            'width': int(frame.camera_K.get('width', 0)),
            'height': int(frame.camera_K.get('height', 0)),
            'fx': float(frame.camera_K['fx']),
            'fy': float(frame.camera_K['fy']),
            'cx': float(frame.camera_K['cx']),
            'cy': float(frame.camera_K['cy']),
        }, stem + '_camera_info.yaml')
        _dump_yaml({
            'T_base_camera': np.asarray(
                frame.T_base_camera, dtype=np.float64).tolist(),
            'camera_position_base': np.asarray(
                frame.camera_position_base, dtype=np.float64).tolist(),
            'valid_depth_ratio': float(frame.valid_depth_ratio),
            'diagnostic_flags': list(frame.diagnostic_flags),
        }, stem + '_T_base_camera.yaml')
    if tsdf_cloud is not None:
        xyz, colors = tsdf_cloud
        if xyz is not None and len(xyz):
            result_dir = session_dir / 'result'
            result_dir.mkdir(exist_ok=True)
            if colors is None:
                colors = np.zeros((len(xyz), 3), dtype=np.uint8)
            _write_ply_xyzrgb(result_dir / 'tsdf_cloud.ply', xyz, colors)
    _dump_yaml(metadata, session_dir / 'metadata.yaml')
    return session_dir
