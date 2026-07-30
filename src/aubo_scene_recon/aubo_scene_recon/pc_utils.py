"""点云读写 / 变换 / voxel — 无 Open3D 依赖，纯 numpy。"""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def cloud_to_arrays(msg: PointCloud2) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """PointCloud2 → (N,3) xyz 与可选 (N,3) rgb∈[0,1]。"""
    field_names = [f.name for f in msg.fields]
    has_rgb = 'rgb' in field_names
    fields = ['x', 'y', 'z', 'rgb'] if has_rgb else ['x', 'y', 'z']
    pts = list(pc2.read_points(msg, field_names=fields, skip_nans=True))
    if not pts:
        return np.zeros((0, 3), dtype=np.float64), None

    xyz = np.empty((len(pts), 3), dtype=np.float64)
    colors = np.empty((len(pts), 3), dtype=np.float64) if has_rgb else None
    for i, p in enumerate(pts):
        xyz[i, 0] = float(p[0])
        xyz[i, 1] = float(p[1])
        xyz[i, 2] = float(p[2])
        if has_rgb:
            colors[i] = _unpack_rgb(p[3])
    return xyz, colors


def _unpack_rgb(color_val) -> np.ndarray:
    """float32 打包的 rgb 字段 → [r,g,b]∈[0,1]。"""
    if isinstance(color_val, float):
        raw = struct.pack('<f', float(color_val))
        color_int = struct.unpack('<I', raw)[0]
    else:
        color_int = int(color_val)
    r = ((color_int >> 16) & 0xFF) / 255.0
    g = ((color_int >> 8) & 0xFF) / 255.0
    b = (color_int & 0xFF) / 255.0
    return np.array([r, g, b], dtype=np.float64)


def _pack_rgb(rgb: np.ndarray) -> np.ndarray:
    """(N,3) [0,1] → float32 位型 rgb（PointCloud2 常用）。"""
    r = np.clip(rgb[:, 0] * 255.0, 0, 255).astype(np.uint32)
    g = np.clip(rgb[:, 1] * 255.0, 0, 255).astype(np.uint32)
    b = np.clip(rgb[:, 2] * 255.0, 0, 255).astype(np.uint32)
    rgb_int = (r << 16) | (g << 8) | b
    return rgb_int.view(np.float32)


def transform_points(xyz: np.ndarray, T: np.ndarray) -> np.ndarray:
    """用 4x4 齐次矩阵变换 (N,3) 点。"""
    if xyz.size == 0:
        return xyz
    R = T[:3, :3]
    t = T[:3, 3]
    return (xyz @ R.T) + t


def filter_by_depth(
    xyz: np.ndarray,
    colors: Optional[np.ndarray],
    min_range: float,
    max_range: float,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """按相机系 z（深度）裁剪。"""
    if xyz.size == 0:
        return xyz, colors
    z = xyz[:, 2]
    mask = (z >= min_range) & (z <= max_range) & np.isfinite(z)
    xyz_f = xyz[mask]
    colors_f = colors[mask] if colors is not None else None
    return xyz_f, colors_f


def voxel_downsample(
    xyz: np.ndarray,
    colors: Optional[np.ndarray],
    voxel_size: float,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """简单体素下采样：每格保留均值点（及均值色）。"""
    if xyz.size == 0 or voxel_size <= 0.0:
        return xyz, colors
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    # 用结构化数组做唯一键
    flat = keys[:, 0] * 73856093 ^ keys[:, 1] * 19349663 ^ keys[:, 2] * 83492791
    order = np.argsort(flat)
    flat_s = flat[order]
    xyz_s = xyz[order]
    colors_s = colors[order] if colors is not None else None

    breaks = np.flatnonzero(np.diff(flat_s)) + 1
    starts = np.concatenate(([0], breaks))
    ends = np.concatenate((breaks, [len(flat_s)]))

    out_xyz = np.empty((len(starts), 3), dtype=np.float64)
    out_colors = np.empty((len(starts), 3), dtype=np.float64) if colors is not None else None
    for i, (s, e) in enumerate(zip(starts, ends)):
        out_xyz[i] = xyz_s[s:e].mean(axis=0)
        if out_colors is not None:
            out_colors[i] = colors_s[s:e].mean(axis=0)
    return out_xyz, out_colors


def make_pointcloud2(
    header: Header,
    xyz: np.ndarray,
    colors: Optional[np.ndarray] = None,
) -> PointCloud2:
    """构造带可选 rgb 的 PointCloud2。"""
    if xyz.size == 0:
        return pc2.create_cloud(header, [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ], [])

    if colors is None:
        pts = [(float(x), float(y), float(z)) for x, y, z in xyz]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud(header, fields, pts)

    rgb_f = _pack_rgb(colors)
    pts = [
        (float(xyz[i, 0]), float(xyz[i, 1]), float(xyz[i, 2]), float(rgb_f[i]))
        for i in range(len(xyz))
    ]
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return pc2.create_cloud(header, fields, pts)


def write_ply_xyzrgb(
    path: Path,
    xyz: np.ndarray,
    colors: Optional[np.ndarray] = None,
) -> None:
    """写 ASCII PLY（xyz + 可选 uchar rgb）。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = int(xyz.shape[0])
    has_color = colors is not None and colors.shape[0] == n
    with path.open('w', encoding='utf-8') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {n}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        if has_color:
            f.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        f.write('end_header\n')
        if has_color:
            rgb_u8 = np.clip(colors * 255.0, 0, 255).astype(np.uint8)
            for i in range(n):
                x, y, z = xyz[i]
                r, g, b = rgb_u8[i]
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n')
        else:
            for i in range(n):
                x, y, z = xyz[i]
                f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')


def transform_msg_to_matrix(t) -> np.ndarray:
    """geometry_msgs/Transform → 4x4。"""
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    R = np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[0, 3] = t.translation.x
    T[1, 3] = t.translation.y
    T[2, 3] = t.translation.z
    return T
