from __future__ import annotations
"""检测点云：检测框深度反投影与 PointCloud2 组装。"""

from typing import Tuple

import numpy as np
from peach_perception.common.tf_utils import pack_rgb_bgr
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

# RGB 位打包统一走 common.geometry.pack_rgb_bgr（与重建侧同实现）
_pack_rgb_bgr = pack_rgb_bgr


def _bbox_cloud_xyzrgb(
    rgb_bgr: np.ndarray,
    depth_mm: np.ndarray,
    K: dict,
    bboxes,
    stride: int = 1,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    检测框内像素反投影成彩色点云：返回 (N,3) xyz（米）与 (N,) 打包 rgb.

    depth_mm 为毫米单位 uint16（Percipio 原始值已 × depth_scale_unit）；
    剔除无效深度（0/饱和 65535），stride 为降采样步长。

    Args:
        rgb_bgr: (H, W, 3) uint8 BGR 图，与深度对齐.
        depth_mm: (H, W) uint16 深度，单位毫米.
        K: 相机内参 {"fx","fy","cx","cy"}（像素单位）.
        bboxes: 检测框列表 [(x1, y1, x2, y2)]（像素，自动裁剪到图内）.
        stride: 降采样步长（像素）；1 为不降采样.

    Returns
    -------
        (xyz, rgb_packed)：xyz 为 (N, 3) float64 相机系点（米），
        rgb_packed 为 (N,) float32 打包颜色；无有效点时均为空数组.

    """
    h, w = depth_mm.shape[:2]
    mask = np.zeros((h, w), dtype=bool)
    for bbox in bboxes:
        x1, y1, x2, y2 = [int(v) for v in bbox]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))
        if x2 <= x1 or y2 <= y1:
            continue
        mask[y1:y2:stride, x1:x2:stride] = True
    # 有效深度：>0 且非饱和
    valid = mask & (depth_mm > 0) & (depth_mm < 65535)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0,), dtype=np.float32)

    vs, us = np.where(valid)
    z = depth_mm[vs, us].astype(np.float64) / 1000.0
    fx, fy = float(K['fx']), float(K['fy'])
    cx, cy = float(K['cx']), float(K['cy'])
    x = (us.astype(np.float64) - cx) * z / fx
    y = (vs.astype(np.float64) - cy) * z / fy
    xyz = np.column_stack((x, y, z))
    rgb_packed = _pack_rgb_bgr(rgb_bgr[vs, us])
    return xyz, rgb_packed


def _xyzrgb_to_cloud_msg(header: Header, xyz: np.ndarray, rgb_f: np.ndarray) -> PointCloud2:
    """
    组装 xyz + 打包 rgb → PointCloud2 消息（x/y/z 各一个 FLOAT32 + rgb 位打包）.

    走官方 sensor_msgs_py.point_cloud2.create_cloud 的 numpy 结构化数组
    快路径（与 target_reconstruction.publish.xyzrgb_to_cloud_msg 同款）；
    fields 手工声明是因为官方预置只有 create_cloud_xyz32（纯 xyz 无 rgb），
    带打包 rgb 的自定义布局必须显式给 fields——这是官方 API 对自定义
    布局的标准用法。

    Args:
        header: 输出消息头（frame_id 决定点云坐标系解释）.
        xyz: (N, 3) 点坐标（单位随 header 坐标系，通常米）；空数组给空云.
        rgb_f: (N,) float32 打包 rgb（见 _pack_rgb_bgr）.

    Returns
    -------
        sensor_msgs/PointCloud2.

    """
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    if xyz.size == 0:
        return pc2.create_cloud(header, fields, [])
    pts = np.zeros((len(xyz), 4), dtype=np.float32)
    pts[:, :3] = xyz
    pts[:, 3] = rgb_f
    return pc2.create_cloud(header, fields, pts)
