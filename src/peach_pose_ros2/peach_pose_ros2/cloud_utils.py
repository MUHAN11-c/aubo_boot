"""
检测框点云 — 深度反投影、rgb 位打包与 PointCloud2 组装.

职责:
  把检测框内像素的实测深度反投影成彩色点云（~/detection_cloud /
  /peach/perception/single_cloud），供 RViz 对照相机全图点云。
  依赖 sensor_msgs / numpy，不依赖 rclpy 节点。

坐标系/单位约定:
  深度 depth_mm 为 uint16 毫米（Percipio 原始值已 × depth_scale_unit）；
  输出 xyz 为相机系米制点（坐标系解释跟随组装时 header.frame_id，
  节点侧会把点先变到输出系再组消息）。rgb 字段按 0xRRGGBB 位打包成
  float32（PointCloud2 rgb 打包约定）。
"""
from __future__ import annotations

from typing import Tuple

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def _pack_rgb_bgr(bgr: np.ndarray) -> np.ndarray:
    """
    (N,3) uint8 BGR → (N,) float32：按位打包成 PointCloud2 的 rgb 字段.

    Args:
        bgr: (N, 3) uint8 数组，列序为 B、G、R（OpenCV 惯例）.

    Returns
    -------
        (N,) float32 视图（位内容为 0xRRGGBB，符合 PointCloud2 rgb 打包约定）.

    """
    b = bgr[:, 0].astype(np.uint32)
    g = bgr[:, 1].astype(np.uint32)
    r = bgr[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32)


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


def _xyzrgb_to_cloud(header: Header, xyz: np.ndarray, rgb_f: np.ndarray) -> PointCloud2:
    """
    组装 xyz + 打包 rgb → PointCloud2 消息（x/y/z 各一个 FLOAT32 + rgb 位打包）.

    走官方 sensor_msgs_py.point_cloud2.create_cloud；fields 手工声明是因为
    官方预置只有 create_cloud_xyz32（纯 xyz 无 rgb），带打包 rgb 的自定义
    布局必须显式给 fields——这是官方 API 对自定义布局的标准用法。

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
    pts = [
        (float(xyz[i, 0]), float(xyz[i, 1]), float(xyz[i, 2]), float(rgb_f[i]))
        for i in range(len(xyz))
    ]
    return pc2.create_cloud(header, fields, pts)
