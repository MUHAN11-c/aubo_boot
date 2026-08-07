"""
深度反投影与点云坐标变换 — 纯 numpy 几何工具.

单位约定:
  - 深度图为 uint16「毫米」[mm]（上游经
    peach_pose_ros2.peach_pose.depth_geometry.normalize_depth_to_uint16_mm
    归一化）；0 与饱和值 65535 一律视为无效深度；
  - 点云坐标一律「米」[m]；颜色为 uint8 BGR（OpenCV 惯例），发布侧经
    pack_rgb_bgr 打包成 float32 位模式的 ``rgb`` 字段（RViz RGB8 约定）；
  - T_base_camera 为 4×4 齐次矩阵（base←camera）：p_base = R @ p_camera + t。

本模块不依赖 ROS，便于离线单测与复用。
"""
from __future__ import annotations

import numpy as np

DEPTH_SATURATED_MM = 65535  # uint16 饱和值 [mm]，视为无效深度


def valid_depth_mask(depth_mm: np.ndarray) -> np.ndarray:
    """
    有效深度掩膜：>0 且非饱和.

    Args:
        depth_mm: (H, W) uint16 深度 [mm].

    Returns
    -------
        (H, W) bool 掩膜.

    """
    return (depth_mm > 0) & (depth_mm < DEPTH_SATURATED_MM)


def valid_depth_ratio(depth_mm: np.ndarray) -> float:
    """
    有效深度占比（有效像素 / 总像素）.

    Args:
        depth_mm: (H, W) uint16 深度 [mm].

    Returns
    -------
        [0, 1] 浮点占比；空图给 0.0.

    """
    total = int(depth_mm.size)
    if total == 0:
        return 0.0
    return float(np.count_nonzero(valid_depth_mask(depth_mm))) / float(total)


def _backproject_with_pixels(depth_mm: np.ndarray, camera_K: dict,
                             stride: int = 1) -> tuple:
    """
    反投影并返回每个点的源像素坐标（供颜色同步采样）.

    Args:
        depth_mm: (H, W) uint16 深度 [mm].
        camera_K: 内参 dict {"fx","fy","cx","cy"}.
        stride: 降采样步长（像素）.

    Returns
    -------
        (xyz, us, vs)：xyz 为 (N, 3) float64 相机系点 [m]；us/vs 为 (N,)
        int 原图像素坐标（与 xyz 逐点对应，行主序）.

    """
    stride = max(1, int(stride))
    roi = depth_mm[::stride, ::stride]
    valid = valid_depth_mask(roi)
    if not np.any(valid):
        return (np.zeros((0, 3), dtype=np.float64),
                np.zeros((0,), dtype=int), np.zeros((0,), dtype=int))
    vs, us = np.nonzero(valid)
    z = roi[vs, us].astype(np.float64) / 1000.0  # [mm] → [m]
    # 抽样网格坐标乘回原图像素坐标
    us = us * stride
    vs = vs * stride
    fx, fy = float(camera_K['fx']), float(camera_K['fy'])
    cx, cy = float(camera_K['cx']), float(camera_K['cy'])
    x = (us.astype(np.float64) - cx) * z / fx
    y = (vs.astype(np.float64) - cy) * z / fy
    return np.column_stack((x, y, z)), us, vs


def backproject_depth(depth_mm: np.ndarray, camera_K: dict,
                      stride: int = 1) -> np.ndarray:
    """
    uint16 毫米深度反投影为相机系点云 [m]（pinhole 模型）.

    x = (u - cx) * z / fx；y = (v - cy) * z / fy；z = depth_mm / 1000。

    Args:
        depth_mm: (H, W) uint16 深度 [mm]，与内参同分辨率.
        camera_K: 内参 dict，键 {"fx","fy","cx","cy"}（像素单位）.
        stride: 降采样步长（像素）；1 为不降采样.

    Returns
    -------
        (N, 3) float64 相机系点 [m]；无有效深度时给 (0, 3) 空数组.

    """
    xyz, _, _ = _backproject_with_pixels(depth_mm, camera_K, stride=stride)
    return xyz


def transform_points(T_base_camera: np.ndarray,
                     cloud_camera: np.ndarray) -> np.ndarray:
    """
    点云由相机系变到 base 系：p_base = R @ p_camera + t.

    Args:
        T_base_camera: (4, 4) 齐次矩阵（base←camera）.
        cloud_camera: (N, 3) 相机系点 [m].

    Returns
    -------
        (N, 3) float64 base 系点 [m]；空输入给 (0, 3) 空数组.

    """
    cloud = np.asarray(cloud_camera, dtype=np.float64)
    if cloud.size == 0:
        return cloud.reshape(0, 3)
    R = T_base_camera[:3, :3]
    t = T_base_camera[:3, 3]
    return (R @ cloud.T).T + t


def build_cloud_base(depth_mm: np.ndarray, camera_K: dict,
                     T_base_camera: np.ndarray,
                     rgb_bgr: np.ndarray = None,
                     stride: int = 1) -> tuple:
    """
    一帧深度 → base 系点云 [m] + 逐点颜色 + 有效深度占比.

    Args:
        depth_mm: (H, W) uint16 深度 [mm].
        camera_K: 内参 dict {"fx","fy","cx","cy"}.
        T_base_camera: (4, 4) 齐次矩阵（base←camera）.
        rgb_bgr: (H, W, 3) uint8 彩色图（OpenCV BGR 排列，与深度同分辨率）；
            None 时只建几何，颜色返回 None.
        stride: 降采样步长（像素）.

    Returns
    -------
        (cloud_base, colors_bgr, ratio)：cloud_base 为 (N, 3) float64 [m]；
        colors_bgr 为 (N, 3) uint8（BGR，与 cloud_base 逐点对应）或 None；
        ratio 为有效深度占比 [0, 1].

    """
    cloud_camera, us, vs = _backproject_with_pixels(
        depth_mm, camera_K, stride=stride)
    cloud_base = transform_points(T_base_camera, cloud_camera)
    colors = None
    if rgb_bgr is not None:
        # 与深度逐点同像素采样（BGR 原样保留，打包在发布侧做）
        colors = np.asarray(rgb_bgr)[vs, us].astype(np.uint8).reshape(-1, 3)
    return cloud_base, colors, valid_depth_ratio(depth_mm)


def pack_rgb_bgr(colors_bgr: np.ndarray) -> np.ndarray:
    """
    (N, 3) uint8 BGR → (N,) float32 位打包（0xRRGGBB，RViz RGB8 约定）.

    语义与 peach_pose_node._pack_rgb_bgr 一致：r<<16 | g<<8 | b 塞进
    float32 位模式，PointCloud2 里以名为 ``rgb`` 的 FLOAT32 字段承载。

    Args:
        colors_bgr: (N, 3) uint8 数组，列序为 B、G、R（OpenCV 惯例）.

    Returns
    -------
        (N,) float32 视图（位内容为 0xRRGGBB）；空输入给 (0,) 空数组.

    """
    colors = np.asarray(colors_bgr, dtype=np.uint8).reshape(-1, 3)
    if colors.shape[0] == 0:
        return np.zeros((0,), dtype=np.float32)
    b = colors[:, 0].astype(np.uint32)
    g = colors[:, 1].astype(np.uint32)
    r = colors[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32)
