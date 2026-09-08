from __future__ import annotations
"""积分：点云、有界 ICP、TSDF、覆盖。"""


import numpy as np
from peach_perception.target_reconstruction.tsdf_volume import require_open3d

DEPTH_SATURATED_MM = 65535  # uint16 饱和值 [mm]，视为无效深度
_DEPTH_TRUNC_M = 1000.0     # open3d 深度截断 [m]（饱和已预清零，仅作兜底）


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


def _depth_image_o3d(depth_mm: np.ndarray):
    """
    uint16 毫米深度 → open3d Image（饱和 65535 预清零，语义与掩膜一致）.

    open3d 只把 0 当无效深度，毫米饱和值 65535（65.535 m）必须显式置 0，
    否则会作为合法远点入云。输入数组先拷贝再改，绝不原地改调用方数据。

    Args:
        depth_mm: (H, W) uint16 深度 [mm].

    Returns
    -------
        open3d.geometry.Image（uint16）.

    """
    o3d = require_open3d()
    img = np.array(depth_mm, dtype=np.uint16, copy=True)
    # cv_bridge 零拷贝数组带显式字节序 dtype（'<u2'），open3d 的 buffer
    # 检查只认原生字节序（'H'/'=H'），'<H' 会被拒收；x86_64 上 '<u2'
    # 与原生等价，重标记即可（大端 '>u2' 先 byteswap 再重标记）
    if img.dtype.byteorder == '>':
        img = img.byteswap().view(np.uint16)
    elif img.dtype.byteorder == '<':
        img = img.view(np.uint16)
    img[img >= DEPTH_SATURATED_MM] = 0
    return o3d.geometry.Image(img)


def _intrinsic_o3d(camera_K: dict, scale: float, width: int, height: int):
    """
    内参 dict → open3d PinholeCameraIntrinsic（scale 用于切片后的等比缩放）.

    Args:
        camera_K: 内参 dict {"fx","fy","cx","cy"}（原图像素单位）.
        scale: 预切片步长（1.0=不缩放；stride>1 时 fx/s 等保持投影等价）.
        width: 图像宽 [px].
        height: 图像高 [px].

    Returns
    -------
        open3d.camera.PinholeCameraIntrinsic.

    """
    o3d = require_open3d()
    s = float(scale)
    return o3d.camera.PinholeCameraIntrinsic(
        int(width), int(height),
        float(camera_K['fx']) / s, float(camera_K['fy']) / s,
        float(camera_K['cx']) / s, float(camera_K['cy']) / s)


def apply_target_mask(depth_mm: np.ndarray, target_mask=None) -> tuple:
    """将深度限制到单目标掩膜，并返回掩膜内有效深度占比."""
    depth = np.asarray(depth_mm)
    if target_mask is None:
        return depth, valid_depth_ratio(depth)
    mask = np.asarray(target_mask)
    if mask.shape != depth.shape[:2]:
        raise ValueError(
            f'目标掩膜尺寸 {mask.shape} 与深度 {depth.shape[:2]} 不一致')
    selected = mask > 0
    pixels = int(np.count_nonzero(selected))
    if pixels == 0:
        return np.zeros_like(depth), 0.0
    masked = np.where(selected, depth, 0).astype(depth.dtype, copy=False)
    # 与 valid_depth_mask 同口径：非 0 且非饱和（io.md「有效深度」），
    # 否则强反光/近距饱和像素会抬高 min_mask_depth_ratio 门与记录值。
    valid = np.count_nonzero(
        selected & np.isfinite(depth) & (depth > 0)
        & (depth < DEPTH_SATURATED_MM))
    return masked, float(valid) / float(pixels)


def build_cloud_base(depth_mm: np.ndarray, camera_K: dict,
                     T_base_camera: np.ndarray,
                     rgb_bgr: np.ndarray = None,
                     stride: int = 1, target_mask=None) -> tuple:
    """
    一帧深度 → base 系点云 [m] + 逐点颜色 + 有效深度占比.

    几何/颜色走 open3d 官方 API：先以单位外参建相机系云，再
    ``pcd.transform(T_base_camera)`` 变 base 系（不用 create_from_* 的
    extrinsic 参数——open3d 那里是经典 CV「world→camera」约定、内部
    取逆，transform 才是正向语义）。stride 在无图路径用官方 stride
    参数，有图路径预切片 + 内参等比缩放（两者均逐点等价）。

    Args:
        depth_mm: (H, W) uint16 深度 [mm].
        camera_K: 内参 dict {"fx","fy","cx","cy"}.
        T_base_camera: (4, 4) 齐次矩阵（base←camera）.
        rgb_bgr: (H, W, 3) uint8 彩色图（OpenCV BGR 排列，与深度同分辨率）；
            None 时只建几何，颜色返回 None.
        stride: 降采样步长（像素）.

    Returns
    -------
        (cloud_base, colors_bgr, ratio, masked_depth)：cloud_base 为
        (N, 3) float64 [m]；colors_bgr 为 (N, 3) uint8（BGR，与 cloud_base
        逐点对应）或 None；ratio 为有效深度占比 [0, 1]；masked_depth 为
        掩膜后深度（无掩膜时即输入数组），调用方复用免二次计算。

    """
    o3d = require_open3d()
    depth_work, ratio = apply_target_mask(depth_mm, target_mask)
    stride = max(1, int(stride))
    T = np.asarray(T_base_camera, dtype=np.float64)
    if rgb_bgr is None:
        # 无图路径：官方 stride 采样（抽样像素坐标保持原图坐标）
        h, w = depth_work.shape[:2]
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            _depth_image_o3d(depth_work), _intrinsic_o3d(camera_K, 1.0, w, h),
            np.eye(4), depth_scale=1000.0, depth_trunc=_DEPTH_TRUNC_M,
            stride=stride, project_valid_depth_only=True)
        pcd.transform(T)
        return (np.asarray(pcd.points, dtype=np.float64).reshape(-1, 3),
                None, ratio, depth_work)
    # 有图路径：create_from_rgbd_image 无 stride 参数，预切片 + 内参缩放
    d = depth_work[::stride, ::stride]
    img = np.ascontiguousarray(rgb_bgr[::stride, ::stride, ::-1])  # BGR→RGB
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(img), _depth_image_o3d(d),
        depth_scale=1000.0, depth_trunc=_DEPTH_TRUNC_M,
        convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, _intrinsic_o3d(camera_K, float(stride), d.shape[1], d.shape[0]),
        np.eye(4), project_valid_depth_only=True)
    pcd.transform(T)
    xyz = np.asarray(pcd.points, dtype=np.float64).reshape(-1, 3)
    # [0,1] RGB → uint8 BGR（uint8→float→uint8 往返无损，已逐值比对）
    rgb01 = np.asarray(pcd.colors, dtype=np.float64)
    colors = np.clip(np.round(rgb01[:, ::-1] * 255.0),
                     0, 255).astype(np.uint8)
    return xyz, colors, ratio, depth_work


class Open3dCloudBuilder:
    """
    Open3d 点云构建薄壳（无状态，委托模块函数）.

    workhorse 本体是模块函数 build_cloud_base（单测直接锚定）；
    本类把 rgb 提前为第二参数，便于编排层位置传参。
    """

    def build(self, depth_mm: np.ndarray, rgb_bgr=None,
              camera_K: dict = None, T_base_camera: np.ndarray = None,
              stride: int = 1, target_mask=None) -> tuple:
        """
        委托 build_cloud_base.

        Args:
            depth_mm: (H, W) uint16 深度 [mm].
            rgb_bgr: (H, W, 3) uint8 BGR；None 只建几何.
            camera_K: 内参 dict {"fx","fy","cx","cy"}.
            T_base_camera: (4, 4) base←camera 位姿.
            stride: 降采样步长（像素）.

        Returns
        -------
            (xyz_base, colors_bgr|None, valid_depth_ratio,
            masked_depth)：masked_depth 为掩膜后深度（供 TSDF 积分/
            帧存档复用，调用方不必再跑一次 apply_target_mask）。

        """
        return build_cloud_base(depth_mm, camera_K, T_base_camera,
                                rgb_bgr=rgb_bgr, stride=stride,
                                target_mask=target_mask)
