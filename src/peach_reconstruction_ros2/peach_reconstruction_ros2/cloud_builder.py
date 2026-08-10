"""
深度反投影与点云坐标变换 — 几何核心走 Open3D 官方 API（懒加载，无 ROS）.

单位约定:
  - 深度图为 uint16「毫米」[mm]（上游经
    peach_pose_ros2.peach_pose.depth_geometry.normalize_depth_to_uint16_mm
    归一化）；0 与饱和值 65535 一律视为无效深度；
  - 点云坐标一律「米」[m]；颜色为 uint8 BGR（OpenCV 惯例），发布侧经
    pack_rgb_bgr 打包成 float32 位模式的 ``rgb`` 字段（RViz RGB8 约定）；
  - T_base_camera 为 4×4 齐次矩阵（base←camera）：p_base = R @ p_camera + t。

官方 API 与语义等价性（2026-08-10 本包 A/B 对拍）:
  - 反投影：``PointCloud.create_from_depth_image``（uint16 原生，
    ``depth_scale=1000.0`` → z=d/1000 m；pinhole 公式与官方文档一致；
    ``stride`` 为官方采样步长，抽样像素坐标保持原图坐标，与旧手写
    「切片+坐标乘回 stride」逐点一致）；相机系结果 diff=0.0，
    经 transform 后 ≤3e-8 m（open3d 内部 float32 舍入量级）；
  - 彩色建云：``RGBDImage.create_from_color_and_depth`` +
    ``create_from_rgbd_image``（颜色同像素采样；uint8→[0,1]→uint8
    往返无损，已逐值比对）；
  - 坐标变换：建云时用单位外参（相机系），再 ``cloud.transform``
    （T_base_camera）变到 base 系——不用 create_from_* 的 extrinsic
    参数：open3d 该参数沿用经典 CV「world→camera」约定，内部取逆
    （与 TSDF integrate 同坑），transform 才是正向语义；
  - ``create_from_rgbd_image`` 无 stride 参数：stride>1 的彩色路径
    预切片 + 内参等比缩放（fx/s, fy/s, cx/s, cy/s），解析上等价。

保留的 numpy 段（非自造几何，官方无对应物或属数据预处理）:
  - 有效深度掩膜/占比：指标计算，与几何求交无关；
  - 饱和 65535 预清零：open3d 只认 0 为无效深度，毫米饱和值须先置 0；
  - BGR↔RGB 通道翻转：OpenCV↔open3d 颜色序约定；
  - pack_rgb_bgr：RViz float32 位打包 rgb 字段，官方无打包 API。

分层契约：build_cloud_base 是模块级 workhorse 函数（单测直接锚定）；
Open3dCloudBuilder 是其实现 interfaces.CloudBuilder 的薄壳（编排层按
注册表 CLOUD_BUILDERS 实例化，设计文档 §2.2）。
"""
from __future__ import annotations

import numpy as np

from peach_reconstruction_ros2.interfaces import CloudBuilder
from peach_reconstruction_ros2.tsdf_volume import require_open3d

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


def backproject_depth(depth_mm: np.ndarray, camera_K: dict,
                      stride: int = 1) -> np.ndarray:
    """
    uint16 毫米深度反投影为相机系点云 [m]（open3d 官方，pinhole 模型）.

    x = (u - cx) * z / fx；y = (v - cy) * z / fy；z = depth_mm / 1000。

    Args:
        depth_mm: (H, W) uint16 深度 [mm]，与内参同分辨率.
        camera_K: 内参 dict，键 {"fx","fy","cx","cy"}（像素单位）.
        stride: 降采样步长（像素）；1 为不降采样.

    Returns
    -------
        (N, 3) float64 相机系点 [m]；无有效深度时给 (0, 3) 空数组.

    """
    o3d = require_open3d()
    h, w = depth_mm.shape[:2]
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        _depth_image_o3d(depth_mm), _intrinsic_o3d(camera_K, 1.0, w, h),
        np.eye(4), depth_scale=1000.0, depth_trunc=_DEPTH_TRUNC_M,
        stride=max(1, int(stride)), project_valid_depth_only=True)
    return np.asarray(pcd.points, dtype=np.float64).reshape(-1, 3)


def transform_points(T_base_camera: np.ndarray,
                     cloud_camera: np.ndarray) -> np.ndarray:
    """
    点云由相机系变到 base 系：p_base = R @ p_camera + t（numpy 线性代数）.

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
        (cloud_base, colors_bgr, ratio)：cloud_base 为 (N, 3) float64 [m]；
        colors_bgr 为 (N, 3) uint8（BGR，与 cloud_base 逐点对应）或 None；
        ratio 为有效深度占比 [0, 1].

    """
    o3d = require_open3d()
    stride = max(1, int(stride))
    T = np.asarray(T_base_camera, dtype=np.float64)
    if rgb_bgr is None:
        # 无图路径：官方 stride 采样（抽样像素坐标保持原图坐标）
        h, w = depth_mm.shape[:2]
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            _depth_image_o3d(depth_mm), _intrinsic_o3d(camera_K, 1.0, w, h),
            np.eye(4), depth_scale=1000.0, depth_trunc=_DEPTH_TRUNC_M,
            stride=stride, project_valid_depth_only=True)
        pcd.transform(T)
        return (np.asarray(pcd.points, dtype=np.float64).reshape(-1, 3),
                None, valid_depth_ratio(depth_mm))
    # 有图路径：create_from_rgbd_image 无 stride 参数，预切片 + 内参缩放
    d = depth_mm[::stride, ::stride]
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
    return xyz, colors, valid_depth_ratio(depth_mm)


def pack_rgb_bgr(colors_bgr: np.ndarray) -> np.ndarray:
    """
    (N, 3) uint8 BGR → (N,) float32 位打包（0xRRGGBB，RViz RGB8 约定）.

    语义与 peach_pose_node._pack_rgb_bgr 一致：r<<16 | g<<8 | b 塞进
    float32 位模式，PointCloud2 里以名为 ``rgb`` 的 FLOAT32 字段承载。
    sensor_msgs_py 无颜色位打包 API，此为唯一保留的手写转换。

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


class Open3dCloudBuilder(CloudBuilder):
    """
    interfaces.CloudBuilder 的 open3d 实现薄壳（无状态，委托模块函数）.

    workhorse 本体是模块函数 build_cloud_base（单测直接锚定，避免
    「类委托函数、函数再委托类」的双向跳转）；本类只把签名对齐到
    ABC 形态（rgb 提前为第二参数，便于编排层位置传参）。
    """

    def build(self, depth_mm: np.ndarray, rgb_bgr=None,
              camera_K: dict = None, T_base_camera: np.ndarray = None,
              stride: int = 1) -> tuple:
        """
        委托 build_cloud_base（签名对齐 interfaces.CloudBuilder）.

        Args:
            depth_mm: (H, W) uint16 深度 [mm].
            rgb_bgr: (H, W, 3) uint8 BGR；None 只建几何.
            camera_K: 内参 dict {"fx","fy","cx","cy"}.
            T_base_camera: (4, 4) base←camera 位姿.
            stride: 降采样步长（像素）.

        Returns
        -------
            (xyz_base, colors_bgr|None, valid_depth_ratio).

        """
        return build_cloud_base(depth_mm, camera_K, T_base_camera,
                                rgb_bgr=rgb_bgr, stride=stride)


# 实现注册表（显式字典，yolo_ros 先例；编排层按名实例化）
CLOUD_BUILDERS = {'open3d': Open3dCloudBuilder}
