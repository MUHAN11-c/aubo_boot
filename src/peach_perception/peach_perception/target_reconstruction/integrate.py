from __future__ import annotations
"""积分：点云、有界 ICP、TSDF、覆盖。"""

from dataclasses import dataclass
import time
from typing import (
    Callable,
    List,
    Optional,
    Tuple,
)

import numpy as np
from peach_perception.common.ema import ScalarEma
from peach_perception.common.geometry import (
    angle_between_deg,
    invert_transform,
    relative_motion,
    unit_vector as _safe_unit,
)
from peach_perception.target_reconstruction.interfaces import (
    CLOUD_BUILDERS,
    CloudBuilder,
    Refiner,
    REFINERS,
    Volume,
    VOLUMES,
)
from scipy.spatial import cKDTree


# === tsdf_volume.py ===

_O3D = None  # 懒加载缓存（无 open3d 的环境仍可 import 本模块）


def require_open3d():
    """返回 open3d 模块；缺失时抛带指引的 RuntimeError."""
    global _O3D
    if _O3D is None:
        try:
            import open3d as o3d
        except ImportError as exc:
            raise RuntimeError(
                'open3d 不可用：TSDF 功能须在工作区 venv（aubo_py3.12）'
                '解释器下运行') from exc
        _O3D = o3d
    return _O3D


class LocalTsdf(Volume):
    """局部 TSDF 体积：在线积分 → 点云/网格提取 → ROI 后处理."""

    def __init__(self, voxel_length: float = 0.003, sdf_trunc: float = 0.012,
                 depth_trunc: float = 1.5, now: Optional[Callable] = None):
        """
        建 TSDF 体积（参数单位均 [m]）.

        Args:
            voxel_length: 体素边长 [m].
            sdf_trunc: 截断距离 [m].
            depth_trunc: 深度截断 [m]（更远的深度不积分）.
            now: 单调时钟（返回 float 秒，协议 I3 由编排层注入，如
                peach_perception.common.ros.RclpyClockAdapter.now）；None 回退
                time.perf_counter（纯核自包含缺省，单测直建实例用）.

        Returns
        -------
            无返回值（None）；integrate_time_s 累计积分墙钟 [s].

        """
        o3d = require_open3d()
        self.voxel_length = float(voxel_length)
        self.sdf_trunc = float(sdf_trunc)
        self.depth_trunc = float(depth_trunc)
        self._now = now if now is not None else time.perf_counter
        self._o3d = o3d
        self.reset()

    def reset(self) -> None:
        """
        清空体积与累计耗时（Volume 契约；实例可复用）.

        语义等同「弃例新建」：体积重建为同参数空 ScalableTSDFVolume，
        integrate_time_s 归零。节点现状每轮 finalize 新建实例，本方法
        为接口层轻量生命周期钩子，不改变该用法。

        Returns
        -------
            无返回值（None）.

        """
        self._volume = self._o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=(self._o3d.pipelines.integration
                        .TSDFVolumeColorType.RGB8))
        self.integrate_time_s = 0.0

    def _make_rgbd(self, rgb_bgr: np.ndarray, depth_mm: np.ndarray,
                   camera_K: dict):
        """
        组 Open3D RGBDImage 与内参（BGR→RGB、uint16[mm]→float32[m]）.

        Args:
            rgb_bgr: (H, W, 3) uint8 BGR.
            depth_mm: (H, W) uint16 深度 [mm]（0 与超 depth_trunc 不积分）.
            camera_K: 内参 dict {"fx","fy","cx","cy"}.

        Returns
        -------
            (rgbd, intrinsic)：Open3D 对象对.

        """
        o3d = require_open3d()
        h, w = depth_mm.shape[:2]
        # BGR → RGB（Open3D 颜色通道序）
        color = o3d.geometry.Image(
            np.ascontiguousarray(rgb_bgr[:, :, ::-1]))
        depth_m = np.ascontiguousarray(
            depth_mm.astype(np.float32) / 1000.0)  # [mm] → [m]
        depth = o3d.geometry.Image(depth_m)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale=1.0,  # 深度已是米制，scale=1
            depth_trunc=self.depth_trunc, convert_rgb_to_intensity=False)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=int(w), height=int(h),
            fx=float(camera_K['fx']), fy=float(camera_K['fy']),
            cx=float(camera_K['cx']), cy=float(camera_K['cy']))
        return rgbd, intrinsic

    def _integrate(self, rgbd, intrinsic,
                   extrinsic_camera_base: np.ndarray) -> None:
        """
        底层积分入口：直接给 world→camera 外参（方向由调用方负责）.

        Args:
            rgbd: Open3D RGBDImage.
            intrinsic: Open3D PinholeCameraIntrinsic.
            extrinsic_camera_base: (4, 4) world(base)→camera 外参.

        Returns
        -------
            无返回值（None）；耗时累计进 integrate_time_s.

        """
        t0 = self._now()
        self._volume.integrate(rgbd, intrinsic,
                               np.asarray(extrinsic_camera_base,
                                          dtype=np.float64))
        self.integrate_time_s += self._now() - t0

    def integrate_frame(self, rgb_bgr: np.ndarray, depth_mm: np.ndarray,
                        camera_K: dict, T_base_camera: np.ndarray) -> None:
        """
        积分一帧：BGR 彩图 + uint16 毫米深度 + base←camera 位姿.

        Args:
            rgb_bgr: (H, W, 3) uint8 BGR（CapturedFrame.rgb）.
            depth_mm: (H, W) uint16 深度 [mm].
            camera_K: 内参 dict {"fx","fy","cx","cy"}.
            T_base_camera: (4, 4) camera→base；内部取逆得 T_camera_base
                （world→camera）传给 Open3D——方向反了云会整体错位.

        Returns
        -------
            无返回值（None）；耗时累计进 integrate_time_s.

        """
        rgbd, intrinsic = self._make_rgbd(rgb_bgr, depth_mm, camera_K)
        # 外参方向：ROS 存 camera→base，Open3D 要 base(world)→camera
        T_camera_base = invert_transform(
            np.asarray(T_base_camera, dtype=np.float64))
        self._integrate(rgbd, intrinsic, T_camera_base)

    def extract_cloud(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        提取 TSDF 点云.

        Returns
        -------
            (xyz, colors_bgr)：xyz 为 (N, 3) float64 [m]（base 系）；
            colors_bgr 为 (N, 3) uint8 BGR（无颜色时给 None）.

        """
        pcd = self._volume.extract_point_cloud()
        xyz = np.asarray(pcd.points, dtype=np.float64)
        colors = None
        if pcd.has_colors():
            rgb01 = np.asarray(pcd.colors, dtype=np.float64)  # [0,1] RGB
            colors = np.clip(np.round(rgb01[:, ::-1] * 255.0),
                             0, 255).astype(np.uint8)  # → BGR uint8
        return xyz, colors

    def extract_mesh(self, center=None, size_xyz=None) -> dict:
        """
        提取三角网格并计算顶点法向，可选按 base 系轴对齐盒裁剪.

        Returns
        -------
            dict：vertices、triangles、normals、colors_bgr 四个 numpy 数组。

        """
        mesh = self._volume.extract_triangle_mesh()
        if center is not None and size_xyz is not None and len(mesh.vertices):
            c = np.asarray(center, dtype=np.float64)
            half = np.asarray(size_xyz, dtype=np.float64) / 2.0
            box = self._o3d.geometry.AxisAlignedBoundingBox(c - half, c + half)
            mesh = mesh.crop(box)
        mesh.compute_vertex_normals()
        colors = np.zeros((len(mesh.vertices), 3), dtype=np.uint8)
        if mesh.has_vertex_colors():
            colors = np.clip(np.round(
                np.asarray(mesh.vertex_colors)[:, ::-1] * 255.0),
                0, 255).astype(np.uint8)
        return {
            'vertices': np.asarray(mesh.vertices, dtype=np.float64),
            'triangles': np.asarray(mesh.triangles, dtype=np.int32),
            'normals': np.asarray(mesh.vertex_normals, dtype=np.float64),
            'colors_bgr': colors,
        }

    @staticmethod
    def crop_to_box(xyz: np.ndarray, colors: Optional[np.ndarray],
                    center, size_xyz) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        轴对齐盒裁剪（base 系 ROI）.

        Args:
            xyz: (N, 3) 点 [m].
            colors: (N, 3) 颜色或 None（与 xyz 同步过滤）.
            center: (3,) 盒中心 [m].
            size_xyz: (3,) 盒尺寸 [m]（local_volume.size_x/y/z）.

        Returns
        -------
            (xyz_in, colors_in)：盒内点与颜色；空云给 (0, 3) 空数组.

        """
        if xyz.size == 0:
            return xyz.reshape(0, 3), colors
        c = np.asarray(center, dtype=np.float64)
        half = np.asarray(size_xyz, dtype=np.float64) / 2.0
        mask = (np.abs(xyz - c) <= half).all(axis=1)
        colors_in = colors[mask] if colors is not None else None
        return xyz[mask], colors_in

    @staticmethod
    def voxel_downsample(xyz: np.ndarray, colors: Optional[np.ndarray],
                         voxel_size: float
                         ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        体素降采样（voxel_size ≤ 0 或空云时原样返回）.

        Args:
            xyz: (N, 3) 点 [m].
            colors: (N, 3) uint8 BGR 或 None.
            voxel_size: 体素边长 [m].

        Returns
        -------
            (xyz_down, colors_down).

        """
        if voxel_size <= 0.0 or xyz.size == 0:
            return xyz, colors
        o3d = require_open3d()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        if colors is not None and len(colors) == len(xyz):
            pcd.colors = o3d.utility.Vector3dVector(
                colors.astype(np.float64)[:, ::-1] / 255.0)  # BGR→RGB [0,1]
        down = pcd.voxel_down_sample(float(voxel_size))
        xyz_d = np.asarray(down.points, dtype=np.float64)
        colors_d = None
        if down.has_colors():
            colors_d = np.clip(np.round(
                np.asarray(down.colors)[:, ::-1] * 255.0), 0, 255).astype(np.uint8)
        return xyz_d, colors_d

    @staticmethod
    def statistical_filter(xyz: np.ndarray, colors: Optional[np.ndarray],
                           nb_neighbors: int = 20, std_ratio: float = 2.0
                           ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        统计离群剔除（open3d remove_statistical_outlier 常用默认：20 邻域 2σ）.

        点数不足 nb_neighbors+1 时原样返回（小云无可剔除意义）。

        Args:
            xyz: (N, 3) 点 [m].
            colors: (N, 3) uint8 BGR 或 None.
            nb_neighbors: 邻域点数.
            std_ratio: 标准差倍率阈值.

        Returns
        -------
            (xyz_in, colors_in)：内点与颜色.

        """
        if xyz.shape[0] <= nb_neighbors:
            return xyz, colors
        o3d = require_open3d()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        _inlier_pcd, inlier_idx = pcd.remove_statistical_outlier(
            nb_neighbors=int(nb_neighbors), std_ratio=float(std_ratio))
        idx = np.asarray(inlier_idx, dtype=int)
        colors_in = colors[idx] if colors is not None else None
        return xyz[idx], colors_in


# 显式注册清单（2.14）：注册名 'local_tsdf'，yaml volume.impl 默认值
VOLUMES.register('local_tsdf', LocalTsdf)


# === cloud_builder.py ===

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
    valid = np.count_nonzero(selected & np.isfinite(depth) & (depth > 0))
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
        (cloud_base, colors_bgr, ratio)：cloud_base 为 (N, 3) float64 [m]；
        colors_bgr 为 (N, 3) uint8（BGR，与 cloud_base 逐点对应）或 None；
        ratio 为有效深度占比 [0, 1].

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
                None, ratio)
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
    return xyz, colors, ratio


class Open3dCloudBuilder(CloudBuilder):
    """
    interfaces.CloudBuilder 的 open3d 实现薄壳（无状态，委托模块函数）.

    workhorse 本体是模块函数 build_cloud_base（单测直接锚定，避免
    「类委托函数、函数再委托类」的双向跳转）；本类只把签名对齐到
    ABC 形态（rgb 提前为第二参数，便于编排层位置传参）。
    """

    def build(self, depth_mm: np.ndarray, rgb_bgr=None,
              camera_K: dict = None, T_base_camera: np.ndarray = None,
              stride: int = 1, target_mask=None) -> tuple:
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
                                rgb_bgr=rgb_bgr, stride=stride,
                                target_mask=target_mask)


# 显式注册清单（2.14）：注册名 'open3d_cloud'，yaml cloud_builder.impl 默认值
CLOUD_BUILDERS.register('open3d_cloud', Open3dCloudBuilder)


# === icp_refiner.py ===

@dataclass(frozen=True)
class IcpConfig:
    """两尺度点到平面 ICP 参数，长度单位均为米."""

    min_points: int = 300
    coarse_voxel: float = 0.006
    fine_voxel: float = 0.003
    coarse_correspondence: float = 0.015
    fine_correspondence: float = 0.007
    coarse_iterations: int = 20
    fine_iterations: int = 10
    min_fitness: float = 0.35
    max_rmse: float = 0.008
    max_translation: float = 0.010
    max_rotation_deg: float = 3.0


@dataclass(frozen=True)
class IcpResult:
    """一次帧到模型配准结果."""

    mode: str
    correction: np.ndarray
    fitness: float
    rmse: float
    translation_m: float
    rotation_deg: float
    reason: str

    @property
    def accepted(self) -> bool:
        """ICP 或 FK 预对齐通过质量门即允许积分."""
        return self.mode in ('icp', 'fk')


def _quality_ok(fitness: float, rmse: float, config: IcpConfig) -> bool:
    """统一的重叠度/RMSE 门，避免 ICP 与 FK 回退采用两套语义."""
    return fitness >= config.min_fitness and rmse <= config.max_rmse


class BoundedIcp(Refiner):
    """Open3D 鲁棒点到平面 ICP；机器人 FK 是绝对位姿，ICP 只做小修正."""

    def __init__(self, config: IcpConfig):
        """保存不可变配置；Open3D 到第一次 refine 时才导入."""
        self.config = config

    @staticmethod
    def _cloud(points):
        """Numpy 点集转 Open3D PointCloud."""
        o3d = require_open3d()
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(
            np.asarray(points, dtype=np.float64).reshape(-1, 3))
        return cloud

    @staticmethod
    def _prepare(cloud, voxel: float, correspondence: float):
        """降采样并估计点到平面 ICP 所需法向."""
        o3d = require_open3d()
        down = cloud.voxel_down_sample(float(voxel))
        radius = max(2.0 * float(voxel), 2.0 * float(correspondence))
        down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        return down

    def refine(self, source_fk_base: np.ndarray,
               target_base: np.ndarray) -> IcpResult:
        """
        配准当前 FK 点云到 TSDF 模型.

        模型尚未形成时返回 mode=fk 用于首帧/预热；模型形成后先评估
        FK 原位姿，再做粗细两层 ICP。ICP 越界或质量差但 FK 原位姿合格时
        返回 mode=fk；二者都差则 mode=reject。
        """
        source = np.asarray(source_fk_base, dtype=np.float64).reshape(-1, 3)
        target = np.asarray(target_base, dtype=np.float64).reshape(-1, 3)
        identity = np.eye(4, dtype=np.float64)
        if len(source) < self.config.min_points:
            return IcpResult(
                'reject', identity, -1.0, -1.0, 0.0, 0.0,
                'insufficient_source_points')
        if len(target) < self.config.min_points:
            return IcpResult(
                'fk', identity, -1.0, -1.0, 0.0, 0.0, 'model_warmup')

        o3d = require_open3d()
        source_raw = self._cloud(source)
        target_raw = self._cloud(target)
        source_fine = self._prepare(
            source_raw, self.config.fine_voxel,
            self.config.fine_correspondence)
        target_fine = self._prepare(
            target_raw, self.config.fine_voxel,
            self.config.fine_correspondence)
        initial = o3d.pipelines.registration.evaluate_registration(
            source_fine, target_fine,
            self.config.fine_correspondence, identity)

        correction = identity
        levels = (
            (self.config.coarse_voxel,
             self.config.coarse_correspondence,
             self.config.coarse_iterations),
            (self.config.fine_voxel,
             self.config.fine_correspondence,
             self.config.fine_iterations),
        )
        final = initial
        for voxel, correspondence, iterations in levels:
            source_level = self._prepare(source_raw, voxel, correspondence)
            target_level = self._prepare(target_raw, voxel, correspondence)
            loss = o3d.pipelines.registration.TukeyLoss(
                k=float(correspondence))
            estimator = (
                o3d.pipelines.registration.TransformationEstimationPointToPlane(
                    loss))
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=int(iterations))
            final = o3d.pipelines.registration.registration_icp(
                source_level, target_level, float(correspondence),
                correction, estimator, criteria)
            correction = np.asarray(final.transformation, dtype=np.float64)

        translation, rotation = relative_motion(correction, identity)
        icp_quality = _quality_ok(
            float(final.fitness), float(final.inlier_rmse), self.config)
        bounded = (translation <= self.config.max_translation
                   and rotation <= self.config.max_rotation_deg)
        if icp_quality and bounded:
            return IcpResult(
                'icp', correction, float(final.fitness),
                float(final.inlier_rmse), translation, rotation, 'accepted')

        if _quality_ok(
                float(initial.fitness), float(initial.inlier_rmse),
                self.config):
            reason = 'icp_out_of_bounds' if not bounded else 'icp_low_quality'
            return IcpResult(
                'fk', identity, float(initial.fitness),
                float(initial.inlier_rmse), 0.0, 0.0, reason)

        reason = 'icp_out_of_bounds' if not bounded else 'low_overlap'
        return IcpResult(
            'reject', correction, float(final.fitness),
            float(final.inlier_rmse), translation, rotation, reason)


# 显式注册清单（2.14）：注册名 'bounded_icp'，yaml refiner.impl 默认值；
# 编排层按名 create（kwargs 透传构造），不写算法条件分支。
REFINERS.register('bounded_icp', BoundedIcp)


# === icp_target_cache.py ===

# 修正量 EMA 平滑系数（与 timing.EMA_ALPHA 同约定：0.3 在响应速度与
# 抗单帧抖动间取折中；首个样本直接播种）
_CORR_EMA_ALPHA = 0.3
# 稳定判据相对漂移阈值的比例：EMA ≤ 1/4×阈值才拉长周期；与「≥1×阈值收回
# 下限」之间留出 (0.25, 1.0) 迟滞保持带，防 k 在边界附近逐帧振荡
_STABLE_RATIO = 0.25


@dataclass(frozen=True)
class IcpTargetRefreshConfig:
    """
    ICP target 全量刷新周期自适应配置（长度单位 [m]，周期单位 [帧]）.

    min_period/max_period/drift_ratio 来自 ROS 参数
    icp.target_refresh_min_period/max_period/drift_ratio；
    max_translation_m 注入 icp.max_translation（漂移阈值基准）；
    max_incremental_points/downsample_voxel 为容量护栏（后者注入
    tsdf.voxel_length，与模型分辨率同尺度）。
    """

    min_period: int = 1
    max_period: int = 5
    drift_ratio: float = 0.5
    max_translation_m: float = 0.010
    max_incremental_points: int = 200_000
    downsample_voxel: float = 0.003


class IcpTargetCache:
    """
    ICP target 增量复用缓存：全量基线 + 逐帧增量拼接 + 自适应周期 k.

    生命周期：随节点构造创建、跨会话复用；会话开始/绑定切换/reset/
    remove_last 重放/finalize 清理等关键事件由编排层调 invalidate()
    复位（模型已不存在或已重建，旧 target 一律作废，下帧强制全量刷新）。
    """

    def __init__(self, config: IcpTargetRefreshConfig):
        """保存配置并归一化上下限；初始状态等同 invalidate()（空缓存）."""
        self._config = config
        # 上限小于下限视为配置错误：归一 max>=min>=1 收敛行为，不抛错
        # （启动期参数层不做业务校验是现状语义，此处防御性兜底）
        self._min_period = max(1, int(config.min_period))
        self._max_period = max(self._min_period, int(config.max_period))
        self._drift_thresh = (float(config.max_translation_m)
                              * max(0.0, float(config.drift_ratio)))
        self._max_incremental_points = max(1, int(config.max_incremental_points))
        self._downsample_voxel = float(config.downsample_voxel)
        # 诊断计数（随 diagnostics JSON registration 子对象投影）
        self.full_refreshes = 0
        self.incremental_appends = 0
        self.invalidate()

    def invalidate(self) -> None:
        """
        关键事件复位：弃缓存、清零计数、自适应状态回保守初值.

        调用后 should_refresh() 恒 True（target 为空），下一次成功采帧
        必走全量 extract；修正量 EMA 与周期一并复位——新会话/新模型的
        漂移历史不遗传。
        """
        self._target: Optional[np.ndarray] = None
        self._frames_since_refresh = 0
        self._period = self._min_period  # 起步保守：无修正量证据前逐帧全量
        self._corr_ema = ScalarEma(_CORR_EMA_ALPHA)

    @property
    def period(self) -> int:
        """当前自适应刷新周期 k [帧]（诊断观测用）."""
        return self._period

    @property
    def target_size(self) -> int:
        """当前缓存 target 点数；空缓存为 0（诊断观测用）."""
        return 0 if self._target is None else int(self._target.shape[0])

    def current_target(self) -> Optional[np.ndarray]:
        """返回当前可复用 target（base 系 (N,3) [m]）；空缓存给 None."""
        return self._target

    def should_refresh(self) -> bool:
        """是否应从 TSDF 全量 extract 刷新（空缓存或距上次刷新已满 k 帧）."""
        return (self._target is None
                or self._frames_since_refresh >= self._period)

    def set_full(self, xyz: np.ndarray) -> None:
        """
        以全量 extract 结果重置基线（由 _refresh_tsdf_outputs 单点回调）.

        Args:
            xyz: (N, 3) 全量提取点云（base 系 [m]，已过 ROI/降采样/统计
                滤波后处理）；空数组表示模型尚无点，缓存保持空（下一帧
                should_refresh 仍为 True，持续重试直到模型形成）.

        Returns
        -------
            无返回值（None）；刷新计数归帧零、full_refreshes 递增.

        """
        arr = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        self._target = arr if arr.shape[0] else None
        self._frames_since_refresh = 0
        self.full_refreshes += 1

    def append_frame(self, cloud_base: np.ndarray) -> None:
        """
        非刷新帧：把本帧 ICP 修正后的 base 系点云增量并入 target.

        仅应在 should_refresh() 为 False 的成功采帧路径调用；拼接结果
        超过 max_incremental_points 时按 downsample_voxel 体素降采样
        （与提取后处理同实现），防 k 偏大时 target 无界增长拖慢 ICP
        内部降采样/KDTree/法向估计。空云帧仍推进刷新计数（帧确已积分）。

        Args:
            cloud_base: (N, 3) 本帧修正后点云（base 系 [m]，已 ROI 裁剪）.

        Returns
        -------
            无返回值（None）.

        """
        pts = np.asarray(cloud_base, dtype=np.float64).reshape(-1, 3)
        if self._target is None:
            # 防御性兜底：正常流程 target 为空时 should_refresh 恒 True，
            # 不会走到本路径；若编排层未来调整调用序，按全量基线处理
            self.set_full(pts)
            return
        if pts.shape[0]:
            self._target = np.vstack((self._target, pts))
        self._frames_since_refresh += 1
        self.incremental_appends += 1
        if self._target.shape[0] > self._max_incremental_points:
            self._target, _ = LocalTsdf.voxel_downsample(
                self._target, None, self._downsample_voxel)

    def note_result(self, mode: str, translation_m: float) -> None:
        """
        消费一次配准结果，更新修正量 EMA 并自适应伸缩刷新周期 k.

        每帧 refine 后调用（含 fk 回退/拒帧路径）：
          - mode != 'icp'（fk 回退或拒帧）：对齐风险信号，k 立即收回下限
            （下帧尽早全量刷新），且不混入修正量 EMA（fk 的 translation
            恒 0，掺入会虚假拉低 EMA）；
          - mode == 'icp'：平移修正量入 EMA；EMA ≥ 漂移阈值
            （max_translation×drift_ratio）→ k 收回下限；EMA ≤ 1/4×阈值
            → k 拉长到上限；中间迟滞带保持现周期。

        Args:
            mode: IcpResult.mode（'icp'/'fk'/'reject'）.
            translation_m: IcpResult.translation_m（相对 FK 的平移修正 [m]）.

        Returns
        -------
            无返回值（None）.

        """
        if mode != 'icp':
            self._period = self._min_period
            return
        corr = self._corr_ema.update(max(0.0, float(translation_m)))
        if corr >= self._drift_thresh:
            # 修正量偏大（漂移风险高）→ 缩短 k，更快全量刷新
            self._period = self._min_period
        elif corr <= _STABLE_RATIO * self._drift_thresh:
            # 修正量长期远小于漂移阈值（稳定）→ 拉长 k，省全量提取
            self._period = self._max_period
        # 中间区间保持现周期（迟滞带防抖）


# === overlap.py ===

DEFAULT_MAX_POINTS = 20000  # 单侧抽稀上限（cKDTree 建树规模）
DEFAULT_SEED = 0            # 抽稀随机种子（固定保证可复现）


def subsample_points(cloud: np.ndarray, max_points: int = DEFAULT_MAX_POINTS,
                     seed: int = DEFAULT_SEED) -> np.ndarray:
    """
    固定种子随机抽稀（无放回）；点数不超上限时原样返回.

    Args:
        cloud: (N, 3) 点云 [m].
        max_points: 抽稀上限.
        seed: 随机种子（同 N 同 seed 结果一致，可复现）.

    Returns
    -------
        (M, 3) 抽稀后点云，M = min(N, max_points).

    """
    n = int(cloud.shape[0])
    if n <= max_points:
        return cloud
    rng = np.random.default_rng(seed)
    idx = rng.choice(n, size=int(max_points), replace=False)
    return cloud[idx]


def cloud_centroid(cloud: np.ndarray) -> Optional[np.ndarray]:
    """
    点云质心 [m].

    Args:
        cloud: (N, 3) 点云.

    Returns
    -------
        (3,) float64 质心；空云给 None.

    """
    if cloud is None or np.asarray(cloud).size == 0:
        return None
    return np.asarray(cloud, dtype=np.float64).reshape(-1, 3).mean(axis=0)


def nn_distance_stats_mm(cloud_a: np.ndarray, cloud_b: np.ndarray,
                         max_points: int = DEFAULT_MAX_POINTS,
                         seed: int = DEFAULT_SEED) -> Optional[dict]:
    """
    两朵点云抽稀后的最近邻距离统计（a→b 单向），单位 [mm].

    Args:
        cloud_a: (N, 3) 查询侧点云 [m].
        cloud_b: (M, 3) 建树侧点云 [m].
        max_points: 两侧各自抽稀上限.
        seed: 抽稀随机种子.

    Returns
    -------
        {'mean_mm', 'median_mm', 'p95_mm'}；任一侧为空给 None.

    """
    a = subsample_points(np.asarray(cloud_a), max_points, seed)
    b = subsample_points(np.asarray(cloud_b), max_points, seed)
    if a.size == 0 or b.size == 0:
        return None
    dist, _ = cKDTree(b).query(a, k=1)
    return {
        'mean_mm': float(np.mean(dist)) * 1000.0,
        'median_mm': float(np.median(dist)) * 1000.0,
        'p95_mm': float(np.percentile(dist, 95)) * 1000.0,
    }


def assembly_overlap_metrics(frames: List,
                             max_points: int = DEFAULT_MAX_POINTS,
                             seed: int = DEFAULT_SEED) -> dict:
    """
    已采帧列表 → 重叠度指标 dict（finalize 时调用）.

    Args:
        frames: CapturedFrame 列表（读各帧 cloud_base，[m]）.
        max_points: 单侧抽稀上限.
        seed: 抽稀随机种子.

    Returns
    -------
        dict，键：
        - ``pairs``：相邻帧统计列表 [{'i', 'mean_mm', 'median_mm',
          'p95_mm'}]，i 为对中较后帧的下标（对 = i-1 与 i）；不足 2 帧为空
        - ``frame_centroids_base``：每帧质心 [m] 列表（空云帧给 None）
        - ``centroid_base``：装配总质心 [m]；无任何点给 None

    """
    clouds = [f.cloud_base for f in frames]
    pairs = []
    for i in range(1, len(clouds)):
        stats = nn_distance_stats_mm(clouds[i - 1], clouds[i],
                                     max_points=max_points, seed=seed)
        if stats is None:
            continue
        pairs.append({'i': i, **stats})
    centroids = []
    for cloud in clouds:
        c = cloud_centroid(cloud)
        centroids.append(None if c is None else [float(v) for v in c])
    valid = [np.asarray(c) for c in clouds
             if c is not None and np.asarray(c).size]
    assembly_centroid = cloud_centroid(np.vstack(valid)) if valid else None
    return {
        'pairs': pairs,
        'frame_centroids_base': centroids,
        'centroid_base': (None if assembly_centroid is None
                          else [float(v) for v in assembly_centroid]),
    }


def summarize_pairs_mm(pairs: List) -> Optional[dict]:
    """
    聚合相邻对统计为一句话指标：mean 取各对平均，p95 取最差对.

    Args:
        pairs: assembly_overlap_metrics 返回的 pairs 列表.

    Returns
    -------
        {'mean_mm', 'p95_mm'}；空列表给 None.

    """
    if not pairs:
        return None
    return {
        'mean_mm': float(np.mean([p['mean_mm'] for p in pairs])),
        'p95_mm': float(np.max([p['p95_mm'] for p in pairs])),
    }


# === view_coverage.py ===

def _angle_deg(first, second):
    """两个单位方向的夹角 [deg]；退化输入按 90°（零向量点积的等价值）."""
    angle = angle_between_deg(first, second)
    return 90.0 if angle is None else angle


def summarize_view_coverage(frames, target_center, cluster_angle_deg=5.0):
    """
    汇总目标到相机的观察方向覆盖与逐机位质量.

    同一机位停留期间会连续接受多帧：按帧计算会让兄弟帧互为最近邻
    （≈0°），view_count 虚高、mean_nearest_baseline 被稀释成接近 0。
    这里先按「目标→相机」方向贪心聚类（夹角 ≤ cluster_angle_deg 同机位，
    代表方向取成员均值），view_count/基线/分布指标按机位代表计算，
    反映真实机位分布；原始总帧数放在 frame_count 供参考。

    Args:
        frames: 采帧列表（需含 camera_position_base/stamp/valid_depth_ratio）.
        target_center: (3,) 目标中心（base 系 [m]）；None 返回 invalid.
        cluster_angle_deg: 机位聚类角阈值（默认 5°）.

    Returns
    -------
        dict：valid/view_count(机位数)/max_baseline_deg/
        mean_nearest_baseline_deg/range 与深度统计/views(机位代表列表)。

    """
    if target_center is None:
        return {
            'valid': False,
            'reason': 'target_center_unavailable',
            'view_count': 0,
            'frame_count': len(frames),
            'views': [],
        }
    center = np.asarray(target_center, dtype=np.float64).reshape(3)
    frame_views = []
    for index, frame in enumerate(frames):
        position = np.asarray(frame.camera_position_base,
                              dtype=np.float64).reshape(3)
        offset = position - center
        direction = _safe_unit(offset)
        if direction is None:
            continue
        frame_views.append({
            'index': index,
            'stamp_sec': float(frame.stamp),
            'position': position,
            'direction': direction,
            'range_m': float(np.linalg.norm(offset)),
            'valid_depth_ratio': float(frame.valid_depth_ratio),
            'registration': dict(frame.registration),
            'diagnostic_flags': list(frame.diagnostic_flags),
        })
    if not frame_views:
        return {
            'valid': False,
            'reason': 'no_valid_camera_direction',
            'view_count': 0,
            'frame_count': len(frames),
            'views': [],
        }

    # 按时间序贪心聚类：与既有机位代表方向夹角 ≤ 阈值即并入该机位
    clusters = []  # [{'rep': 单位方向, 'members': [frame_view, ...]}]
    for view in frame_views:
        for cluster in clusters:
            if _angle_deg(view['direction'], cluster['rep']) <= cluster_angle_deg:
                cluster['members'].append(view)
                merged = np.sum(
                    [m['direction'] for m in cluster['members']], axis=0)
                cluster['rep'] = _safe_unit(merged)
                break
        else:
            clusters.append({'rep': view['direction'], 'members': [view]})

    views = []
    directions = []
    for cluster in clusters:
        members = cluster['members']
        mean_position = np.mean([m['position'] for m in members], axis=0)
        first = members[0]
        views.append({
            'index': first['index'],
            'stamp_sec': first['stamp_sec'],
            'camera_position_base': [float(v) for v in mean_position],
            'direction_target_to_camera': [float(v) for v in cluster['rep']],
            'range_m': float(np.mean([m['range_m'] for m in members])),
            'valid_depth_ratio': float(np.mean(
                [m['valid_depth_ratio'] for m in members])),
            'frame_count': len(members),
            'member_indices': [int(m['index']) for m in members],
            'registration': first['registration'],
            'diagnostic_flags': first['diagnostic_flags'],
        })
        directions.append(cluster['rep'])

    pair_angles = []
    nearest_angles = []
    for i, direction in enumerate(directions):
        distances = [
            _angle_deg(direction, other)
            for j, other in enumerate(directions) if i != j
        ]
        if distances:
            nearest_angles.append(min(distances))
            pair_angles.extend(distances)
    ranges = [item['range_m'] for item in views]
    depth_ratios = [item['valid_depth_ratio'] for item in views]
    return {
        'valid': True,
        'reason': 'ok',
        'view_count': len(views),
        'frame_count': len(frame_views),
        'max_baseline_deg': (0.0 if not pair_angles else max(pair_angles)),
        'mean_nearest_baseline_deg': (
            0.0 if not nearest_angles else float(np.mean(nearest_angles))),
        'range_min_m': min(ranges),
        'range_max_m': max(ranges),
        'range_mean_m': float(np.mean(ranges)),
        'valid_depth_ratio_mean': float(np.mean(depth_ratios)),
        'valid_depth_ratio_min': min(depth_ratios),
        'views': views,
    }
