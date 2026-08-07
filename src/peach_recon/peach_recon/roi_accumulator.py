"""
ROI 累积器 — Open3D ScalableTSDFVolume 三维重建内核.

每帧对齐 RGB-D + 相机位姿 → volume.integrate。注意：tf2
lookup_transform(base, cam) 返回 cam→world，而 Open3D integrate 的
extrinsic 实为 world→camera（合成实验验证，勿删 inv）；extract_cloud()
提取 numpy 点+颜色；reset() 重建 volume。仅 venv 可 import。
"""
import numpy as np
import open3d as o3d


class TsdfRecon:
    """TSDF 重建器：RGB-D 逐帧融合 → 提取彩色点云（仅 venv，open3d）."""

    def __init__(self, tsdf_voxel_length: float = 0.005,
                 sdf_trunc: float = 0.02, depth_scale: float = 1000.0,
                 depth_max: float = 3.0):
        """
        构造重建器.

        Args:
            tsdf_voxel_length: TSDF 体素边长 (m)（5 mm 级，桃级细节足够）.
            sdf_trunc: 截断距离 (m)（≥4×voxel，薄表面不被吃掉）.
            depth_scale: 深度比例（uint16 毫米 → 米，恒 1000）.
            depth_max: 融合深度上限 (m)（远处背景不入图）.

        Returns
        -------
            无返回值（None）.

        """
        self.tsdf_voxel_length = float(tsdf_voxel_length)
        self.sdf_trunc = float(sdf_trunc)
        self.depth_scale = float(depth_scale)
        self.depth_max = float(depth_max)
        self._volume = self._new_volume()

    def _new_volume(self) -> o3d.pipelines.integration.ScalableTSDFVolume:
        """新建 ScalableTSDFVolume（RGB8 真彩）."""
        return o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.tsdf_voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    def integrate_rgbd(self, rgb_bgr: np.ndarray, depth_mm: np.ndarray,
                       K: dict, T_map_cam: np.ndarray) -> None:
        """
        融合一帧对齐 RGB-D.

        Args:
            rgb_bgr: (H, W, 3) uint8 BGR 图（OpenCV 惯例，内部转 RGB）.
            depth_mm: (H, W) uint16 深度（毫米，与彩图对齐）.
            K: 相机内参 {"fx","fy","cx","cy","width","height"}（像素）.
            T_map_cam: (4, 4) 相机位姿（输出系←相机系；内部取逆给 Open3D）.

        Returns
        -------
            无返回值（None）.

        """
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            int(K['width']), int(K['height']),
            float(K['fx']), float(K['fy']), float(K['cx']), float(K['cy']))
        color_rgb = rgb_bgr[:, :, ::-1].copy()       # BGR → RGB（真彩融合）
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.ascontiguousarray(color_rgb, dtype=np.uint8)),
            o3d.geometry.Image(np.ascontiguousarray(depth_mm, dtype=np.uint16)),
            depth_scale=self.depth_scale, depth_trunc=self.depth_max,
            convert_rgb_to_intensity=False)
        # Open3D integrate 的 extrinsic 实为 world→camera（合成实验验证：
        # 传 cam→world 几何落错位置；取逆后墙面正确落在世界坐标）。
        # tf2 lookup_transform(base, cam) 返回 cam→world，故必须取逆
        self._volume.integrate(rgbd, intrinsic, np.linalg.inv(T_map_cam))

    def extract_cloud(self):
        """
        提取 TSDF 点云.

        Returns
        -------
            (xyz (N,3) float64 米, rgb (N,3) float64 [0,1])；空云两个空数组.

        """
        pcd = self._volume.extract_point_cloud()
        xyz = np.asarray(pcd.points, dtype=np.float64).reshape(-1, 3)
        if pcd.has_colors():
            rgb = np.asarray(pcd.colors, dtype=np.float64).reshape(-1, 3)
        else:
            rgb = np.zeros((0, 3), dtype=np.float64)
        return xyz, rgb

    @property
    def n_points(self) -> int:
        """当前提取点数."""
        return len(self._volume.extract_point_cloud().points)

    def reset(self) -> None:
        """重建 TSDF volume（清空全部融合）."""
        self._volume = self._new_volume()
