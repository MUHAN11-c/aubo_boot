"""
精化器 — ROI 累积点云按类别重拟合（复用 peach_pose_ros2 拟合原语）.

裸果（fruit）→ 球拟合定心定径；袋装桃（bag）→ 圆柱拟合定轴定径。
拟合原语直接 import peach_pose_ros2.peach_pose.fitting（install 空间可用）；
法线由 open3d 最近邻估计（仅 venv 可 import 本模块）。
"""
import numpy as np
import open3d as o3d
from peach_pose_ros2.peach_pose.fitting import (
    fit_cylinder_robust,
    fit_sphere_robust,
)


def estimate_normals_o3d(points: np.ndarray, k: int = 16) -> np.ndarray:
    """
    open3d 最近邻法线估计（统一朝向点云质心）.

    Args:
        points: (N, 3) 点（米）.
        k: KD 树近邻数.

    Returns
    -------
        (N, 3) 单位法线.

    """
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud.estimate_normals(
        o3d.geometry.KDTreeSearchParamKNN(knn=k))
    normals = np.asarray(cloud.normals)
    # 法线符号统一朝向点云质心（拟合原语对符号不敏感，仅求一致）
    centroid = points.mean(axis=0)
    flip = np.einsum('ij,ij->i', normals, centroid - points) < 0
    normals[flip] = -normals[flip]
    return normals


def refine_target(points: np.ndarray, kind: str,
                  radius_range=(0.025, 0.045),
                  thresh: float = 0.005) -> dict:
    """
    ROI 点云按类别重拟合.

    Args:
        points: (N, 3) ROI 累积点（输出系，米）；N<30 直接失败.
        kind: 'fruit' → 球拟合（球心+半径）；'bag' → 圆柱拟合（轴+半径）.
        radius_range: 半径接受区间 (m)（桃 Ø50–90 mm / 袋半径同量级）.
        thresh: RANSAC 内点残差阈值 (m)（累积云噪声略大，默认 5 mm）.

    Returns
    -------
        fruit: dict(center(3,), radius, inlier_ratio, rms)；
        bag: dict(center(3,), axis(3,), radius, inlier_ratio, rms)；
        拟合失败给 None.

    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts) < 30:
        return None
    normals = estimate_normals_o3d(pts)
    if kind == 'fruit':
        est = fit_sphere_robust(pts, normals, radius_prior=None,
                                radius_range=radius_range, thresh=thresh)
        if est is None:
            return None
        return {'center': est['center'], 'radius': float(est['radius']),
                'inlier_ratio': float(est['inlier_ratio']),
                'rms': float(est['rms'])}
    est = fit_cylinder_robust(pts, normals, radius_range=radius_range,
                              thresh=thresh)
    if est is None:
        return None
    return {'center': np.asarray(est['q0']), 'axis': np.asarray(est['axis']),
            'radius': float(est['radius']),
            'inlier_ratio': float(est['inlier_ratio']),
            'rms': float(est['rms'])}
