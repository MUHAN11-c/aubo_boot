"""
桃子球体拟合 — 仅作可视化参考，不参与安全管线.

成熟桃子尺寸一致性好，对检测框内的实测深度点做点+法线 RANSAC 球拟合 +
几何 LM 抛光，为人工验收提供直观的尺寸/位置 sanity check。拟合结果只用于
3D 线框与卡片展示，**不得**进入袋径、入口点、行程、工具净空或 ACCEPT 判定
（安全输出仍只来自 pipeline.py 的实测深度几何）。

理论要点（原设计文档已随旧项目移除，要点记录于此）:
  - 球面法线 n = (p−c)/r → c = p − r·n（射线约束）：2 点+法线采样求半径，
    半径夹紧剪掉退化假设，比 4 点采样迭代数与方差同步下降；
  - 内点再做几何正交距离 LM 抛光（各向同性噪声下的 MLE）。
实现位于 peach_pose/fitting.py。
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from ..fitting import fit_sphere_robust


def fit_sphere_reference(points: np.ndarray,
                         normals: Optional[np.ndarray] = None,
                         radius_range: Tuple[float, float] = (0.025, 0.045),
                         min_points: int = 30) -> Tuple[Optional[np.ndarray], float]:
    """
    点+法线 RANSAC 球拟合 + 几何抛光 (可视化参考).

    Args:
        points: (N, 3) 相机系实测深度点，单位 m
        normals: (N, 3) 与 points 对齐的单位法线（可为 None，退化为 4 点采样）
        radius_range: 接受的半径区间 (m)，成熟桃子约 25–45mm 半径
        min_points: 最少点数，不足直接放弃

    Returns
    -------
    (center(3,) 或 None, radius_m)；失败时 center 为 None、radius 为 0

    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < min_points:
        return None, 0.0
    est = fit_sphere_robust(pts, normals, radius_prior=None,
                            radius_range=radius_range)
    if est is None or est['inlier_ratio'] < 0.3:
        return None, 0.0
    # 球心应落在点云附近，防止拟合飘到遮挡物后方
    if np.linalg.norm(est['center'] - np.median(pts, axis=0)) > 0.15:
        return None, 0.0
    return est['center'], est['radius']
