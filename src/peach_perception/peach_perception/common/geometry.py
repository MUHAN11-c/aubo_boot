from __future__ import annotations
"""几何原语：拟合、深度单位、TF 纯函数。"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from tf_transformations import (
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)


# === fitting.py ===

# ═══════════════════════════════════════════════════════════════
# 法线估计（有序深度图邻域叉积）
# ═══════════════════════════════════════════════════════════════

def estimate_normals(depth_roi: np.ndarray, xoff: int, yoff: int, K: dict,
                     depth_jump_mm: int = 30) -> Tuple[np.ndarray, np.ndarray]:
    """
    有序深度图 → 单位法线图 (h, w, 3)，统一朝向相机.

    理论: 表面切向量 ≈ 相邻像素 3D 点差，法线 = 两切向叉积。
    深度突变（遮挡边缘）处法线不可靠，置无效。

    Args:
        depth_roi: (h, w) uint16 深度（毫米）.
        xoff: ROI 在全图中的 x 像素偏移.
        yoff: ROI 在全图中的 y 像素偏移.
        K: 相机内参 {"fx","fy","cx","cy"}（像素单位）.
        depth_jump_mm: 相邻像素深度差超过该值（毫米）视为边缘，法线置无效.

    Returns
    -------
        (normals, normal_valid)：(h, w, 3) float64 单位法线（相机系）与
        (h, w) bool 有效标记（图边缘一圈恒无效）.

    """
    h, w = depth_roi.shape
    z = depth_roi.astype(np.float64) / 1000.0
    valid = (depth_roi > 0) & (depth_roi < 65535)

    us = np.arange(w)[None, :] + xoff
    vs = np.arange(h)[:, None] + yoff
    X = (us - K['cx']) * z / K['fx']
    Y = (vs - K['cy']) * z / K['fy']

    # 中心差分切向量（边缘退化为前/后向差分，numpy 自动处理）
    du = np.zeros((h, w, 3))
    dv = np.zeros((h, w, 3))
    du[:, 1:-1, 0] = X[:, 2:] - X[:, :-2]
    du[:, 1:-1, 2] = z[:, 2:] - z[:, :-2]
    dv[1:-1, :, 1] = Y[2:, :] - Y[:-2, :]
    dv[1:-1, :, 2] = z[2:, :] - z[:-2, :]

    n = np.cross(du, dv)
    norm = np.linalg.norm(n, axis=2)
    with np.errstate(invalid='ignore', divide='ignore'):
        n = n / np.where(norm > 1e-12, norm, 1.0)[..., None]

    # 朝向相机（相机在原点看 +Z，可见面法线应指向原点）
    dot = n[..., 0] * X + n[..., 1] * Y + n[..., 2] * z
    flip = dot > 0
    n[flip] = -n[flip]

    # 邻域有效 + 深度连续 + 范数正常
    jump_u = np.zeros((h, w), bool)
    jump_v = np.zeros((h, w), bool)
    jump_u[:, 1:-1] = (np.abs(depth_roi[:, 2:].astype(np.int32)
                              - depth_roi[:, :-2].astype(np.int32)) > depth_jump_mm)
    jump_v[1:-1, :] = (np.abs(depth_roi[2:, :].astype(np.int32)
                              - depth_roi[:-2, :].astype(np.int32)) > depth_jump_mm)
    nvalid = valid & (norm > 1e-12) & ~jump_u & ~jump_v
    nvalid[[0, -1], :] = False
    nvalid[:, [0, -1]] = False
    return n, nvalid


# ═══════════════════════════════════════════════════════════════
# 球拟合: 点+法线 RANSAC + 几何 LM 抛光
# ═══════════════════════════════════════════════════════════════

def _sphere_inliers(points: np.ndarray, center: np.ndarray, radius: float,
                    thresh: float) -> np.ndarray:
    """
    球面内点索引：|‖p−c‖ − r| ≤ thresh.

    Args:
        points: (N, 3) 点（米）.
        center: (3,) 球心（米）.
        radius: 半径（米）.
        thresh: 径向残差阈值（米）.

    Returns
    -------
        内点下标 int 数组.

    """
    d = np.linalg.norm(points - center, axis=1)
    return np.where(np.abs(d - radius) <= thresh)[0]


def ransac_sphere(points: np.ndarray, normals: Optional[np.ndarray] = None,
                  radius_prior: Optional[float] = None,
                  radius_range: Tuple[float, float] = (0.025, 0.045),
                  thresh: float = 0.004, max_iter: int = 300,
                  seed: int = 0) -> Optional[dict]:
    """
    球 RANSAC：法线可用时 1–2 点采样，否则 4 点采样；半径夹紧.

    理论: c = p − r·n（射线约束）；半径已知 → 最小采样 1 点（迭代数 ~7）；
    半径未知 → 2 点+法线，r = |p₁−p₂|² / ((n₁−n₂)·(p₁−p₂))。

    Args:
        points: (N, 3) 点（米）；N<10 直接 None.
        normals: (N, 3) 单位法线；None 或含非有限值时退化 4 点采样.
        radius_prior: 半径先验（米）；给定时单点采样.
        radius_range: 接受的半径区间 (min, max)（米），区间外假设丢弃.
        thresh: 内点径向残差阈值（米）.
        max_iter: RANSAC 迭代次数.
        seed: 随机种子（保证可复现）.

    Returns
    -------
        dict(center, radius, inliers, inlier_ratio, rms)；内点 <10 给 None.

    """
    rng = np.random.default_rng(seed)
    n = len(points)
    if n < 10:
        return None
    use_normals = (normals is not None and len(normals) == n
                   and np.isfinite(normals).all())
    best = None
    for _ in range(max_iter):
        center = radius = None
        if use_normals and radius_prior is not None:
            i = rng.integers(n)
            center = points[i] - radius_prior * normals[i]
            radius = radius_prior
        elif use_normals:
            i, j = rng.choice(n, size=2, replace=False)
            dn = normals[i] - normals[j]
            dp = points[i] - points[j]
            denom = float(dn @ dp)
            if abs(denom) < 1e-9:
                continue
            radius = float(dp @ dp) / denom
            if not (radius_range[0] <= radius <= radius_range[1]):
                continue
            center = points[i] - radius * normals[i]
        else:
            idx = rng.choice(n, size=min(4, n), replace=False)
            pts = points[idx]
            a = np.column_stack((2.0 * pts, np.ones(len(pts))))
            b = np.einsum('ij,ij->i', pts, pts)
            sol, *_ = np.linalg.lstsq(a, b, rcond=None)
            t = sol[3] + float(sol[:3] @ sol[:3])
            if t <= 0:
                continue
            center, radius = sol[:3], float(np.sqrt(t))
            if not (radius_range[0] <= radius <= radius_range[1]):
                continue

        inl = _sphere_inliers(points, center, radius, thresh)
        if best is None or len(inl) > len(best['inliers']):
            best = {'center': center, 'radius': radius, 'inliers': inl}

    if best is None or len(best['inliers']) < 10:
        return None
    d = np.linalg.norm(points[best['inliers']] - best['center'], axis=1)
    best['inlier_ratio'] = len(best['inliers']) / n
    best['rms'] = float(np.sqrt(np.mean((d - best['radius']) ** 2)))
    return best


def polish_sphere_lm(points: np.ndarray, center0: np.ndarray,
                     radius: float, fixed_radius: bool = True) -> Tuple[np.ndarray, float]:
    """
    几何正交距离 LM 抛光（MLE）。fixed_radius=True 时只估 3 DOF 球心.

    理论: min Σ(||p_i−c|| − r)² 是各向同性高斯噪声的最大似然；固定半径
    删除 Fisher 信息矩阵最病态方向，CRLB 严格下降。

    Args:
        points: (N, 3) 内点（米）.
        center0: (3,) 初始球心（米）.
        radius: 半径（米）；fixed_radius 时固定为该值.
        fixed_radius: True 只估球心（3 DOF）；False 球心+半径联估（4 DOF）.

    Returns
    -------
        (center (3,), radius)：抛光后的球心与半径（米）.

    """
    from scipy.optimize import least_squares

    if fixed_radius:
        def resid(c):
            return np.linalg.norm(points - c, axis=1) - radius
        sol = least_squares(resid, center0, method='lm')
        return sol.x, radius

    def resid(cr):
        return np.linalg.norm(points - cr[:3], axis=1) - cr[3]
    sol = least_squares(resid, np.append(center0, radius), method='lm')
    return sol.x[:3], float(sol.x[3])


def fit_sphere_robust(points: np.ndarray, normals: Optional[np.ndarray] = None,
                      radius_prior: Optional[float] = 0.035,
                      radius_range: Tuple[float, float] = (0.025, 0.045),
                      thresh: float = 0.004, max_iter: int = 300,
                      seed: int = 0) -> Optional[dict]:
    """
    完整球拟合: RANSAC → 内点 LM 抛光 → 重计内点与统计量.

    Args:
        points: (N, 3) 点（米）.
        normals: (N, 3) 单位法线；None 退化 4 点采样.
        radius_prior: 半径先验（米）；None 表示半径自由（抛光亦放开）.
        radius_range: 接受的半径区间（米），抛光后越界判失败.
        thresh: 内点径向残差阈值（米）.
        max_iter: RANSAC 迭代次数.
        seed: 随机种子.

    Returns
    -------
        dict(center, radius, inliers, inlier_ratio, rms)；失败给 None.

    """
    est = ransac_sphere(points, normals, radius_prior, radius_range,
                        thresh, max_iter, seed)
    if est is None:
        return None
    inl_pts = points[est['inliers']]
    center, radius = polish_sphere_lm(
        inl_pts, est['center'], est['radius'],
        fixed_radius=radius_prior is not None)
    if not (radius_range[0] <= radius <= radius_range[1]):
        return None
    inl = _sphere_inliers(points, center, radius, thresh)
    if len(inl) < 10:
        return None
    d = np.linalg.norm(points[inl] - center, axis=1)
    return {'center': center, 'radius': radius, 'inliers': inl,
            'inlier_ratio': len(inl) / len(points),
            'rms': float(np.sqrt(np.mean((d - radius) ** 2)))}


# ═══════════════════════════════════════════════════════════════
# 圆柱拟合: 2 点+2 法线 RANSAC + Eberly 轴向抛光
# ═══════════════════════════════════════════════════════════════

def _cylinder_radial_dist(points: np.ndarray, q0: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """
    点到圆柱轴的径向距离.

    Args:
        points: (N, 3) 点（米）.
        q0: (3,) 轴上一点（米）.
        axis: (3,) 单位轴向.

    Returns
    -------
        (N,) 径向距离（米）.

    """
    rel = points - q0
    perp = rel - np.outer(rel @ axis, axis)
    return np.linalg.norm(perp, axis=1)


def ransac_cylinder(points: np.ndarray, normals: np.ndarray,
                    radius_range: Tuple[float, float] = (0.025, 0.050),
                    thresh: float = 0.0035, max_iter: int = 300,
                    seed: int = 0) -> Optional[dict]:
    """
    圆柱 RANSAC（PCL SACMODEL_CYLINDER 同款构造，clean-room 实现）.

    理论: 圆柱法线 ⊥ 轴 → a = (n₁×n₂)/|n₁×n₂|；投影 ⊥a 平面退化为 2D 圆，
    圆心 = 两射线 p'₁+α·n'₁ 与 p'₂+β·n'₂ 的交点；半径夹紧剪掉退化假设。

    Args:
        points: (N, 3) 点（米）；N<20 直接 None.
        normals: (N, 3) 单位法线；None 或数量不符直接 None.
        radius_range: 接受的半径区间 (min, max)（米），区间外假设丢弃.
        thresh: 内点径向残差阈值（米）.
        max_iter: RANSAC 迭代次数.
        seed: 随机种子.

    Returns
    -------
        dict(axis, q0, radius, inliers, inlier_ratio, rms)；内点 <20 给 None.

    """
    rng = np.random.default_rng(seed)
    n = len(points)
    if n < 20 or normals is None or len(normals) != n:
        return None
    best = None
    for _ in range(max_iter):
        i, j = rng.choice(n, size=2, replace=False)
        n1, n2 = normals[i], normals[j]
        a = np.cross(n1, n2)
        la = np.linalg.norm(a)
        if la < 0.17:  # sin(10°)：近平行法线无法定轴
            continue
        a = a / la
        p1p = points[i] - (points[i] @ a) * a
        p2p = points[j] - (points[j] @ a) * a
        n1p = n1 - (n1 @ a) * a
        n2p = n2 - (n2 @ a) * a
        m = np.column_stack((n1p, -n2p))  # 3x2
        # 最小二乘解 2 维射线参数（两射线一般异面，取最近点中点）
        alpha_beta, *_ = np.linalg.lstsq(m, p2p - p1p, rcond=None)
        c1 = p1p + alpha_beta[0] * n1p
        c2 = p2p + alpha_beta[1] * n2p
        center2d = 0.5 * (c1 + c2)
        radius = 0.5 * (np.linalg.norm(p1p - center2d) + np.linalg.norm(p2p - center2d))
        if not (radius_range[0] <= radius <= radius_range[1]):
            continue

        d = _cylinder_radial_dist(points, center2d, a)
        inl = np.where(np.abs(d - radius) <= thresh)[0]
        if best is None or len(inl) > len(best['inliers']):
            best = {'axis': a, 'q0': center2d, 'radius': radius, 'inliers': inl}

    if best is None or len(best['inliers']) < 20:
        return None
    d = _cylinder_radial_dist(points[best['inliers']], best['q0'], best['axis'])
    best['inlier_ratio'] = len(best['inliers']) / n
    best['rms'] = float(np.sqrt(np.mean((d - best['radius']) ** 2)))
    return best


# ── Eberly 圆柱抛光（BSD 3-Clause, 改编自 xingjiepan/cylinder_fitting）──

def _eberly_direction(theta: float, phi: float) -> np.ndarray:
    """
    球坐标 (theta, phi) → 单位方向向量（Eberly 参数化）.

    Args:
        theta: 与 +Z 的极角 (rad).
        phi: 绕 Z 的方位角 (rad).

    Returns
    -------
        (3,) 单位向量.

    """
    return np.array([np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta),
                     np.cos(theta)])


def _eberly_G(w: np.ndarray, X: np.ndarray) -> float:
    """
    Eberly 目标函数 G(w)：方向 w 下的代数残差能量.

    Args:
        w: (3,) 单位轴向.
        X: (N, 3) 已去均值点（米）.

    Returns
    -------
        能量标量；退化（分母≈0）给 inf.

    """
    P = np.eye(3) - np.outer(w, w)
    Y = X @ P.T
    A = Y.T @ Y
    S = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    A_hat = S @ A @ S.T
    u = float(np.mean(np.einsum('ij,ij->i', Y, Y)))
    num = A_hat @ (np.einsum('ij,ij->i', Y, Y)[:, None] * Y).sum(axis=0)
    den = np.trace(A_hat @ A)
    if abs(den) < 1e-18:
        return float('inf')
    v = num / den
    return float(np.sum((np.einsum('ij,ij->i', Y, Y) - u - 2.0 * Y @ v) ** 2))


def _eberly_center(w: np.ndarray, X: np.ndarray) -> np.ndarray:
    """
    Eberly 闭式轴心 C(w)（消元结果，见 cylinder_fitting/fitting.py）.

    Args:
        w: (3,) 单位轴向.
        X: (N, 3) 已去均值点（米）.

    Returns
    -------
        (3,) 轴心（去均值坐标系下；分母退化时给零向量）.

    """
    P = np.eye(3) - np.outer(w, w)
    Y = X @ P.T
    A = Y.T @ Y
    S = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    A_hat = S @ A @ S.T
    num = A_hat @ (np.einsum('ij,ij->i', Y, Y)[:, None] * Y).sum(axis=0)
    den = np.trace(A_hat @ A)
    if abs(den) < 1e-18:
        return np.zeros(3)
    return num / den


def polish_cylinder_axis(points: np.ndarray,
                         axis_hint: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Eberly 轴向抛光：5 维消元为 2 维，以 axis_hint 为起点 Powell 优化.

    Args:
        points: (N, 3) 点（米）.
        axis_hint: (3,) 初始轴向（内部归一化；结果与之同向，防符号翻转）.

    Returns
    -------
        (axis, q0)：(3,) 单位轴向与 (3,) 轴上一点（米）.

    """
    from scipy.optimize import minimize

    t = points.mean(axis=0)
    X = points - t
    hint = axis_hint / np.linalg.norm(axis_hint)
    theta0 = float(np.arccos(np.clip(hint[2], -1.0, 1.0)))
    phi0 = float(np.arctan2(hint[1], hint[0]))
    starts = [(theta0, phi0), (0.0, 0.0),
              (np.pi / 2, 0.0), (np.pi / 2, np.pi / 2)]
    best_w, best_g = None, float('inf')
    for sp in starts:
        res = minimize(lambda x: _eberly_G(_eberly_direction(x[0], x[1]), X),
                       sp, method='Powell', tol=1e-6)
        if res.fun < best_g:
            best_g, best_w = res.fun, _eberly_direction(res.x[0], res.x[1])
    if best_w is None:
        return hint, t
    # 与 hint 同向，避免符号翻转
    if best_w @ hint < 0:
        best_w = -best_w
    # 轴上一点 = Eberly 闭式轴心（平移回原坐标系）
    q0 = _eberly_center(best_w, X) + t
    return best_w, q0


def fit_cylinder_robust(points: np.ndarray, normals: np.ndarray,
                        radius_range: Tuple[float, float] = (0.025, 0.050),
                        thresh: float = 0.0035, max_iter: int = 300,
                        polish: bool = True, seed: int = 0) -> Optional[dict]:
    """
    完整圆柱拟合: RANSAC → Eberly 轴向抛光 → 半径几何重估 → 重计统计量.

    半径用 mean(d_i) 重估（Eberly 代数残差导致 ~1.9mm 系统性半径偏差，
    轴向不受此影响，见 references/notes_fitting_algorithms.md §2.2）。

    Args:
        points: (N, 3) 点（米）.
        normals: (N, 3) 单位法线.
        radius_range: 接受的半径区间 (min, max)（米）.
        thresh: 内点径向残差阈值（米）.
        max_iter: RANSAC 迭代次数.
        polish: True 时做 Eberly 轴向抛光（内点抽稀到 800）.
        seed: 随机种子.

    Returns
    -------
        dict(axis, q0, radius, inliers, inlier_ratio, rms)；失败给 None.

    """
    est = ransac_cylinder(points, normals, radius_range, thresh, max_iter, seed)
    if est is None:
        return None
    inl = est['inliers']
    axis, q0 = est['axis'], est['q0']
    if polish and len(inl) >= 20:
        # 抛光代价随点数线性增长，内点抽稀到 800（精度损失可忽略）
        polish_idx = inl if len(inl) <= 800 else inl[np.linspace(
            0, len(inl) - 1, 800, dtype=int)]
        axis, q0 = polish_cylinder_axis(points[polish_idx], axis)
        # 重选内点（轴向更新后）
        d = _cylinder_radial_dist(points, q0, axis)
        radius = float(np.median(d[inl]))
        inl = np.where(np.abs(d - radius) <= thresh)[0]
        if len(inl) < 20:
            return None
    d = _cylinder_radial_dist(points[inl], q0, axis)
    radius = float(np.mean(d))
    rms = float(np.sqrt(np.mean((d - radius) ** 2)))
    return {'axis': axis, 'q0': q0, 'radius': radius, 'inliers': inl,
            'inlier_ratio': len(inl) / len(points), 'rms': rms}


# === depth_geometry.py ===

_UINT16_MAX_MM = 65535.0  # uint16 深度上限（毫米）；饱和值下游视为无效


def normalize_depth_to_uint16_mm(depth: np.ndarray,
                                 depth_scale_unit: float) -> np.ndarray:
    """
    深度图归一化为 uint16 毫米（管线统一输入约定）.

    uint16 路径：``raw × depth_scale_unit`` = 毫米（Percipio 常见 0.25；
    数据集回放的真毫米深度设 1.0），round + clip 到 [0, 65535]；
    scale==1.0 时原样返回（零拷贝）。

    浮点路径（32FC1 等）：输入视为「米」，×1000 转毫米，depth_scale_unit
    不生效；NaN/±Inf/负值一律置 0（无效深度），round + clip 到 uint16。

    Args:
        depth: (H, W) 深度图，uint16（原始值）或浮点（米）.
        depth_scale_unit: uint16 路径的比例因子（毫米/单位）；浮点路径忽略.

    Returns
    -------
        (H, W) uint16 深度，单位毫米.

    Raises
    ------
        ValueError: 不支持的 dtype（既非 uint16 也非浮点）.

    """
    if depth.dtype == np.uint16:
        if abs(depth_scale_unit - 1.0) > 1e-9:
            return np.clip(
                np.round(depth.astype(np.float32) * depth_scale_unit),
                0.0, _UINT16_MAX_MM).astype(np.uint16)
        return depth
    if np.issubdtype(depth.dtype, np.floating):
        mm = depth.astype(np.float64) * 1000.0
        mm = np.where(np.isfinite(mm) & (mm > 0.0), mm, 0.0)
        return np.clip(np.round(mm), 0.0, _UINT16_MAX_MM).astype(np.uint16)
    raise ValueError(f'不支持的深度 dtype {depth.dtype}（仅支持 uint16/浮点）')


# === tf_utils.py ===

@dataclass(frozen=True)
class QuaternionValue:
    """
    单位四元数不可变值对象（geometry_msgs/Quaternion 的纯核替代）.

    纯核不得 import geometry_msgs，故 rotation_to_quat 返回本值对象；
    字段语义与消息一致 (x, y, z, w)。编排层需要消息时自行构造：
    ``Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)``。
    """

    x: float
    y: float
    z: float
    w: float

    def as_tuple(self) -> tuple:
        """返回 (x, y, z, w) 元组，供 tf_transformations 等数组接口使用."""
        return (self.x, self.y, self.z, self.w)


def transform_msg_to_matrix(transform) -> np.ndarray:
    """
    Transform（鸭子类型）→ 4×4 齐次矩阵 T（p_out = R@p_in + t）.

    官方 tf_transformations 组合：translation_matrix @ quaternion_matrix
    （后者内部按模长归一化，非单位四元数输入也安全）。统一自原
    peach_perception.scene_perception.tf_utils._transform_msg_to_matrix 与
    peach_perception.target_reconstruction.tf_utils.transform_msg_to_matrix（两者
    数值等价：quaternion_matrix 不写平移列）。

    Args:
        transform: 带 .translation.x/y/z 与 .rotation.x/y/z/w 的对象.

    Returns
    -------
        (4, 4) float64 齐次矩阵（平移单位随消息，通常为米）.

    """
    tr = transform.translation
    q = transform.rotation
    q_xyzw = (q.x, q.y, q.z, q.w)
    return translation_matrix((tr.x, tr.y, tr.z)) @ quaternion_matrix(q_xyzw)


def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    4×4 齐次矩阵求逆：T_camera_base = inv(T_base_camera).

    官方 np.linalg.inv（通用 4×4 求逆）：刚体矩阵上数值误差 ~1e-16，
    与手写 [R.T, -R.T@t] 在测试锚点精度（atol=1e-12）内无差别；
    输入的刚性由 test_tf_utils 正逆互反用例守门，无需自造刚体特化。
    （实现沿用原 peach_perception.target_reconstruction.tf_utils.invert_transform。）

    Args:
        T: (4, 4) 齐次矩阵.

    Returns
    -------
        (4, 4) float64 逆矩阵.

    """
    return np.linalg.inv(np.asarray(T, dtype=np.float64))


def relative_motion(T_a: np.ndarray, T_b: np.ndarray) -> tuple:
    """
    两个 base←camera 位姿间的相对运动量（视角过滤用）.

    保留 numpy 闭式（官方无等价物）：tf_transformations 没有「两旋转
    夹角」直出 API，须绕 quaternion_from_matrix → 2·arccos(|w|) 取角，
    反而多一次四元数往返；trace 闭式 R_rel→arccos((tr−1)/2) 是教科书
    标准式，单次矩阵乘即得。concatenate_matrices 仅为矩阵乘语法糖，
    无语义收益，不用。
    （实现沿用原 peach_perception.target_reconstruction.tf_utils.relative_motion。）

    Args:
        T_a: (4, 4) 本帧位姿.
        T_b: (4, 4) 参考帧位姿（上一已采帧）.

    Returns
    -------
        (translation_m, rotation_deg)：平移差范数 [m] 与相对旋转角 [deg].

    """
    R_rel = T_a[:3, :3] @ T_b[:3, :3].T
    cos_angle = float(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
    rotation_deg = float(np.degrees(np.arccos(cos_angle)))
    translation_m = float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    return translation_m, rotation_deg


def rotation_to_quat(R: np.ndarray) -> QuaternionValue:
    """
    3×3 旋转矩阵 → 单位四元数值对象（官方 quaternion_from_matrix）.

    Args:
        R: (3, 3) 旋转矩阵（非正交输入的行为随官方实现，调用方保证刚性）.

    Returns
    -------
        QuaternionValue（x, y, z, w），模长为 1.

    """
    m4 = np.eye(4, dtype=float)
    m4[:3, :3] = np.asarray(R, dtype=float)
    q = quaternion_from_matrix(m4)            # numpy [x, y, z, w]
    return QuaternionValue(
        x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def transform_point(T: np.ndarray, point) -> np.ndarray:
    """
    点按齐次矩阵变换：p_out = R@p_in + t（None 透传）.

    Args:
        T: (4, 4) 齐次矩阵，输出系←输入系.
        point: (3,) 点坐标（米），None 原样返回.

    Returns
    -------
        (3,) float64 输出系点坐标；输入 None 时返回 None.

    """
    if point is None:
        return None
    return T[:3, :3] @ np.asarray(point, dtype=float) + T[:3, 3]


def transform_direction(T: np.ndarray, direction) -> np.ndarray:
    """
    方向向量按齐次矩阵变换：只乘 R 不加平移，并重新归一化（None 透传）.

    平移不影响方向；近零退化向量不归一化（防除零），原样返回旋转结果。

    Args:
        T: (4, 4) 齐次矩阵，输出系←输入系.
        direction: (3,) 方向向量，None 原样返回.

    Returns
    -------
        (3,) float64 输出系单位方向向量；输入 None 时返回 None.

    """
    if direction is None:
        return None
    d = T[:3, :3] @ np.asarray(direction, dtype=float)
    n = float(np.linalg.norm(d))
    return d / n if n > 1e-9 else d


def gravity_camera_from_R(R_out_cam: np.ndarray) -> np.ndarray:
    """
    由 output←camera 旋转反推相机系重力方向（gravity_mode='tf' 用）.

    约定 output_frame（如 base_link）内重力向量为 [0, 0, -1]（竖直向下）；
    方向向量只乘旋转、不加平移：g_cam = normalize(R_out_cam.T @ g_out)。
    （实现沿用原 peach_perception.scene_perception.tf_utils._gravity_camera_from_R。）

    Args:
        R_out_cam: (3, 3) 旋转矩阵，output_frame←相机系.

    Returns
    -------
        (3,) 相机系单位重力向量；退化（近零）时原样返回.

    """
    g = np.asarray(R_out_cam, dtype=float).T @ np.array([0.0, 0.0, -1.0])
    n = float(np.linalg.norm(g))
    return g / n if n > 1e-9 else g
