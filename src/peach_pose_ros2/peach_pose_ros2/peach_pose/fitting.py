"""鲁棒几何拟合 — 法线估计 / 球 RANSAC / 圆柱 RANSAC（实测深度派生量）。

管线位置:
  剪切抓取核心几何层。为 pipeline.py（袋轴/袋径/entry_start）与
  inspector/sphere_ref.py（桃子球参考）提供拟合原语。

核心理论 (详见 docs/grasp_axis_entry_design.md 附A/附B):
  1. 球面法线 n = (p−c)/r → c = p − r·n（射线约束）：半径固定时最小采样
     1 点+法线；RANSAC 迭代数 N = log(1−p)/log(1−w^k)，k=1 → 7 次。
  2. 固定半径删掉 Fisher 信息矩阵中最病态方向（小球冠条件数 O(α⁻⁴)→O(α⁻²)）。
  3. 圆柱面法线 ⊥ 轴线：两不平行法线叉积即轴方向 a=(n₁×n₂)/|n₁×n₂|；
     投影 ⊥a 平面后 3D 圆柱退化 2D 圆，圆心=两射线交点。
  4. Eberly 圆柱拟合把 5 维问题消元为 2 维（轴向抛光），其代数残差导致
     半径系统性偏差，故半径另做几何重估 mean(d_i)。

法线说明:
  法线由有序深度图邻域叉积估计，是实测深度的一阶派生量，不违反
  "只用传感器实测深度"铁律。

Eberly 抛光改编自 xingjiepan/cylinder_fitting (BSD 3-Clause,
https://github.com/xingjiepan/cylinder_fitting, 见 references/github/)，
实现了 David Eberly "Fitting 3D Data with a Cylinder"。
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


# ═══════════════════════════════════════════════════════════════
# 法线估计（有序深度图邻域叉积）
# ═══════════════════════════════════════════════════════════════

def estimate_normals(depth_roi: np.ndarray, xoff: int, yoff: int, K: dict,
                     depth_jump_mm: int = 30) -> Tuple[np.ndarray, np.ndarray]:
    """有序深度图 → 单位法线图 (h, w, 3)，统一朝向相机。

    理论: 表面切向量 ≈ 相邻像素 3D 点差，法线 = 两切向叉积。
    深度突变（遮挡边缘）处法线不可靠，置无效。

    Args:
        depth_roi: (h, w) uint16 深度 (mm)
        xoff, yoff: ROI 在全图中的像素偏移
        K: 相机内参 {"fx","fy","cx","cy"}
        depth_jump_mm: 相邻像素深度差超过该值视为边缘（法线无效）

    Returns:
        (normals (h,w,3) float64, normal_valid (h,w) bool)
    """
    h, w = depth_roi.shape
    z = depth_roi.astype(np.float64) / 1000.0
    valid = (depth_roi > 0) & (depth_roi < 65535)

    us = np.arange(w)[None, :] + xoff
    vs = np.arange(h)[:, None] + yoff
    X = (us - K["cx"]) * z / K["fx"]
    Y = (vs - K["cy"]) * z / K["fy"]

    # 中心差分切向量（边缘退化为前/后向差分，numpy 自动处理）
    du = np.zeros((h, w, 3))
    dv = np.zeros((h, w, 3))
    du[:, 1:-1, 0] = X[:, 2:] - X[:, :-2]
    du[:, 1:-1, 2] = z[:, 2:] - z[:, :-2]
    dv[1:-1, :, 1] = Y[2:, :] - Y[:-2, :]
    dv[1:-1, :, 2] = z[2:, :] - z[:-2, :]

    n = np.cross(du, dv)
    norm = np.linalg.norm(n, axis=2)
    with np.errstate(invalid="ignore", divide="ignore"):
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
    d = np.linalg.norm(points - center, axis=1)
    return np.where(np.abs(d - radius) <= thresh)[0]


def ransac_sphere(points: np.ndarray, normals: Optional[np.ndarray] = None,
                  radius_prior: Optional[float] = None,
                  radius_range: Tuple[float, float] = (0.025, 0.045),
                  thresh: float = 0.004, max_iter: int = 300,
                  seed: int = 0) -> Optional[dict]:
    """球 RANSAC：法线可用时 1–2 点采样，否则 4 点采样；半径夹紧。

    理论: c = p − r·n（射线约束）；半径已知 → 最小采样 1 点（迭代数 ~7）；
    半径未知 → 2 点+法线，r = |p₁−p₂|² / ((n₁−n₂)·(p₁−p₂))。

    Returns:
        dict(center, radius, inliers, inlier_ratio, rms) 或 None
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
            b = np.einsum("ij,ij->i", pts, pts)
            sol, *_ = np.linalg.lstsq(a, b, rcond=None)
            t = sol[3] + float(sol[:3] @ sol[:3])
            if t <= 0:
                continue
            center, radius = sol[:3], float(np.sqrt(t))
            if not (radius_range[0] <= radius <= radius_range[1]):
                continue

        inl = _sphere_inliers(points, center, radius, thresh)
        if best is None or len(inl) > len(best["inliers"]):
            best = {"center": center, "radius": radius, "inliers": inl}

    if best is None or len(best["inliers"]) < 10:
        return None
    d = np.linalg.norm(points[best["inliers"]] - best["center"], axis=1)
    best["inlier_ratio"] = len(best["inliers"]) / n
    best["rms"] = float(np.sqrt(np.mean((d - best["radius"]) ** 2)))
    return best


def polish_sphere_lm(points: np.ndarray, center0: np.ndarray,
                     radius: float, fixed_radius: bool = True) -> Tuple[np.ndarray, float]:
    """几何正交距离 LM 抛光（MLE）。fixed_radius=True 时只估 3 DOF 球心。

    理论: min Σ(||p_i−c|| − r)² 是各向同性高斯噪声的最大似然；固定半径
    删除 Fisher 信息矩阵最病态方向，CRLB 严格下降。
    """
    from scipy.optimize import least_squares

    if fixed_radius:
        def resid(c):
            return np.linalg.norm(points - c, axis=1) - radius
        sol = least_squares(resid, center0, method="lm")
        return sol.x, radius

    def resid(cr):
        return np.linalg.norm(points - cr[:3], axis=1) - cr[3]
    sol = least_squares(resid, np.append(center0, radius), method="lm")
    return sol.x[:3], float(sol.x[3])


def fit_sphere_robust(points: np.ndarray, normals: Optional[np.ndarray] = None,
                      radius_prior: Optional[float] = 0.035,
                      radius_range: Tuple[float, float] = (0.025, 0.045),
                      thresh: float = 0.004, max_iter: int = 300,
                      seed: int = 0) -> Optional[dict]:
    """完整球拟合: RANSAC → 内点 LM 抛光 → 重计内点与统计量。"""
    est = ransac_sphere(points, normals, radius_prior, radius_range,
                        thresh, max_iter, seed)
    if est is None:
        return None
    inl_pts = points[est["inliers"]]
    center, radius = polish_sphere_lm(
        inl_pts, est["center"], est["radius"],
        fixed_radius=radius_prior is not None)
    if not (radius_range[0] <= radius <= radius_range[1]):
        return None
    inl = _sphere_inliers(points, center, radius, thresh)
    if len(inl) < 10:
        return None
    d = np.linalg.norm(points[inl] - center, axis=1)
    return {"center": center, "radius": radius, "inliers": inl,
            "inlier_ratio": len(inl) / len(points),
            "rms": float(np.sqrt(np.mean((d - radius) ** 2)))}


# ═══════════════════════════════════════════════════════════════
# 圆柱拟合: 2 点+2 法线 RANSAC + Eberly 轴向抛光
# ═══════════════════════════════════════════════════════════════

def _cylinder_radial_dist(points: np.ndarray, q0: np.ndarray, axis: np.ndarray) -> np.ndarray:
    rel = points - q0
    perp = rel - np.outer(rel @ axis, axis)
    return np.linalg.norm(perp, axis=1)


def ransac_cylinder(points: np.ndarray, normals: np.ndarray,
                    radius_range: Tuple[float, float] = (0.025, 0.050),
                    thresh: float = 0.0035, max_iter: int = 300,
                    seed: int = 0) -> Optional[dict]:
    """圆柱 RANSAC（PCL SACMODEL_CYLINDER 同款构造，clean-room 实现）。

    理论: 圆柱法线 ⊥ 轴 → a = (n₁×n₂)/|n₁×n₂|；投影 ⊥a 平面退化为 2D 圆，
    圆心 = 两射线 p'₁+α·n'₁ 与 p'₂+β·n'₂ 的交点；半径夹紧剪掉退化假设。

    Returns:
        dict(axis, q0, radius, inliers, inlier_ratio, rms) 或 None
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
        if best is None or len(inl) > len(best["inliers"]):
            best = {"axis": a, "q0": center2d, "radius": radius, "inliers": inl}

    if best is None or len(best["inliers"]) < 20:
        return None
    d = _cylinder_radial_dist(points[best["inliers"]], best["q0"], best["axis"])
    best["inlier_ratio"] = len(best["inliers"]) / n
    best["rms"] = float(np.sqrt(np.mean((d - best["radius"]) ** 2)))
    return best


# ── Eberly 圆柱抛光（BSD 3-Clause, 改编自 xingjiepan/cylinder_fitting）──

def _eberly_direction(theta: float, phi: float) -> np.ndarray:
    return np.array([np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta),
                     np.cos(theta)])


def _eberly_G(w: np.ndarray, X: np.ndarray) -> float:
    n = len(X)
    P = np.eye(3) - np.outer(w, w)
    Y = X @ P.T
    A = Y.T @ Y
    S = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    A_hat = S @ A @ S.T
    u = float(np.mean(np.einsum("ij,ij->i", Y, Y)))
    num = A_hat @ (np.einsum("ij,ij->i", Y, Y)[:, None] * Y).sum(axis=0)
    den = np.trace(A_hat @ A)
    if abs(den) < 1e-18:
        return float("inf")
    v = num / den
    return float(np.sum((np.einsum("ij,ij->i", Y, Y) - u - 2.0 * Y @ v) ** 2))


def _eberly_center(w: np.ndarray, X: np.ndarray) -> np.ndarray:
    """Eberly 闭式轴心 C(w)（消元结果，见 cylinder_fitting/fitting.py）。"""
    P = np.eye(3) - np.outer(w, w)
    Y = X @ P.T
    A = Y.T @ Y
    S = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    A_hat = S @ A @ S.T
    num = A_hat @ (np.einsum("ij,ij->i", Y, Y)[:, None] * Y).sum(axis=0)
    den = np.trace(A_hat @ A)
    if abs(den) < 1e-18:
        return np.zeros(3)
    return num / den


def polish_cylinder_axis(points: np.ndarray, axis_hint: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Eberly 轴向抛光：5 维消元为 2 维，以 axis_hint 为起点 Powell 优化。

    Returns: (axis 单位向量, 轴上一点)
    """
    from scipy.optimize import minimize

    t = points.mean(axis=0)
    X = points - t
    hint = axis_hint / np.linalg.norm(axis_hint)
    theta0 = float(np.arccos(np.clip(hint[2], -1.0, 1.0)))
    phi0 = float(np.arctan2(hint[1], hint[0]))
    starts = [(theta0, phi0), (0.0, 0.0),
              (np.pi / 2, 0.0), (np.pi / 2, np.pi / 2)]
    best_w, best_g = None, float("inf")
    for sp in starts:
        res = minimize(lambda x: _eberly_G(_eberly_direction(x[0], x[1]), X),
                       sp, method="Powell", tol=1e-6)
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
    """完整圆柱拟合: RANSAC → Eberly 轴向抛光 → 半径几何重估 → 重计统计量。

    半径用 mean(d_i) 重估（Eberly 代数残差导致 ~1.9mm 系统性半径偏差，
    轴向不受此影响，见 references/notes_fitting_algorithms.md §2.2）。
    """
    est = ransac_cylinder(points, normals, radius_range, thresh, max_iter, seed)
    if est is None:
        return None
    inl = est["inliers"]
    axis, q0 = est["axis"], est["q0"]
    if polish and len(inl) >= 20:
        # 抛光代价随点数线性增长，内点抽稀到 800（精度损失可忽略）
        polish_idx = inl if len(inl) <= 800 else np.linspace(
            0, len(points) - 1, 800, dtype=int)
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
    return {"axis": axis, "q0": q0, "radius": radius, "inliers": inl,
            "inlier_ratio": len(inl) / len(points), "rms": rms}
