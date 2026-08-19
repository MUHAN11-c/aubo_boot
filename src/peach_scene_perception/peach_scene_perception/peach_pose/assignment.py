"""
全局一对一身份分配：马氏门控 + 匈牙利最小代价（零 scipy）.

替换逐目标贪心近邻。多解代价接近时标 AMBIGUOUS，不强制合并。
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# 3 自由度约 3σ（χ²₀.₉₉₇ ≈ 11.3；取 9 ≈ 3σ 实用门）
CHI2_GATE = 9.0
AMBIGUOUS_RATIO = 1.2


def regularize_cov(cov, floor: float = 1e-6) -> np.ndarray:
    """3×3 协方差对称化并加对角地板，保证可逆."""
    mat = np.asarray(cov, dtype=float).reshape(3, 3)
    mat = 0.5 * (mat + mat.T)
    mat = mat + floor * np.eye(3)
    return mat


def mahalanobis2(delta, cov) -> float:
    """平方马氏距离 (x-μ)ᵀ Σ⁻¹ (x-μ)."""
    d = np.asarray(delta, dtype=float).reshape(3)
    inv = np.linalg.inv(regularize_cov(cov))
    return float(d @ inv @ d)


def estimate_pose_covariance(
        points: np.ndarray,
        axis: Optional[np.ndarray],
        theta_err_deg: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    由前景点云样本协方差写位置 Σ；轴向 Σ 为垂直于轴的角不确定度.

    点数不足时退回各向同性地板（1 cm / 5°）.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(pts) >= 4:
        centered = pts - np.median(pts, axis=0)
        pos = (centered.T @ centered) / max(len(pts) - 1, 1)
    else:
        pos = (0.01 ** 2) * np.eye(3)
    pos = regularize_cov(pos, floor=1e-6)
    sigma_theta = np.radians(max(float(theta_err_deg), 1.0))
    if axis is None or not np.all(np.isfinite(axis)):
        direction = (sigma_theta ** 2) * np.eye(3)
    else:
        a = np.asarray(axis, dtype=float).reshape(3)
        n = float(np.linalg.norm(a))
        if n < 1e-9:
            direction = (sigma_theta ** 2) * np.eye(3)
        else:
            a = a / n
            # 轴角 σ 映射到切空间：Σ ≈ σ² (I − aaᵀ)
            direction = (sigma_theta ** 2) * (np.eye(3) - np.outer(a, a))
    direction = regularize_cov(direction, floor=1e-8)
    return pos, direction


def hungarian(cost: np.ndarray) -> List[Tuple[int, int]]:
    """
    矩形代价矩阵的最小权和一对一分配（Munkres）.

    禁止边用 +inf。返回 (row, col) 列表，不含填充虚节点。
    """
    cost = np.asarray(cost, dtype=float)
    if cost.size == 0:
        return []
    n, m = cost.shape
    k = max(n, m)
    big = 1e12
    C = np.full((k, k), big, dtype=float)
    finite = np.isfinite(cost)
    C[:n, :m] = np.where(finite, cost, big)
    # 行减最小值
    C = C - C.min(axis=1, keepdims=True)
    C = C - C.min(axis=0, keepdims=True)
    star = np.zeros((k, k), dtype=bool)
    prime = np.zeros((k, k), dtype=bool)
    row_cover = np.zeros(k, dtype=bool)
    col_cover = np.zeros(k, dtype=bool)
    for i in range(k):
        for j in range(k):
            if C[i, j] == 0 and not row_cover[i] and not col_cover[j]:
                star[i, j] = True
                row_cover[i] = True
                col_cover[j] = True
    row_cover[:] = False
    col_cover[:] = False

    def cover_starred():
        col_cover[:] = star.any(axis=0)

    cover_starred()
    while col_cover.sum() < k:
        def find_uncovered_zero():
            for i in range(k):
                if row_cover[i]:
                    continue
                for j in range(k):
                    if not col_cover[j] and C[i, j] == 0 and not prime[i, j]:
                        return i, j
            return None

        while True:
            z = find_uncovered_zero()
            if z is None:
                leftover = C[~row_cover][:, ~col_cover]
                if leftover.size == 0:
                    break
                mval = leftover.min()
                C[~row_cover] += mval
                C[:, ~col_cover] -= mval
                C[np.abs(C) < 1e-12] = 0.0
                continue
            i, j = z
            prime[i, j] = True
            star_cols = np.where(star[i])[0]
            if star_cols.size:
                row_cover[i] = True
                col_cover[star_cols[0]] = False
                continue
            # 增广路
            path = [(i, j)]
            while True:
                star_rows = np.where(star[:, path[-1][1]])[0]
                if not star_rows.size:
                    break
                r = int(star_rows[0])
                path.append((r, path[-1][1]))
                prime_cols = np.where(prime[r])[0]
                path.append((r, int(prime_cols[0])))
            for r, c in path:
                star[r, c] = not star[r, c]
            prime[:] = False
            row_cover[:] = False
            col_cover[:] = False
            cover_starred()
            break

    pairs = []
    for i in range(n):
        js = np.where(star[i, :m])[0]
        if js.size and np.isfinite(cost[i, js[0]]):
            pairs.append((i, int(js[0])))
    return pairs


def assign_detections(
        detections: Sequence[dict],
        table: Dict[str, dict],
        frame_used: set,
        match_radius: float,
        recovery_scale: float = 1.0,
) -> List[Tuple[Optional[str], float, str]]:
    """
    本帧检测相对表项做全局 1-1 分配.

    每个 detection 字典需含 position、(可选) covariance、class_id.
    返回与 detections 等长的 (target_id|None, mahalanobis2, status).
    status: ok / new / ambiguous.
    """
    n = len(detections)
    if n == 0:
        return []
    tracks = [
        (tid, rec) for tid, rec in table.items() if tid not in frame_used]
    if not tracks:
        return [(None, 0.0, 'new') for _ in detections]

    cost = np.full((n, len(tracks)), np.inf)
    for i, det in enumerate(detections):
        pos = np.asarray(det['position'], dtype=float).reshape(3)
        cov = det.get('covariance')
        if cov is None:
            sigma = max(match_radius / 3.0, 1e-3) * float(recovery_scale)
            cov = (sigma ** 2) * np.eye(3)
        else:
            cov = regularize_cov(cov) * (float(recovery_scale) ** 2)
        cid = int(det.get('class_id', 0))
        for j, (_tid, rec) in enumerate(tracks):
            if rec.get('class_id', cid) != cid:
                continue
            d2 = mahalanobis2(pos - rec['position'], cov)
            if d2 <= CHI2_GATE:
                cost[i, j] = d2

    pairs = hungarian(cost)
    assigned_cols = {j for _, j in pairs}
    assigned_rows = {i for i, _ in pairs}
    # 歧义：某检测存在另一未占用轨道，代价与最优比 < AMBIGUOUS_RATIO
    results: List[Tuple[Optional[str], float, str]] = [
        (None, 0.0, 'new') for _ in detections]
    for i, j in pairs:
        best = cost[i, j]
        ambiguous = False
        for jj in range(len(tracks)):
            if jj == j or jj in assigned_cols:
                continue
            alt = cost[i, jj]
            if (np.isfinite(alt) and alt * AMBIGUOUS_RATIO >= best
                    and alt <= best * AMBIGUOUS_RATIO):
                ambiguous = True
                break
        if ambiguous:
            results[i] = (None, float(best), 'ambiguous')
        else:
            results[i] = (tracks[j][0], float(best), 'ok')
    for i in range(n):
        if i not in assigned_rows:
            # 若有多个有限代价却因匈牙利落到 inf（被占），保持 new
            results[i] = (None, 0.0, 'new')
    return results
