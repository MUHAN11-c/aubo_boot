"""
世界系身份分配：χ² 门控 + 匈牙利全局 1-1 分配与跟踪状态分类（纯核）.

语义（见 architecture.md §3 感知层）：马氏距离 χ² 门 + 歧义比判定 +
两档匹配半径；scipy linear_sum_assignment 只承担纯 1-1 求解，门控与
歧义语义在本模块（不可被官方库替代）。
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

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
    矩形代价矩阵的最小化和一对一分配（scipy 官方求解器）.

    禁止边用 +inf：以大有限代价参与求解，结果里再按原代价有限性过滤。
    返回 (row, col) 列表，不含被禁止的配对。
    """
    cost = np.asarray(cost, dtype=float)
    if cost.size == 0:
        return []
    big = 1e12
    finite = np.isfinite(cost)
    C = np.where(finite, cost, big)
    rows, cols = linear_sum_assignment(C)
    return [
        (int(i), int(j)) for i, j in zip(rows, cols)
        if np.isfinite(cost[i, j])]


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


# === 观测质量（observation_quality.py） ===

# 跟踪状态 token：节点映射到 PeachTargetObservation.msg 同名常量
STATUS_OBSERVED = 'OBSERVED'
STATUS_OCCLUDED = 'OCCLUDED'
STATUS_LOST = 'LOST'
STATUS_OUT_OF_VIEW = 'OUT_OF_VIEW'
STATUS_DEPTH_VOID = 'DEPTH_VOID'


def bbox_touches_image_edge(bbox: Tuple[int, int, int, int],
                            width: int, height: int) -> bool:
    """
    检测框是否触及图像边缘（含出界裁剪后贴边）.

    判定 OUT_OF_VIEW 的证据：目标走出视野前最后一帧的检测框必然贴在
    图像某侧边缘上；被枝叶遮挡/检测漏检而消失的目标框一般在图内。

    Args:
        bbox: (x1, y1, x2, y2) 像素框.
        width: 图像宽（像素）.
        height: 图像高（像素）.

    Returns
    -------
        任一边贴到图像边界（x1<=0 / y1<=0 / x2>=width / y2>=height）为真.

    """
    x1, y1, x2, y2 = (int(v) for v in bbox)
    return x1 <= 0 or y1 <= 0 or x2 >= int(width) or y2 >= int(height)


def classify_tracking_status(has_observation: bool, has_mask: bool,
                             mask_depth_ratio: Optional[float],
                             min_depth_ratio: float,
                             last_bbox_touched_edge: bool) -> str:
    """
    单目标跟踪状态四分类（阶段 D1；优先级自上而下首个命中即返回）.

    Args:
        has_observation: 本帧该目标是否有检测/几何输出（payload 非空）.
        has_mask: 本帧是否有可用 SAM 掩膜（仅 has_observation 为真时有意义）.
        mask_depth_ratio: 掩膜内有效深度占比 [0,1]；无掩膜时可为 None.
        min_depth_ratio: DEPTH_VOID 判定的有效深度占比下限.
        last_bbox_touched_edge: 目标消失前最后一帧检测框是否触图像边缘.

    Returns
    -------
        状态 token（本模块 STATUS_* 常量）：
        无观测 → OUT_OF_VIEW（触边消失）/ LOST（其余消失）；
        有观测无掩膜 → OCCLUDED；掩膜内有效深度占比低于阈值 → DEPTH_VOID；
        否则 OBSERVED.

    """
    if not has_observation:
        return STATUS_OUT_OF_VIEW if last_bbox_touched_edge else STATUS_LOST
    if not has_mask:
        return STATUS_OCCLUDED
    ratio = mask_depth_ratio
    if ratio is None or not math.isfinite(float(ratio)):
        # 占比缺失按 0 处理：无法证明有足够实测深度，保守判 DEPTH_VOID
        ratio = 0.0
    if float(ratio) < min_depth_ratio:
        return STATUS_DEPTH_VOID
    return STATUS_OBSERVED
