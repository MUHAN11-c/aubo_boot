"""
Safety-gated RGB-D pose pipelines for one target, bagged or unbagged.

Two parallel lines sharing the same cylindrical cutting tool and safety gates:

- ``RobustBagPosePipeline``   — bagged peach (class 0): cylinder-RANSAC axis
- ``RobustFruitPosePipeline`` — unbagged peach (class 1): sphere fit + stem-cavity axis

The module intentionally has no GUI, Open3D, Torch, or robot dependency.  This
makes the safety decision testable on recorded RGB-D pairs and prevents a
visualisation-only algorithm from becoming an implicit execution path.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from .contracts import (
    BagGrasp2D, BagGraspReference3D, BagObservation, compute_entry_start,
    compute_travel_range, TOOL_GEOMETRY, ToolGeometry,
)
from .fitting import (
    estimate_normals, fit_cylinder_robust, fit_sphere_robust, polish_sphere_lm,
)


@dataclass
class TargetPoseResult:
    """Result for exactly one target; never indexes another detection."""

    target_id: str
    grasp_2d: BagGrasp2D
    grasp_3d: BagGraspReference3D
    mask_source: str
    metrics: dict
    target_kind: str = 'bag'  # "bag" | "fruit"


class RobustBagPosePipeline:
    """
    Conservative pose estimator for a cylindrical bag insertion tool.

    It uses only measured depth for all safety decisions.  A supplied instance
    mask is preferred, while a depth-band connected component is an explicit,
    inspectable fallback.
    """

    def __init__(self, tool: ToolGeometry = TOOL_GEOMETRY, min_depth_m=0.3,
                 max_depth_m=2.5, min_points=100):
        self.tool = tool
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.min_points = min_points

    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback') -> TargetPoseResult:
        """Estimate one target in ``bbox`` and return an explicit safe status."""
        x1, y1, x2, y2 = self._clip_bbox(bbox, obs.depth.shape)
        base_2d = BagGrasp2D(detection_bbox=(x1, y1, x2 - x1, y2 - y1))
        if x2 - x1 < 8 or y2 - y1 < 8:
            return self._failed(target_id, base_2d, 'invalid_bbox', mask_source)

        roi = obs.depth[y1:y2, x1:x2]
        valid = self._valid_depth(roi)
        valid_ratio = float(valid.mean()) if valid.size else 0.0
        local_mask, source = self._foreground(roi, valid, mask, bbox, source=mask_source)
        base_2d.foreground_mask = local_mask
        coverage = float(local_mask.mean()) if local_mask.size else 0.0
        points, pixels = self._to_points(roi, local_mask, x1, y1, obs.camera_K)
        points, pixels = self._filter_depth_outliers(points, pixels)
        if len(points) < self.min_points:
            return self._failed(target_id, base_2d, 'insufficient_measured_points', source,
                                valid_depth_ratio=valid_ratio, foreground_ratio=coverage,
                                n_points=len(points))

        gravity = np.asarray(obs.gravity_hint if obs.gravity_hint is not None
                             else [0.0, 1.0, 0.0], dtype=float)
        if np.linalg.norm(gravity) < 1e-8:
            return self._failed(target_id, base_2d, 'invalid_gravity', source,
                                valid_depth_ratio=valid_ratio, foreground_ratio=coverage,
                                n_points=len(points))
        gravity /= np.linalg.norm(gravity)

        # ── 套入轴估计: 圆柱 RANSAC 主估 → 2D 掩膜校验 → 重力显式降级 ──
        # 理论: 圆柱法线 ⊥ 轴 (a = n₁×n₂)；局部几何结构良态，全局 PCA 对
        # 近回转体退化；重力假设仅在自由悬垂时成立 (docs/grasp_axis_entry_design.md)
        normals_map, nvalid_map = estimate_normals(roi, x1, y1, obs.camera_K)
        pnormals = normals_map[pixels[:, 1], pixels[:, 0]]
        pnvalid = nvalid_map[pixels[:, 1], pixels[:, 0]]
        fit_pts, fit_nrm = points[pnvalid], pnormals[pnvalid]

        axis = None
        axis_source = 'gravity_prior'
        axis_confidence = 0.4          # 重力先验置信度封顶 0.4
        theta_err_deg = 20.0           # 重力先验的保守角误差 (deg)
        cyl = None
        if len(fit_pts) >= 200:
            cyl = fit_cylinder_robust(fit_pts, fit_nrm)
        if cyl is not None and cyl['inlier_ratio'] >= 0.35:
            axis = cyl['axis']
            axis_source = 'cylinder_ransac'
            axis_confidence = float(
                np.clip(cyl['inlier_ratio'] / 0.7, 0.0, 1.0)
                * np.clip(1.0 - cyl['rms'] / 0.004, 0.0, 1.0))
        if axis is None:
            axis = -gravity
        # 轴符号定向: 底→颈 = 逆重力方向；近水平时定向不可靠
        if float(axis @ gravity) > 0.0:
            axis = -axis
        orientation_uncertain = abs(float(axis @ gravity)) < 0.3

        # ── 底/颈/袋径: 沿拟合轴投影分位带（背面遮挡时为观测下界） ──
        proj = points @ axis
        bottom_band = points[proj <= np.percentile(proj, 10)]
        neck_band = points[proj >= np.percentile(proj, 90)]
        transverse_points = points - np.outer(points @ axis, axis)
        transverse_center = np.median(transverse_points, axis=0)
        bottom_axis = float(np.median(bottom_band @ axis))
        neck_axis = float(np.median(neck_band @ axis))
        bottom = transverse_center + bottom_axis * axis
        neck = transverse_center + neck_axis * axis
        length = float(np.dot(neck - bottom, axis))
        if length <= 0.03:
            return self._failed(target_id, base_2d, 'bag_axis_too_short', source,
                                valid_depth_ratio=valid_ratio, foreground_ratio=coverage,
                                n_points=len(points), bag_length_m=length)

        # 2D 校验: 掩膜 PCA 主轴 与 袋轴投影(底→颈) 的夹角
        disagreement_deg = None
        _bpx, _npx = self._project(bottom, obs.camera_K), self._project(neck, obs.camera_K)
        if _bpx is not None and _npx is not None:
            disagreement_deg = self._mask_axis_disagreement(local_mask, _bpx, _npx)

        radial = np.linalg.norm(transverse_points - transverse_center, axis=1)
        diameter = float(2.0 * np.percentile(radial, 95))

        # ── entry_start = P_bottom − (d_tool + d_s)·axis (Gürsoy 分解) ──
        standoff = self.tool.entry_d_tool + self.tool.entry_d_s
        entry = compute_entry_start(bottom, axis, standoff)
        _, travel = compute_travel_range(entry, neck, axis, self.tool)
        R = self._frame(axis, points)

        # ── 误差预算: δ = L·sin(θ_err) ≤ 径向余量 (docs/grasp_axis_entry_design.md §核心二) ──
        # θ_err 代理: 拟合残差/袋长（指向误差）+ 定向与 2D 校验惩罚。
        # 注意: 圆形掩膜的 2D 主轴无意义，disagreement 仅在掩膜细长时生效。
        if cyl is not None and axis_source == 'cylinder_ransac':
            theta_err_deg = float(np.clip(
                np.degrees(np.arctan2(2.0 * cyl['rms'], max(length, 0.03))),
                2.0, 30.0))
        if orientation_uncertain:
            theta_err_deg = max(theta_err_deg, 12.0)
        radial_clearance = self.tool.D_inner / 2.0 - diameter / 2.0 - self.tool.clearance_min
        budget_m = (standoff + travel) * np.sin(np.radians(theta_err_deg))

        flags = []
        if valid_ratio < 0.40:
            flags.append('low_valid_depth')
        if coverage < 0.01:
            flags.append('small_foreground')
        if diameter + 2.0 * self.tool.clearance_min >= self.tool.D_inner:
            flags.append('tool_clearance_failed')
        if travel < 0.05:
            flags.append('travel_too_short')
        if axis_source == 'gravity_prior':
            flags.append('axis_from_gravity_prior')
        if orientation_uncertain:
            flags.append('axis_orientation_uncertain')
        if disagreement_deg is not None and disagreement_deg > 45.0:
            flags.append('axis_2d_mismatch')
        if budget_m * 1000.0 > radial_clearance * 1000.0:
            flags.append('error_budget_exceeded')
        # A mask touching three or more ROI sides is normally an unresolved
        # crop/occlusion, not evidence of a complete bag envelope.
        boundary_touch, boundary_sides = self._boundary_metrics(local_mask)
        if boundary_touch > 0.15 or boundary_sides >= 3:
            flags.append('foreground_truncated')

        status = 'ACCEPT' if not flags else (
            'REJECT' if 'tool_clearance_failed' in flags else 'REOBSERVE')
        confidence = float(np.clip(
            min(valid_ratio / 0.65, 1.0) * min(len(points) / 800.0, 1.0)
            * (1.0 - min(boundary_touch, 0.8)), 0.0, 1.0))
        base_2d.bottom_px = self._project(bottom, obs.camera_K)
        base_2d.neck_px = self._project(neck, obs.camera_K)
        base_2d.grasp_px = self._project(entry, obs.camera_K)
        base_2d.bag_axis_line = [base_2d.bottom_px, base_2d.neck_px]
        base_2d.travel_line = [
            base_2d.grasp_px, self._project(entry + travel * axis, obs.camera_K)]
        base_2d.confidence = confidence
        base_2d.status = status
        base_2d.diagnostic_flags = flags.copy()
        metrics = {'valid_depth_ratio': valid_ratio, 'foreground_ratio': coverage,
                   'boundary_touch_ratio': boundary_touch,
                   'boundary_sides_touched': boundary_sides,
                   'n_points': len(points),
                   'bag_length_m': length, 'bag_diameter_upper_m': diameter,
                   'travel_m': travel,
                   'axis_confidence': axis_confidence,
                   'axis_disagreement_deg': (None if disagreement_deg is None
                                             else float(disagreement_deg)),
                   'theta_err_deg': float(theta_err_deg),
                   'error_budget_mm': float(budget_m * 1000.0),
                   'radial_clearance_mm': float(radial_clearance * 1000.0),
                   # ROS2 BagFitting 追加字段（纯诊断，不参与门控）
                   'cylinder_rms_m': (float(cyl['rms']) if cyl is not None else None),
                   'cylinder_inlier_ratio': (
                       float(cyl['inlier_ratio']) if cyl is not None else None)}
        g3d = BagGraspReference3D(
            frame_id=obs.frame_id, entry_start=entry, position=entry,
            orientation=R, bag_bottom=bottom, bag_neck=neck,
            translation_direction=axis, bag_diameter_upper_m=diameter,
            suggested_travel_m=travel, suggested_travel_end=entry + travel * axis,
            confidence=confidence, status=status, diagnostic_flags=flags,
            diagnostic_info={**metrics, 'mask_source': source,
                             'axis_source': axis_source,
                             'D_bag_mm': f'{diameter * 1000:.0f}'},
            strategy_id='robust_bag_pose',
            model_version=str(obs.metadata.get('model_version', 'unknown')),
            calibration_version=str(obs.metadata.get(
                'calibration_version', 'unknown')),
            tool_version=self.tool.version)
        return TargetPoseResult(target_id, base_2d, g3d, source, metrics)

    def _failed(self, target_id, g2d, reason, source, **metrics):
        g2d.status = 'REJECT'
        g2d.diagnostic_flags = [reason]
        g3d = BagGraspReference3D(status='REJECT', diagnostic_flags=[reason],
                                  strategy_id='robust_bag_pose', tool_version=self.tool.version,
                                  diagnostic_info={**metrics, 'mask_source': source})
        return TargetPoseResult(target_id, g2d, g3d, source, metrics)

    @staticmethod
    def _clip_bbox(bbox, shape):
        h, w = shape[:2]
        x1, y1, x2, y2 = (int(round(v)) for v in bbox)
        return max(0, x1), max(0, y1), min(w, x2), min(h, y2)

    def _valid_depth(self, depth):
        z = depth.astype(np.float32) / 1000.0
        return (depth > 0) & (depth < 65535) & (z > self.min_depth_m) & (z < self.max_depth_m)

    def _foreground(self, depth, valid, supplied_mask, bbox, source):
        h, w = depth.shape
        if supplied_mask is not None:
            m = np.asarray(supplied_mask, dtype=bool)
            if m.shape != (h, w):
                x1, y1, x2, y2 = map(int, bbox)
                if m.ndim == 2 and m.shape[0] >= y2 and m.shape[1] >= x2:
                    m = m[y1:y2, x1:x2]
            if m.shape == (h, w) and int((m & valid).sum()) >= 50:
                return m & valid, source
        # Explicit fallback: depth mode in central region + connected component.
        ch, cw = max(1, h // 3), max(1, w // 3)
        cy, cx = h // 2, w // 2
        centre = depth[cy-ch//2:cy+(ch+1)//2, cx-cw//2:cx+(cw+1)//2]
        values = centre[(centre > 0) & (centre < 65535)]
        if len(values) < 20:
            return np.zeros_like(valid), 'depth_fallback'
        z0 = np.median(values)
        mad = np.median(np.abs(values.astype(float) - z0))
        band = max(25.0, 3.0 * mad)
        binary = ((np.abs(depth.astype(float) - z0) <= band) & valid).astype(np.uint8)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
        if n <= 1:
            return binary.astype(bool), 'depth_fallback'
        label = labels[cy, cx]
        if label == 0:
            label = int(1 + np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        return labels == label, 'depth_fallback'

    def _to_points(self, depth, mask, xoff, yoff, K):
        ys, xs = np.where(mask & self._valid_depth(depth))
        z = depth[ys, xs].astype(float) / 1000.0
        points = np.column_stack(((xs + xoff - K['cx']) * z / K['fx'],
                                 (ys + yoff - K['cy']) * z / K['fy'], z))
        return points, np.column_stack((xs, ys))

    @staticmethod
    def _filter_depth_outliers(points, pixels):
        if len(points) < 10:
            return points, pixels
        z = points[:, 2]
        med = np.median(z)
        mad = np.median(np.abs(z - med))
        if mad < 1e-5:
            return points, pixels
        keep = np.abs(z - med) <= 3.5 * mad
        return points[keep], pixels[keep]

    @staticmethod
    def _boundary_metrics(mask):
        if not mask.any():
            return 1.0, 4
        border = np.concatenate((mask[0], mask[-1], mask[:, 0], mask[:, -1]))
        ratio = float(border.mean())
        side_fractions = (mask[0].mean(), mask[-1].mean(),
                          mask[:, 0].mean(), mask[:, -1].mean())
        sides = sum(fraction > 0.05 for fraction in side_fractions)
        return ratio, int(sides)

    @staticmethod
    def _frame(axis, points):
        centred = points - points.mean(axis=0)
        _, vec = np.linalg.eigh(centred.T @ centred / max(len(points), 1))
        x = vec[:, -1] - np.dot(vec[:, -1], axis) * axis
        if np.linalg.norm(x) < 1e-8:
            ref = np.array([1., 0., 0.]) if abs(axis[0]) < .9 else np.array([0., 0., 1.])
            x = np.cross(axis, ref)
        x /= np.linalg.norm(x)
        if x[0] < 0:
            x = -x
        y = np.cross(axis, x)
        y /= np.linalg.norm(y)
        return np.column_stack((x, y, axis))

    @staticmethod
    def _mask_axis_disagreement(mask: np.ndarray, bottom_px: tuple,
                                neck_px: tuple) -> Optional[float]:
        """
        2D 校验: 前景掩膜 PCA 主轴 与 袋轴投影(底→颈像素) 的无向夹角 (deg).

        对极点/轴向错误的廉价交叉验证（Kok 2024 的 180° 尾部教训）：
        3D 拟合轴投影到图像后与掩膜 2D 主轴夹角过大，说明拟合可疑。
        """
        ys, xs = np.where(mask)
        if len(xs) < 30:
            return None
        a2d = np.array([neck_px[0] - bottom_px[0],
                        neck_px[1] - bottom_px[1]], dtype=float)
        la = np.linalg.norm(a2d)
        if la < 5.0:  # 投影过短，轴向几乎正对相机，2D 校验无意义
            return None
        a2d /= la
        pts2d = np.column_stack([xs, ys]).astype(float)
        pts2d -= pts2d.mean(axis=0)
        cov = pts2d.T @ pts2d / len(pts2d)
        evals, vecs = np.linalg.eigh(cov)
        if evals[-1] < 2.5 * max(evals[0], 1e-9):
            return None  # 掩膜近圆形，2D 主轴无判别力
        v2d = vecs[:, -1]
        cos = abs(float(a2d @ v2d))
        return float(np.degrees(np.arccos(np.clip(cos, 0.0, 1.0))))

    @staticmethod
    def _project(point, K):
        if point is None or point[2] <= 1e-8:
            return None
        return (float(point[0] * K['fx'] / point[2] + K['cx']),
                float(point[1] * K['fy'] / point[2] + K['cy']))


# ═══════════════════════════════════════════════════════════════
# 裸果管线 (class 1, peach_nobag) — 同一圆柱剪切工具的第二条流程线
# ═══════════════════════════════════════════════════════════════

class RobustFruitPosePipeline(RobustBagPosePipeline):
    """
    Unbagged-peach pose estimator for the same cylindrical cutting tool.

    与袋装线并列：袋装用"圆柱 RANSAC 定轴"，裸果是近球体、没有圆柱结构，
    改用"球拟合定心定径 + 梗洼定向"：

    - **球心/半径**：点+法线 RANSAC 球拟合（fitting.fit_sphere_robust），
      半径夹紧 [25,45]mm（成熟桃 Ø60–85mm），内点几何 LM 抛光；
    - **套入轴（梗方向）**：桃的果梗附着处有凹陷（植物学事实）。在拟合球面上
      按 Fibonacci 方向扫描，找"实测表面相对拟合球面下陷最深"的方向帽
      （径向残差中位数 < −1.5mm 且点数足够）作为梗端方向——这是零标注的
      局部几何原语（ROG-Grasp 式局部结构优于全局启发式的同一思想）；
    - **降级**：梗洼不可见（被叶挡/背对相机/形状光滑）→ 重力先验，
      置信度封顶 0.4，状态至多 REOBSERVE，绝不硬给 ACCEPT；
    - **entry/行程/净空/误差预算**：与袋装线完全相同的公式与门控。

    参考点定义：bottom = 球心 − r·axis（远离梗端，圆柱从此处起套），
    neck = 球心 + r·axis（梗端，刀片在其前方 margin_neck 处停止）。
    """

    CAVITY_HALF_ANGLE_COS = np.cos(np.radians(20.0))
    CAVITY_MIN_DIP_M = 0.0015     # 梗洼最小下陷深度 1.5mm
    CAVITY_MIN_POINTS = 12

    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback') -> TargetPoseResult:
        """Estimate one unbagged peach in ``bbox`` and return an explicit safe status."""
        x1, y1, x2, y2 = self._clip_bbox(bbox, obs.depth.shape)
        base_2d = BagGrasp2D(detection_bbox=(x1, y1, x2 - x1, y2 - y1))
        if x2 - x1 < 8 or y2 - y1 < 8:
            return self._failed_fruit(target_id, base_2d, 'invalid_bbox', mask_source)

        roi = obs.depth[y1:y2, x1:x2]
        valid = self._valid_depth(roi)
        valid_ratio = float(valid.mean()) if valid.size else 0.0
        local_mask, source = self._foreground(roi, valid, mask, bbox, source=mask_source)
        base_2d.foreground_mask = local_mask
        coverage = float(local_mask.mean()) if local_mask.size else 0.0
        points, pixels = self._to_points(roi, local_mask, x1, y1, obs.camera_K)
        points, pixels = self._filter_depth_outliers(points, pixels)
        if len(points) < self.min_points:
            return self._failed_fruit(target_id, base_2d, 'insufficient_measured_points',
                                      source, valid_depth_ratio=valid_ratio,
                                      foreground_ratio=coverage, n_points=len(points))

        gravity = np.asarray(obs.gravity_hint if obs.gravity_hint is not None
                             else [0.0, 1.0, 0.0], dtype=float)
        if np.linalg.norm(gravity) < 1e-8:
            return self._failed_fruit(target_id, base_2d, 'invalid_gravity', source,
                                      valid_depth_ratio=valid_ratio,
                                      foreground_ratio=coverage, n_points=len(points))
        gravity /= np.linalg.norm(gravity)

        normals_map, nvalid_map = estimate_normals(roi, x1, y1, obs.camera_K)
        pnormals = normals_map[pixels[:, 1], pixels[:, 0]]
        pnvalid = nvalid_map[pixels[:, 1], pixels[:, 0]]

        # ── 球拟合定心定径 (点+法线 RANSAC + 几何抛光) ──
        sph = fit_sphere_robust(points[pnvalid], pnormals[pnvalid],
                                radius_prior=None,
                                radius_range=(0.025, 0.045)) if pnvalid.sum() >= 50 else None
        sphere_ok = sph is not None and sph['inlier_ratio'] >= 0.35

        # ── 套入轴: 梗洼检测(+剔洼二轮抛光、重力极性校正) → 重力显式降级 ──
        axis = None
        axis_source = 'gravity_prior'
        axis_confidence = 0.4
        theta_err_deg = 20.0
        cavity_dip_mm = None
        polarity_corrected = False
        if sphere_ok:
            inl_pts = points[sph['inliers']]
            center, radius = sph['center'], sph['radius']
            axis0, dip0 = self._stem_cavity_axis(inl_pts, center, radius)
            if axis0 is not None:
                # 洼区会拉偏球面参考：剔除洼帽后抛光（固定半径只估球心——
                # 去掉帽后弧段更小，放开半径会重新激活半径/球心耦合病态）
                rel = inl_pts - center
                u = rel / np.linalg.norm(rel, axis=1, keepdims=True)
                cap = (u @ axis0) > np.cos(np.radians(15.0))
                if int((~cap).sum()) >= 50:
                    center = polish_sphere_lm(
                        inl_pts[~cap], center, radius, fixed_radius=True)[0]
                    axis0, dip0 = self._stem_cavity_axis(inl_pts, center, radius)
                if axis0 is not None:
                    axis = axis0
                    cavity_dip_mm = float(dip0 * 1000.0)
                    axis_source = 'stem_cavity'
                    axis_confidence = float(np.clip(-dip0 / 0.006, 0.05, 1.0))
                    theta_err_deg = float(np.clip(
                        np.degrees(np.arctan2(2.0 * sph['rms'], 2.0 * radius))
                        + (1.0 - axis_confidence) * 6.0, 3.0, 30.0))
                    sph = {**sph, 'center': center, 'radius': radius}
        if axis is None:
            axis = -gravity
        # 重力极性校正: 桃挂枝梗朝上（Sa 2017 先验），检测到的洼朝下说明是萼洼
        # （Kok 2024 的对极点混淆），翻轴 + 降置信 + 加角罚，而不是直接放弃
        if axis_source == 'stem_cavity' and float(axis @ gravity) > 0.0:
            axis = -axis
            polarity_corrected = True
            axis_confidence *= 0.7
            theta_err_deg = min(theta_err_deg + 5.0, 30.0)
        # 近水平定向（梗向与重力近垂直）才是真正歧义
        orientation_uncertain = abs(float(axis @ gravity)) < 0.2

        # ── 参考点与袋径（沿轴） ──
        if sphere_ok:
            center, radius = sph['center'], sph['radius']
        else:
            # 无球时的保守退化: 横向中位中心 + 横向 P95 半径
            transverse0 = points - np.outer(points @ axis, axis)
            center = np.median(transverse0, axis=0) + float(
                np.median(points @ axis)) * axis
            radius = float(np.percentile(
                np.linalg.norm(transverse0 - np.median(transverse0, axis=0), axis=1), 95))
        bottom = center - radius * axis
        neck = center + radius * axis
        length = 2.0 * radius
        # 横向 P95 袋径（相对球心、⊥轴），与拟合直径取保守大者
        radial = np.linalg.norm(
            (points - center) - np.outer((points - center) @ axis, axis), axis=1)
        diameter_p95 = float(2.0 * np.percentile(radial, 95))
        diameter = max(2.0 * radius, diameter_p95)  # 保守取大

        # ── entry_start = P_bottom − (d_tool + d_s)·axis（与袋装线同公式） ──
        standoff = self.tool.entry_d_tool + self.tool.entry_d_s
        entry = compute_entry_start(bottom, axis, standoff)
        _, travel = compute_travel_range(entry, neck, axis, self.tool)
        R = self._frame(axis, points)

        # ── 误差预算（与袋装线同公式） ──
        if orientation_uncertain:
            theta_err_deg = max(theta_err_deg, 12.0)
        radial_clearance = self.tool.D_inner / 2.0 - diameter / 2.0 - self.tool.clearance_min
        budget_m = (standoff + travel) * np.sin(np.radians(theta_err_deg))

        disagreement_deg = None
        _bpx, _npx = self._project(bottom, obs.camera_K), self._project(neck, obs.camera_K)
        if _bpx is not None and _npx is not None:
            disagreement_deg = self._mask_axis_disagreement(local_mask, _bpx, _npx)

        flags = []
        if valid_ratio < 0.40:
            flags.append('low_valid_depth')
        if coverage < 0.01:
            flags.append('small_foreground')
        if diameter + 2.0 * self.tool.clearance_min >= self.tool.D_inner:
            flags.append('tool_clearance_failed')
        if travel < 0.05:
            flags.append('travel_too_short')
        if not sphere_ok:
            flags.append('sphere_fit_unstable')
        if axis_source == 'gravity_prior':
            flags.append('axis_from_gravity_prior')
        if orientation_uncertain:
            flags.append('axis_orientation_uncertain')
        if disagreement_deg is not None and disagreement_deg > 45.0:
            flags.append('axis_2d_mismatch')
        if budget_m * 1000.0 > radial_clearance * 1000.0:
            flags.append('error_budget_exceeded')
        boundary_touch, boundary_sides = self._boundary_metrics(local_mask)
        if boundary_touch > 0.15 or boundary_sides >= 3:
            flags.append('foreground_truncated')

        status = 'ACCEPT' if not flags else (
            'REJECT' if 'tool_clearance_failed' in flags else 'REOBSERVE')
        confidence = float(np.clip(
            min(valid_ratio / 0.65, 1.0) * min(len(points) / 800.0, 1.0)
            * (1.0 - min(boundary_touch, 0.8)), 0.0, 1.0))
        base_2d.bottom_px = self._project(bottom, obs.camera_K)
        base_2d.neck_px = self._project(neck, obs.camera_K)
        base_2d.grasp_px = self._project(entry, obs.camera_K)
        base_2d.bag_axis_line = [base_2d.bottom_px, base_2d.neck_px]
        base_2d.travel_line = [
            base_2d.grasp_px, self._project(entry + travel * axis, obs.camera_K)]
        base_2d.confidence = confidence
        base_2d.status = status
        base_2d.diagnostic_flags = flags.copy()
        metrics = {'valid_depth_ratio': valid_ratio, 'foreground_ratio': coverage,
                   'boundary_touch_ratio': boundary_touch,
                   'boundary_sides_touched': boundary_sides,
                   'n_points': len(points),
                   'bag_length_m': length, 'bag_diameter_upper_m': diameter,
                   'travel_m': travel,
                   'fruit_radius_m': (radius if sphere_ok else None),
                   'sphere_rms_m': (sph['rms'] if sphere_ok else None),
                   'sphere_inlier_ratio': (sph['inlier_ratio'] if sphere_ok else None),
                   'cavity_dip_mm': cavity_dip_mm,
                   'axis_polarity_corrected': polarity_corrected,
                   'axis_confidence': axis_confidence,
                   'axis_disagreement_deg': (None if disagreement_deg is None
                                             else float(disagreement_deg)),
                   'theta_err_deg': float(theta_err_deg),
                   'error_budget_mm': float(budget_m * 1000.0),
                   'radial_clearance_mm': float(radial_clearance * 1000.0)}
        g3d = BagGraspReference3D(
            frame_id=obs.frame_id, entry_start=entry, position=entry,
            orientation=R, bag_bottom=bottom, bag_neck=neck,
            translation_direction=axis, bag_diameter_upper_m=diameter,
            suggested_travel_m=travel, suggested_travel_end=entry + travel * axis,
            confidence=confidence, status=status, diagnostic_flags=flags,
            diagnostic_info={**metrics, 'mask_source': source,
                             'axis_source': axis_source,
                             'D_bag_mm': f'{diameter * 1000:.0f}'},
            strategy_id='robust_fruit_pose',
            model_version=str(obs.metadata.get('model_version', 'unknown')),
            calibration_version=str(obs.metadata.get(
                'calibration_version', 'unknown')),
            tool_version=self.tool.version)
        return TargetPoseResult(target_id, base_2d, g3d, source, metrics,
                                target_kind='fruit')

    def _stem_cavity_axis(self, points: np.ndarray, center: np.ndarray,
                          radius: float) -> Tuple[Optional[np.ndarray], float]:
        """
        梗洼定向: 拟合球面上径向残差下陷最深的方向帽 = 果梗方向.

        理论: 桃果梗附着处凹陷（植物学形态）；拟合球面是参考面，表面点
        相对球面的径向残差 d_i − r 在梗端方向帽内显著为负。打分取 P30
        分位以容忍扫描帽大于真实洼区的稀释。零标注局部几何原语，
        只对可见半球有效（无点方向自动跳过）。
        """
        rel = points - center
        d = np.linalg.norm(rel, axis=1)
        ok = d > 1e-6
        if ok.sum() < 30:
            return None, 0.0
        u = rel[ok] / d[ok, None]
        resid = d[ok] - radius

        # Fibonacci 球面方向采样（约 200 向）
        n_dir = 200
        k = np.arange(n_dir)
        z = 1.0 - 2.0 * (k + 0.5) / n_dir
        phi = k * np.pi * (3.0 - np.sqrt(5.0))
        dirs = np.column_stack((np.sqrt(1 - z ** 2) * np.cos(phi),
                                np.sqrt(1 - z ** 2) * np.sin(phi), z))
        sim = dirs @ u.T                       # (n_dir, n_pts)
        best_i, best_dip = -1, 0.0
        for i in range(n_dir):
            sel = sim[i] >= self.CAVITY_HALF_ANGLE_COS
            if sel.sum() < self.CAVITY_MIN_POINTS:
                continue
            # P30 分位: 容忍扫描帽(20°)大于真实洼区时的稀释（中位数会被拉到 0）
            dip = float(np.percentile(resid[sel], 30))
            if dip < best_dip:
                best_dip, best_i = dip, i
        if best_i < 0 or best_dip > -self.CAVITY_MIN_DIP_M:
            return None, 0.0
        # 方向精化: 帽内点残差加权平均方向
        sel = sim[best_i] >= self.CAVITY_HALF_ANGLE_COS
        w = np.clip(-(resid[sel]), 0.0, None) + 1e-6
        refined = (u[sel] * w[:, None]).sum(axis=0)
        refined /= np.linalg.norm(refined)
        return refined, best_dip

    def _failed_fruit(self, target_id, g2d, reason, source, **metrics):
        g2d.status = 'REJECT'
        g2d.diagnostic_flags = [reason]
        g3d = BagGraspReference3D(status='REJECT', diagnostic_flags=[reason],
                                  strategy_id='robust_fruit_pose',
                                  tool_version=self.tool.version,
                                  diagnostic_info={**metrics, 'mask_source': source})
        return TargetPoseResult(target_id, g2d, g3d, source, metrics,
                                target_kind='fruit')
