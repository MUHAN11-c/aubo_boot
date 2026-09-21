"""
袋/果位姿管线：圆柱轴袋线与球+梗洼果线（看场景算法核）.

两条线共用同一圆柱刀具、入口/行程公式与安全门控；掩膜/深度门控
委托 image_gates，协方差委托 identity，拟合委托 common.geometry。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from peach_harvester.vision.common.bag_landmarks import (
    clamp_upper_hemisphere,
    enforce_wide_bottom,
    estimate_bag_landmarks,
)
from peach_harvester.vision.common.geometry import (
    backproject,
    estimate_normals,
    fit_cylinder_robust,
    fit_sphere_robust,
    grasp_frame_from_axis,
    polish_sphere_lm,
    project_point,
    transform_direction,
    transform_point,
)

from .contracts import (
    BagGrasp2D,
    BagGraspReference3D,
    BagObservation,
    compute_entry_start,
    compute_travel_range,
    TOOL_GEOMETRY,
    ToolGeometry,
)
from .identity import estimate_pose_covariance
from .image_gates import clip_bbox, foreground_mask, valid_depth_mask

# W3 迁移兼容 re-export：grasp_frame_from_axis 已迁 common.geometry
# （identity.memory_grasp 与本模块共用；迁出消除 identity↔pose_pipelines
# 循环依赖）。旧 import 路径保持可用，调用方逐步改走 geometry。
__all__ = [
    'RobustBagPosePipeline',
    'RobustFruitPosePipeline',
    'TargetPoseResult',
    'apply_transform_to_reference',
    'grasp_frame_from_axis',
    'make_pipeline',
    'PIPELINES_BY_IMPL',
]


def apply_transform_to_reference(grasp_3d, T: np.ndarray) -> None:
    """
    抓取几何由相机系变到输出系（默认 base_link），原地修改 grasp_3d.

    T 为 4×4 齐次矩阵（输出系←相机系）。规则：点 R@p+t（含 entry_start /
    bag_bottom / bag_neck / suggested_travel_end / legacy position /
    points_centroid，走 common.geometry transform_point）；方向只乘 R 并
    归一化（transform_direction：平移不影响方向）；姿态矩阵左乘 R。None
    字段原样保留。（W3 起 `_apply_T_to_grasp3d` 公开为本名，pipeline 不再
    跨模块 import 私有符号。）

    Args:
        grasp_3d: BagGraspReference3D（相机光学系，米）；被原地改写.
        T: (4, 4) 齐次矩阵，输出系←相机系.

    Returns
    -------
        None（结果写回 grasp_3d）.

    """
    # 行程终点、legacy position 与身份锚点（前景点云质心）也是点，必须同步
    # 变换（漏改会让 markers 的行程箭头终点留在相机系，与输出系几何错位；
    # 质心漏改则身份锚点掉到相机系，匹配半径在世界系下失真）
    grasp_3d.entry_start = transform_point(T, grasp_3d.entry_start)
    grasp_3d.bag_bottom = transform_point(T, grasp_3d.bag_bottom)
    grasp_3d.bag_neck = transform_point(T, grasp_3d.bag_neck)
    grasp_3d.suggested_travel_end = transform_point(T, grasp_3d.suggested_travel_end)
    grasp_3d.position = transform_point(T, grasp_3d.position)
    grasp_3d.points_centroid = transform_point(T, grasp_3d.points_centroid)
    grasp_3d.translation_direction = transform_direction(
        T, grasp_3d.translation_direction)
    if grasp_3d.orientation is not None:
        grasp_3d.orientation = (
            T[:3, :3] @ np.asarray(grasp_3d.orientation, dtype=float))


@dataclass
class TargetPoseResult:
    """单个目标的估计结果；只索引本检测，绝不混入其他目标."""

    target_id: str                    # 目标 ID（如 'target_0' / 'frame:idx'）
    grasp_2d: BagGrasp2D              # 图像平面参考（像素坐标）
    grasp_3d: BagGraspReference3D     # 3D 抓取参考（相机光学系，米）
    mask_source: str                  # 前景掩膜来源标签（诊断追溯用）
    metrics: dict                     # 诊断指标；缺项约定 None（消息层填 -1）
    target_kind: str = 'bag'  # "bag" | "fruit"


# 信息类 flags：描述估计过程（轴来源/极性校正/辅助先验），不反映几何
# 质量，不参与单帧门。2026-09-21 前所有 flags 一律压 ACCEPT（生产 56/56
# 恒 REOBSERVE、refine 的 ACCEPT 偏好分支死码）；分离后 ACCEPT=无门控类
# flag 可达，REJECT/REOBSERVE 语义不变。
_INFORMATIONAL_FLAGS = frozenset({
    'axis_from_pca',              # 轴来自 PCA 而非圆柱 RANSAC（过程事实）
    'axis_from_profile_sign',     # 剖面符号退化回主轴（过程事实）
    'taper_neck',                 # 锥度判口成功（过程事实）
    'taper_polarity_swapped',     # 宽窄端已对调（校正完成）
    'taper_lower_hemisphere_ignored',
    'polarity_upper_hemisphere',  # 上半球夹持已生效（校正完成）
    'fruit_prior_auxiliary',      # 球先验仅为辅助量（按构造恒在）
    'neck_from_band', 'bottom_from_band',
    'gravity_defaulted',          # 无重力提示回退默认（下游另有不确定度门）
    'length_extended_from_2d',    # 袋长经 2D 掩膜展程补全（2D+3D 融合）
})


def _gating_flags(flags) -> list:
    """过滤出参与单帧门的 flags（全集减信息类）."""
    return [f for f in flags if f not in _INFORMATIONAL_FLAGS]


def axis_length_consistent(length_m: float, bbox_w_px: int, bbox_h_px: int,
                           z_med_m: float, fx_px: float) -> bool:
    """
    3D 袋长与检测框 2D 展程一致性（长度塌缩显式化门）.

    背景：地标/重力极性钳制在重力方向失准时会把 bottom→neck 长度压塌
    （2026-09-20 生产 56 帧 travel 0.024-0.040m ⟹ 隐含袋长 ~4-5.5cm，
    物理 ~15cm+），旧口径下只表现为 travel_too_short 恒命中、根因不可
    见。本门用 2D 展程×深度/焦距的独立粗测交叉验证：3D 长度不足粗测
    一半且粗测本身不退化（>8cm）时判不一致。

    Args:
        length_m: 估计的 3D 袋长（米）.
        bbox_w_px: 检测框宽（像素）.
        bbox_h_px: 检测框高（像素）.
        z_med_m: 前景点中位深度（米）.
        fx_px: 相机焦距（像素）.

    Returns
    -------
        True = 一致或粗测无判别力；False = 长度塌缩嫌疑.

    """
    if fx_px <= 0.0 or z_med_m <= 0.0:
        return True
    implied_m = float(np.hypot(bbox_w_px, bbox_h_px)) * z_med_m / fx_px
    if implied_m <= 0.08:
        return True
    return length_m >= 0.5 * implied_m


def mask_axial_length_px(raw_mask: np.ndarray, bottom_px, dir2d,
                         xoff: int, yoff: int) -> Optional[tuple]:
    """
    原始 SAM 掩膜像素沿投影轴方向相对袋底的两端展程（像素，1/99 分位）.

    2D+3D 融合用：几何点云只覆盖「有有效深度的段」，SAM 掩膜覆盖 RGB
    可见的整段（含深度空洞/被浅遮挡部分）。返回 (t_lo, t_hi)：t_hi 为
    颈向展程、t_lo 为底向展程（负值），分别钳制两端延伸。

    Args:
        raw_mask: ROI 尺寸原始 SAM 掩膜（未经深度门控；None/空给 None）.
        bottom_px: 袋底 3D 投影像素（全图坐标，向内取 ROI 局部）.
        dir2d: 投影轴 2D 单位方向（底→颈）.
        xoff: ROI 在全图的 x 偏移（像素）.
        yoff: ROI 在全图的 y 偏移（像素）.

    Returns
    -------
        (t_lo, t_hi) 像素展程；掩膜过小/展程退化给 None.

    """
    if raw_mask is None:
        return None
    m = np.asarray(raw_mask, dtype=bool)
    ys, xs = np.nonzero(m)
    if xs.size < 30:
        return None
    rel_x = xs - (float(bottom_px[0]) - float(xoff))
    rel_y = ys - (float(bottom_px[1]) - float(yoff))
    t = rel_x * dir2d[0] + rel_y * dir2d[1]
    t_lo, t_hi = np.percentile(t, [1.0, 99.0])
    if float(t_hi - t_lo) < 12.0:
        return None
    return float(t_lo), float(t_hi)


@dataclass
class _EstimateInputs:
    """estimate 同构前奏产物（袋/果共用；W13-A 抽取，字段见 _prepare_estimate_inputs）."""

    x1: int
    """ROI 左上 x（像素）."""
    y1: int
    """ROI 左上 y（像素）."""
    x2: int
    """ROI 右下 x（像素）."""
    y2: int
    """ROI 右下 y（像素）."""
    roi: np.ndarray
    """(h, w) uint16 ROI 深度（毫米）."""
    base_2d: BagGrasp2D
    """2D 结果骨架（foreground_mask 已填）."""
    local_mask: np.ndarray
    """(h, w) bool ROI 前景掩膜."""
    valid_ratio: float
    """ROI 有效深度占比."""
    coverage: float
    """ROI 前景掩膜占比."""
    points: np.ndarray
    """(N, 3) float64 相机系点（米，已剔离群）."""
    pixels: np.ndarray
    """(N, 2) int ROI 内像素坐标 (x, y)."""
    source: str
    """掩膜来源标签."""
    gravity: np.ndarray
    """(3,) 单位重力方向（相机系）."""


class RobustBagPosePipeline:
    """
    袋装桃的保守位姿估计器（圆柱套入工具）.

    所有安全判定只用实测深度。优先使用外部实例掩膜（SAM），
    深度带连通域是显式、可检查的降级来源。
    """

    kind = 'bag'

    def __init__(self, tool: ToolGeometry = TOOL_GEOMETRY, min_depth_m=0.3,
                 max_depth_m=2.5, min_points=100):
        """
        构造袋装管线.

        Args:
            tool: 工具几何（默认台架测量值 TOOL_GEOMETRY）.
            min_depth_m: 有效深度下限 (m)，过近视为噪声.
            max_depth_m: 有效深度上限 (m)，过远视为背景.
            min_points: 有效前景点数下限，不足直接 REJECT.

        Returns
        -------
            无返回值（None）.

        """
        self.tool = tool
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.min_points = min_points

    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback',
                 valid_roi: Optional[np.ndarray] = None,
                 target_valid_ratio: Optional[float] = None,
                 raw_mask: Optional[np.ndarray] = None) -> TargetPoseResult:
        """
        估计 bbox 内单个袋装目标，返回显式安全状态的结果.

        Args:
            obs: 单帧输入（深度 uint16 毫米；gravity_hint 为相机系方向或 None）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素，自动裁剪到图内）.
            mask: 外部前景掩膜（全图或 ROI，bool/0-1）；None 走深度带降级.
            mask_source: 掩膜来源标签，写入诊断.
            valid_roi: 与 ROI 同尺寸的有效深度掩膜；None 时按管线深度窗现算.
            target_valid_ratio: 目标级有效深度占比 |SAM∩valid|/|SAM|
                （build_masks 的 sam_yield）；None 回退旧 ROI 均值口径
                （外部直调/降级路径）。该值同时驱动 low_valid_depth 门、
                confidence 与 σ 缩放、遮挡分类.
            raw_mask: ROI 尺寸的原始 SAM 掩膜（未经深度门控）。给定时做
                2D+3D 长度融合：几何点云只覆盖有有效深度的段，掩膜沿投
                影轴的像素展程×中位深度/焦距可补全被深度空洞/浅遮挡截
                断的部分（只延伸不回缩，打 length_extended_from_2d 信
                息类 flag；投影退化/展程不足时跳过）.

        Returns
        -------
            TargetPoseResult；status ∈ ACCEPT/REOBSERVE/REJECT。门控语义
            （2026-09-21 起）：ACCEPT=无门控类 flag（信息类见
            _INFORMATIONAL_FLAGS 不再压状态）；硬性失败直接 REJECT。

        与 RobustFruitPosePipeline.estimate 同构（W13-A）：前奏共用
        _prepare_estimate_inputs；中段（轴估计/参考点/门控）与结果组装
        按袋线工艺分线，本轮未并成模板方法——轴来源（圆柱 RANSAC+袋
        地标）与果线（球拟合+梗洼）分支、flags/metrics 字段集均不同且
        纠缠，回放塔要求逐字节保持。改本方法时对照果线同名方法。

        """
        failure, inputs = self._prepare_estimate_inputs(
            obs, target_id, bbox, mask, mask_source, valid_roi,
            target_valid_ratio)
        if failure is not None:
            return failure
        x1, y1, x2, y2 = inputs.x1, inputs.y1, inputs.x2, inputs.y2
        roi = inputs.roi
        base_2d = inputs.base_2d
        local_mask = inputs.local_mask
        valid_ratio = inputs.valid_ratio
        coverage = inputs.coverage
        points, pixels = inputs.points, inputs.pixels
        source = inputs.source
        gravity = inputs.gravity

        # ── 套入轴估计: 圆柱 RANSAC 主估 → 2D 掩膜校验 → 重力显式降级 ──
        # 理论: 圆柱法线 ⊥ 轴 (a = n₁×n₂)；局部几何结构良态，全局 PCA 对
        # 近回转体退化；重力假设仅在自由悬垂时成立
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
        # 轴符号定向: 底→颈；近水平只标不确定，不改成竖轴（斜袋口在侧）
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
        _bottom_px = self._project(bottom, obs.camera_K)
        _neck_px = self._project(neck, obs.camera_K)
        if _bottom_px is not None and _neck_px is not None:
            disagreement_deg = self._mask_axis_disagreement(
                local_mask, _bottom_px, _neck_px)

        radial = np.linalg.norm(transverse_points - transverse_center, axis=1)
        diameter = float(2.0 * np.percentile(radial, 95))
        bbox_aspect = float(max(x2 - x1, 1) / max(y2 - y1, 1))
        landmarks = estimate_bag_landmarks(
            points, gravity,
            mask_area_ratio=coverage, edge_touch=False,
            neighbor_gap_m=1.0, valid_depth_ratio=valid_ratio,
            bbox_aspect=bbox_aspect)
        landmark_flags = list(landmarks.flags)
        if (landmarks.neck_center is not None
                and landmarks.bottom_center is not None
                and landmarks.bag_axis is not None):
            bottom = landmarks.bottom_center
            neck = landmarks.neck_center
            axis = landmarks.bag_axis
            axis_source = 'bag_landmarks'
            if landmarks.d95_m > 0.0:
                diameter = float(landmarks.d95_m)
        bottom, neck, axis, width_flipped = enforce_wide_bottom(
            bottom, neck, axis, points)
        if width_flipped:
            landmark_flags.append('taper_polarity_swapped')
        bottom_px = self._project(bottom, obs.camera_K)
        neck_px = self._project(neck, obs.camera_K)
        if self._mask_axis_against_taper(local_mask, x1, y1, bottom_px, neck_px):
            flipped = -np.asarray(axis, dtype=float)
            if float(flipped @ gravity) <= 0.0:
                bottom, neck = neck, bottom
                axis = flipped
                if 'taper_polarity_swapped' not in landmark_flags:
                    landmark_flags.append('taper_polarity_swapped')
                landmark_flags.append('mask_bbox_flush_mouth')
        bottom, neck, axis, up_flipped = clamp_upper_hemisphere(
            bottom, neck, axis, gravity)
        if up_flipped:
            landmark_flags.append('polarity_upper_hemisphere')
        axial = (points - np.asarray(bottom, dtype=float)) @ np.asarray(
            axis, dtype=float)
        if axial.size >= 8:
            t_tip = float(np.percentile(axial, 98))
            if t_tip > 0.02:
                neck = np.asarray(bottom, dtype=float) + t_tip * np.asarray(
                    axis, dtype=float)
        # ── 2D+3D 融合补全（检测框限幅，09-21 用户定版）──
        # 深度空洞/遮挡让点云只覆盖袋体可见段，分位带长度系统性偏短
        # （活流对拍 len −29%）。2D 证据优先原始 SAM 掩膜剪影（
        # mask_axial_length_px，P99 展程）；无掩膜时退检测框角点展程。
        # 展程一律被检测框沿轴像素范围限幅（框是硬边界），另设绝对/
        # 相对护栏防误检框带飞长度；两端只延不缩。直径不融合——径向
        # P95 已稳健且直接进净空安全门，掺入框内叶片会假性收紧净空。
        fuse_flags = []
        _bpx = self._project(bottom, obs.camera_K)
        _npx = self._project(neck, obs.camera_K)
        if _bpx is not None and _npx is not None and length > 1e-4:
            u2d = np.array([_npx[0] - _bpx[0], _npx[1] - _bpx[1]],
                           dtype=float)
            span_px = float(np.linalg.norm(u2d))
            if span_px >= 5.0:
                u2d /= span_px
                px_per_m = span_px / length
                corners = np.array([
                    (x1, y1), (x2, y1), (x1, y2), (x2, y2)], dtype=float)
                tc = (corners - np.array([_bpx[0], _bpx[1]])) @ u2d
                hi_px = float(tc.max())
                lo_px = float(tc.min())
                mask_span = mask_axial_length_px(
                    raw_mask, _bpx, u2d, x1, y1)
                if mask_span is not None:
                    # 掩膜剪影优先作两端证据，检测框保持硬限幅
                    hi_px = min(hi_px, mask_span[1])
                    lo_px = max(lo_px, mask_span[0])
                cap = min(0.35, max(3.0 * length, length + 0.05))
                s_hi = min(hi_px / px_per_m, cap)
                s_lo = max(lo_px / px_per_m, -cap)
                if s_hi > length + 0.005:
                    neck = (np.asarray(bottom, dtype=float)
                            + s_hi * np.asarray(axis, dtype=float))
                    fuse_flags.append('length_extended_from_2d')
                if s_lo < -0.005:
                    bottom = (np.asarray(bottom, dtype=float)
                              + s_lo * np.asarray(axis, dtype=float))
                    fuse_flags.append('length_extended_from_2d')
        length = float(np.dot(neck - bottom, axis))

        # ── entry_start = P_bottom − (d_tool + d_s)·axis (Gürsoy 分解) ──
        standoff = self.tool.entry_standoff
        entry = compute_entry_start(bottom, axis, standoff)
        _, travel = compute_travel_range(entry, neck, axis, self.tool)
        R = self._frame(axis, points)

        # ── 误差预算: δ = L·sin(θ_err) ≤ 径向余量 ──
        # θ_err 代理: 拟合残差/袋长（指向误差）+ 定向与 2D 校验惩罚。
        # 注意: 圆形掩膜的 2D 主轴无意义，disagreement 仅在掩膜细长时生效。
        if cyl is not None and axis_source == 'cylinder_ransac':
            theta_err_deg = float(np.clip(
                np.degrees(np.arctan2(2.0 * cyl['rms'], max(length, 0.03))),
                2.0, 30.0))
        if orientation_uncertain:
            theta_err_deg = max(theta_err_deg, 12.0)
        radial_clearance = self.tool.d_inner_m / 2.0 - diameter / 2.0 - self.tool.clearance_min
        budget_m = (standoff + travel) * np.sin(np.radians(theta_err_deg))

        flags = []
        flags.extend(landmark_flags)
        flags.extend(fuse_flags)
        if valid_ratio < 0.40:
            flags.append('low_valid_depth')
        if coverage < 0.01:
            flags.append('small_foreground')
        if diameter + 2.0 * self.tool.clearance_min >= self.tool.d_inner_m:
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
        # 长度塌缩显式化：3D 袋长 vs 检测框 2D 展程×深度/焦距交叉验证
        # （生产长度塌缩只变现为 travel_too_short 恒命中、根因不可见）
        if not axis_length_consistent(
                length, x2 - x1, y2 - y1, float(np.median(points[:, 2])),
                float(obs.camera_K.get('fx', 0.0))):
            flags.append('axis_length_inconsistent')

        gating = _gating_flags(flags)
        status = 'ACCEPT' if not gating else (
            'REJECT' if 'tool_clearance_failed' in flags else 'REOBSERVE')
        confidence = float(np.clip(
            min(valid_ratio / 0.65, 1.0) * min(len(points) / 800.0, 1.0)
            * (1.0 - min(boundary_touch, 0.8)), 0.0, 1.0))
        base_2d.bottom_px = self._project(bottom, obs.camera_K)
        base_2d.neck_px = self._project(neck, obs.camera_K)
        base_2d.grasp_px = self._project(entry, obs.camera_K)
        base_2d.bag_axis_line = [base_2d.bottom_px, base_2d.neck_px]
        # 紫线终点 = 袋口（分割/检测框极限），不走果包络与袋口的中点。
        base_2d.travel_line = [base_2d.grasp_px, base_2d.neck_px]
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
        pos_cov, dir_cov = estimate_pose_covariance(
            points, axis, float(theta_err_deg))
        grasp_3d = BagGraspReference3D(
            frame_id=obs.frame_id, entry_start=entry, position=entry,
            points_centroid=0.5 * (np.asarray(bottom) + np.asarray(neck)),
            orientation=R, bag_bottom=bottom, bag_neck=neck,
            translation_direction=axis, bag_diameter_upper_m=diameter,
            suggested_travel_m=travel, suggested_travel_end=neck,
            position_covariance=pos_cov, direction_covariance=dir_cov,
            confidence=confidence, status=status, diagnostic_flags=flags,
            diagnostic_info={**metrics, 'mask_source': source,
                             'axis_source': axis_source,
                             'occlusion_class': landmarks.occlusion_class,
                             'fruit_prior_radius_m': landmarks.fruit_prior_radius_m,
                             'd95_m': diameter,
                             'sigma_position_m': landmarks.sigma_position_m,
                             'sigma_axis_deg': landmarks.sigma_axis_deg,
                             'D_bag_mm': f'{diameter * 1000:.0f}'},
            strategy_id='robust_bag_pose',
            model_version=str(obs.metadata.get('model_version', 'unknown')),
            calibration_version=str(obs.metadata.get(
                'calibration_version', 'unknown')),
            tool_version=self.tool.version)
        return TargetPoseResult(target_id, base_2d, grasp_3d, source, metrics)

    def _prepare_estimate_inputs(self, obs, target_id, bbox, mask,
                                 mask_source, valid_roi,
                                 target_valid_ratio=None):
        """
        袋/果两线 estimate 的同构前奏（W13-A 抽取；步骤序与原内联逐字一致）.

        裁框 → 建 BagGrasp2D → 尺寸门 → 深度窗/前景掩膜 → 反投影点云
        → MAD 离群剔除 → 点数门 → 重力解析归一。任一门失败即返回本线
        （self.kind）的 REJECT 结果；成功返回前奏产物供两线各自的
        中段（轴估计/参考点/门控）继续。

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素，自动裁剪到图内）.
            mask: 外部前景掩膜（全图或 ROI）；None 走深度带降级.
            mask_source: 掩膜来源标签，写入诊断.
            valid_roi: 与 ROI 同尺寸的有效深度掩膜；None 按管线深度窗现算.
            target_valid_ratio: 目标级有效深度占比（见 estimate 文档）；
                None 回退 ROI 均值（旧口径，外部直调/降级路径保留）.

        Returns
        -------
            (failure, inputs)：failure 非 None 时调用方直接返回该 REJECT
            结果；inputs 为 _EstimateInputs（failure 非 None 时为 None）.

        """
        x1, y1, x2, y2 = self._clip_bbox(bbox, obs.depth.shape)
        base_2d = BagGrasp2D(detection_bbox=(x1, y1, x2 - x1, y2 - y1))
        if x2 - x1 < 8 or y2 - y1 < 8:
            return self._failed(
                target_id, base_2d, 'invalid_bbox', mask_source), None

        roi = obs.depth[y1:y2, x1:x2]
        valid = valid_roi if valid_roi is not None else self._valid_depth(roi)
        # valid_ratio 语义（2026-09-21 修正）：优先目标级 |SAM∩valid|/|SAM|
        # （sam_yield）。旧 ROI 均值把检测框背景超窗算进分母，实测被压到
        # 掩膜占比量级（0.09 vs 目标真实 ~0.9），连带压垮 confidence、
        # σ_position（膨胀 ~5×）与遮挡分类；目标级口径三者同步回到真实。
        if target_valid_ratio is not None:
            valid_ratio = float(np.clip(target_valid_ratio, 0.0, 1.0))
        else:
            valid_ratio = float(valid.mean()) if valid.size else 0.0
        local_mask, source = self._foreground(roi, valid, mask, bbox, source=mask_source)
        base_2d.foreground_mask = local_mask
        coverage = float(local_mask.mean()) if local_mask.size else 0.0
        points, pixels = self._to_points(
            roi, local_mask, x1, y1, obs.camera_K, valid)
        points, pixels = self._filter_depth_outliers(points, pixels)
        if len(points) < self.min_points:
            return self._failed(
                target_id, base_2d, 'insufficient_measured_points', source,
                valid_depth_ratio=valid_ratio, foreground_ratio=coverage,
                n_points=len(points)), None

        gravity = np.asarray(obs.gravity_hint if obs.gravity_hint is not None
                             else [0.0, 1.0, 0.0], dtype=float)
        if np.linalg.norm(gravity) < 1e-8:
            return self._failed(
                target_id, base_2d, 'invalid_gravity', source,
                valid_depth_ratio=valid_ratio, foreground_ratio=coverage,
                n_points=len(points)), None
        gravity /= np.linalg.norm(gravity)
        inputs = _EstimateInputs(
            x1=x1, y1=y1, x2=x2, y2=y2, roi=roi, base_2d=base_2d,
            local_mask=local_mask, valid_ratio=valid_ratio, coverage=coverage,
            points=points, pixels=pixels, source=source, gravity=gravity)
        return None, inputs

    def _failed(self, target_id, grasp_2d, reason, source, **metrics):
        """
        构造 REJECT 结果（袋/果两线共用；W13-A 合并原 _failed_fruit）.

        strategy_id 与 target_kind 均按 self.kind 推导（'bag' →
        'robust_bag_pose' / target_kind='bag'；'fruit' → 'robust_fruit_pose'
        / target_kind='fruit'），两线失败产物与合并前逐字段一致。

        Args:
            target_id: 目标 ID.
            grasp_2d: 已建的 BagGrasp2D（被改写为 REJECT）.
            reason: 失败原因标记（写入 diagnostic_flags）.
            source: 掩膜来源标签.
            **metrics: 已采集的诊断指标，原样透传.

        Returns
        -------
            TargetPoseResult（target_kind=self.kind，status=REJECT）.

        """
        grasp_2d.status = 'REJECT'
        grasp_2d.diagnostic_flags = [reason]
        grasp_3d = BagGraspReference3D(
            status='REJECT', diagnostic_flags=[reason],
            strategy_id=f'robust_{self.kind}_pose',
            tool_version=self.tool.version,
            diagnostic_info={**metrics, 'mask_source': source})
        return TargetPoseResult(target_id, grasp_2d, grasp_3d, source, metrics,
                                target_kind=self.kind)

    @staticmethod
    def _clip_bbox(bbox, shape):
        """检测框裁剪到图像范围内（委托模块级 :func:`clip_bbox`）."""
        return clip_bbox(bbox, shape)

    def _valid_depth(self, depth):
        """
        有效深度掩膜（委托模块级 :func:`valid_depth_mask`）.

        有效区间取本管线构造参数 min_depth_m / max_depth_m。
        """
        return valid_depth_mask(depth, self.min_depth_m, self.max_depth_m)

    def _foreground(self, depth, valid, supplied_mask, bbox, source):
        """前景掩膜（委托模块级 :func:`foreground_mask`，语义不变）."""
        return foreground_mask(depth, valid, supplied_mask, bbox, source)

    @staticmethod
    def _to_points(depth, mask, xoff, yoff, K, valid):
        """
        前景像素反投影为相机系 3D 点（米），委托 geometry.backproject.

        W3：距离窗不再在此重算——直接复用 estimate 已解析的 `valid`
        掩膜（valid_roi 或管线深度窗，同一 valid_depth_mask 口径）；
        反投影数学单源在 common.geometry.backproject。
        （W13-A：原声明 self 但函数体零引用，改为静态方法；调用点
        self._to_points(...) / 实例直调均不受影响。）

        Args:
            depth: (h, w) uint16 ROI 深度（毫米）.
            mask: (h, w) bool 前景掩膜（内部再与 valid 求交）.
            xoff: ROI 在全图的 x 像素偏移.
            yoff: ROI 在全图的 y 像素偏移.
            K: 相机内参 {"fx","fy","cx","cy"}.
            valid: (h, w) bool 有效深度掩膜（estimate 传入）.

        Returns
        -------
            (points, pixels)：points 为 (N, 3) float 相机系坐标（米），
            pixels 为 (N, 2) int ROI 内像素坐标 (x, y).

        """
        selected = np.asarray(mask, dtype=bool) & valid
        points, _ = backproject(depth, selected, K, xoff=xoff, yoff=yoff)
        ys, xs = np.nonzero(selected)
        return points, np.column_stack((xs, ys))

    @staticmethod
    def _filter_depth_outliers(points, pixels):
        """
        按 z 的 MAD 剔除离群点（|z−中位| > 3.5·MAD）.

        Args:
            points: (N, 3) 点（米）.
            pixels: (N, 2) 与 points 对齐的像素.

        Returns
        -------
            (过滤后 points, 过滤后 pixels)；N<10 或 MAD≈0 时原样返回.

        """
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
        """
        前景触边统计：掩膜触边视为 ROI 裁切/遮挡信号.

        Args:
            mask: (h, w) bool ROI 前景掩膜.

        Returns
        -------
            (touch_ratio, sides)：四条边上前景占比的均值，以及占比 >5% 的
            边数 (0–4)；空掩膜给 (1.0, 4)（最保守）.

        """
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
        """由袋轴构造右手抓取系 R = [Xg, Yg, Zg]（Zg=axis）."""
        centred = points - points.mean(axis=0)
        _, vec = np.linalg.eigh(centred.T @ centred / max(len(points), 1))
        x = vec[:, -1] - np.dot(vec[:, -1], axis) * axis
        if np.linalg.norm(x) >= 1e-8:
            x /= np.linalg.norm(x)
            if x[0] < 0:
                x = -x
            y = np.cross(axis, x)
            y /= np.linalg.norm(y)
            return np.column_stack((x, y, axis))
        return grasp_frame_from_axis(axis)

    @staticmethod
    def _mask_axis_against_taper(
            mask: np.ndarray, x1: int, y1: int, bottom_px, neck_px) -> bool:
        """
        口在沿轴朝外更贴检测框边的那一端（如左边竖缝贴左框）.

        竖缝垂直方向很长，两半宽度会把口判成宽头。果鼓贴框底时，
        到四边最短距也会两端都贴。只比各端朝外那条框边。True = 当前
        底比口更贴朝外边，对调.
        """
        if (mask is None or mask.size == 0 or bottom_px is None
                or neck_px is None):
            return False
        ys, xs = np.where(mask > 0)
        if xs.size < 30:
            return False
        height, width = mask.shape[:2]
        origin = np.array(
            [float(bottom_px[0]) - float(x1),
             float(bottom_px[1]) - float(y1)], dtype=float)
        tip = np.array(
            [float(neck_px[0]) - float(x1),
             float(neck_px[1]) - float(y1)], dtype=float)
        axis_2d = tip - origin
        span = float(np.linalg.norm(axis_2d))
        if span < 8.0:
            return False
        axis_2d /= span
        pts = np.column_stack((xs.astype(float), ys.astype(float)))
        along = (pts - origin) @ axis_2d
        band = 0.25 * span

        # [lo,hi] 沿轴带内前景像素到 outward 所指图边的中位距离；点太少返回
        # inf（视为不贴边）。返回值 = 当前判的底端比口端更贴朝外边（差 >3 px），
        # True 则按「贴边端为口」对调（语义见 _mask_axis_against_taper docstring）。
        def _outward_flush(lo: float, hi: float, outward) -> float:
            selected = pts[(along >= lo) & (along <= hi)]
            if selected.shape[0] < 8:
                return float('inf')
            left = selected[:, 0]
            right = (width - 1.0) - selected[:, 0]
            top = selected[:, 1]
            bottom = (height - 1.0) - selected[:, 1]
            edges = (
                (np.array([-1.0, 0.0]), left),
                (np.array([1.0, 0.0]), right),
                (np.array([0.0, -1.0]), top),
                (np.array([0.0, 1.0]), bottom),
            )
            dist = max(edges, key=lambda item: float(np.dot(outward, item[0])))[1]
            return float(np.median(dist))

        flush_bottom = _outward_flush(-0.05 * span, band, -axis_2d)
        flush_neck = _outward_flush(span - band, span * 1.05, axis_2d)
        if not (np.isfinite(flush_bottom) and np.isfinite(flush_neck)):
            return False
        return flush_bottom + 3.0 < flush_neck

    @staticmethod
    def _mask_axis_disagreement(mask: np.ndarray, bottom_px: tuple,
                                neck_px: tuple) -> Optional[float]:
        """
        2D 校验: 前景掩膜 PCA 主轴 与 袋轴投影(底→颈像素) 的无向夹角 (deg).

        对极点/轴向错误的廉价交叉验证（Kok 2024 的 180° 尾部教训）：
        3D 拟合轴投影到图像后与掩膜 2D 主轴夹角过大，说明拟合可疑。

        Args:
            mask: (h, w) bool ROI 前景掩膜.
            bottom_px: 袋底投影像素 (u, v).
            neck_px: 袋颈投影像素 (u, v).

        Returns
        -------
            夹角 (deg, 0–90)；掩膜点太少/投影过短/掩膜近圆形等无判别力
            情形返回 None.

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
        """
        相机系 3D 点 → 像素 (u, v)（W13-A 起委托 geometry.project_point）.

        Args:
            point: (3,) 点（米）；None 或 z≤1e-8 给 None.
            K: 相机内参 {"fx","fy","cx","cy"}.

        Returns
        -------
            (u, v) float 像素；不可投影返回 None.

        """
        return project_point(point, K)


# ═══════════════════════════════════════════════════════════════
# 裸果管线 (class 1, peach_nobag) — 同一圆柱剪切工具的第二条流程线
# ═══════════════════════════════════════════════════════════════

class RobustFruitPosePipeline(RobustBagPosePipeline):
    """
    裸果桃位姿估计器（同一圆柱剪切工具）.

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

    kind = 'fruit'

    CAVITY_HALF_ANGLE_COS = np.cos(np.radians(20.0))
    CAVITY_MIN_DIP_M = 0.0015     # 梗洼最小下陷深度 1.5mm
    CAVITY_MIN_POINTS = 12

    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback',
                 valid_roi: Optional[np.ndarray] = None,
                 target_valid_ratio: Optional[float] = None
                 ) -> TargetPoseResult:
        """
        估计 bbox 内单个裸果目标，返回显式安全状态的结果.

        Args:
            obs: 单帧输入（深度 uint16 毫米；gravity_hint 为相机系方向或 None）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素，自动裁剪到图内）.
            mask: 外部前景掩膜（全图或 ROI）；None 走深度带降级.
            mask_source: 掩膜来源标签，写入诊断.
            valid_roi: 与 ROI 同尺寸的有效深度掩膜；None 时按管线深度窗现算.
            target_valid_ratio: 目标级有效深度占比（见袋线同名参数文档）.

        Returns
        -------
            TargetPoseResult（target_kind='fruit'）；metrics 追加
            fruit_radius_m / sphere_rms_m / sphere_inlier_ratio /
            cavity_dip_mm / axis_polarity_corrected（球拟合失败时前三项为 None）.

        与 RobustBagPosePipeline.estimate 同构（W13-A）：前奏共用
        _prepare_estimate_inputs；中段（球拟合+梗洼定向）与结果组装按
        果线工艺分线，本轮未并成模板方法——flags/metrics/参考点公式
        与袋线（圆柱 RANSAC+袋地标）纠缠，回放塔要求逐字节保持。
        改本方法时对照袋线同名方法。

        """
        failure, inputs = self._prepare_estimate_inputs(
            obs, target_id, bbox, mask, mask_source, valid_roi,
            target_valid_ratio)
        if failure is not None:
            return failure
        # 果线后段只消费 x1/y1（estimate_normals ROI 偏移），不引用 x2/y2
        x1, y1 = inputs.x1, inputs.y1
        roi = inputs.roi
        base_2d = inputs.base_2d
        local_mask = inputs.local_mask
        valid_ratio = inputs.valid_ratio
        coverage = inputs.coverage
        points, pixels = inputs.points, inputs.pixels
        source = inputs.source
        gravity = inputs.gravity

        normals_map, nvalid_map = estimate_normals(roi, x1, y1, obs.camera_K)
        pnormals = normals_map[pixels[:, 1], pixels[:, 0]]
        pnvalid = nvalid_map[pixels[:, 1], pixels[:, 0]]

        # ── 球拟合定心定径 (点+法线 RANSAC + 几何抛光) ──
        # fit 的 inliers 是相对「法线有效子集」的下标，内点取点必须用同一子集
        valid_pts, valid_nrm = points[pnvalid], pnormals[pnvalid]
        sph = fit_sphere_robust(valid_pts, valid_nrm,
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
            inl_pts = valid_pts[sph['inliers']]
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
        standoff = self.tool.entry_standoff
        entry = compute_entry_start(bottom, axis, standoff)
        _, travel = compute_travel_range(entry, neck, axis, self.tool)
        R = self._frame(axis, points)

        # ── 误差预算（与袋装线同公式） ──
        if orientation_uncertain:
            theta_err_deg = max(theta_err_deg, 12.0)
        radial_clearance = self.tool.d_inner_m / 2.0 - diameter / 2.0 - self.tool.clearance_min
        budget_m = (standoff + travel) * np.sin(np.radians(theta_err_deg))

        disagreement_deg = None
        _bottom_px = self._project(bottom, obs.camera_K)
        _neck_px = self._project(neck, obs.camera_K)
        if _bottom_px is not None and _neck_px is not None:
            disagreement_deg = self._mask_axis_disagreement(
                local_mask, _bottom_px, _neck_px)

        flags = []
        flags.append('unbagged_display_only')
        if valid_ratio < 0.40:
            flags.append('low_valid_depth')
        if coverage < 0.01:
            flags.append('small_foreground')
        if diameter + 2.0 * self.tool.clearance_min >= self.tool.d_inner_m:
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

        # 果线门控：unbagged_display_only 是硬性「仅显示」标记，保持门控
        # （不入 _INFORMATIONAL_FLAGS），果线至多 REOBSERVE 的设计不变。
        gating = _gating_flags(flags)
        status = 'ACCEPT' if not gating else (
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
        pos_cov, dir_cov = estimate_pose_covariance(
            points, axis, float(theta_err_deg))
        grasp_3d = BagGraspReference3D(
            frame_id=obs.frame_id, entry_start=entry, position=entry,
            points_centroid=np.median(points, axis=0),
            orientation=R, bag_bottom=bottom, bag_neck=neck,
            translation_direction=axis, bag_diameter_upper_m=diameter,
            suggested_travel_m=travel, suggested_travel_end=entry + travel * axis,
            position_covariance=pos_cov, direction_covariance=dir_cov,
            confidence=confidence, status=status, diagnostic_flags=flags,
            diagnostic_info={**metrics, 'mask_source': source,
                             'axis_source': axis_source,
                             'D_bag_mm': f'{diameter * 1000:.0f}'},
            strategy_id='robust_fruit_pose',
            model_version=str(obs.metadata.get('model_version', 'unknown')),
            calibration_version=str(obs.metadata.get(
                'calibration_version', 'unknown')),
            tool_version=self.tool.version)
        return TargetPoseResult(target_id, base_2d, grasp_3d, source, metrics,
                                target_kind='fruit')

    def _stem_cavity_axis(self, points: np.ndarray, center: np.ndarray,
                          radius: float) -> Tuple[Optional[np.ndarray], float]:
        """
        梗洼定向: 拟合球面上径向残差下陷最深的方向帽 = 果梗方向.

        理论: 桃果梗附着处凹陷（植物学形态）；拟合球面是参考面，表面点
        相对球面的径向残差 d_i − r 在梗端方向帽内显著为负。打分取 P30
        分位以容忍扫描帽大于真实洼区的稀释。零标注局部几何原语，
        只对可见半球有效（无点方向自动跳过）。

        Args:
            points: (N, 3) 球内点（相机系，米）.
            center: (3,) 拟合球心（米）.
            radius: 拟合半径（米）.

        Returns
        -------
            (axis, dip)：axis 为 (3,) 单位梗端方向（找不到给 None），
            dip 为最深帽的 P30 径向残差（米，负值表示下陷；未找到给 0.0）.

        """
        rel = points - center
        d = np.linalg.norm(rel, axis=1)
        ok = d > 1e-6
        if ok.sum() < 30:
            return None, 0.0
        u = rel[ok] / d[ok, None]
        residuals = d[ok] - radius

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
            dip = float(np.percentile(residuals[sel], 30))
            if dip < best_dip:
                best_dip, best_i = dip, i
        if best_i < 0 or best_dip > -self.CAVITY_MIN_DIP_M:
            return None, 0.0
        # 方向精化: 帽内点残差加权平均方向
        sel = sim[best_i] >= self.CAVITY_HALF_ANGLE_COS
        w = np.clip(-(residuals[sel]), 0.0, None) + 1e-6
        refined = (u[sel] * w[:, None]).sum(axis=0)
        refined /= np.linalg.norm(refined)
        return refined, best_dip


# 袋/果两条位姿线的实现映射（yaml pipeline.bag_impl / fruit_impl；未知名
# 列出全部可用名后失败）。新增第三条线 = 加一个类 + 这里一项。
PIPELINES_BY_IMPL = {
    'robust_bag': RobustBagPosePipeline,
    'robust_fruit': RobustFruitPosePipeline,
}


def make_pipeline(impl_name: str, **kwargs):
    """按 yaml 实现名构造位姿管线；未知名列出全部可用名后抛错."""
    cls = PIPELINES_BY_IMPL.get(impl_name)
    if cls is None:
        raise ValueError(
            f'未知位姿管线实现 {impl_name!r}，可用: '
            f'{sorted(PIPELINES_BY_IMPL)}')
    return cls(**kwargs)
