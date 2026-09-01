from __future__ import annotations
"""检测/分割/前景/位姿管线（看场景算法核）。"""

from dataclasses import dataclass
import logging
import math
import threading
import time
from typing import (
    Dict,
    Iterable,
    List,
    Optional,
    Sequence,
    Tuple,
)

import cv2
from geometry_msgs.msg import Quaternion
import numpy as np
from peach_perception.common.geometry import (
    estimate_normals,
    fit_cylinder_robust,
    fit_sphere_robust,
    polish_sphere_lm,
    rotation_to_quat,
    transform_direction,
    transform_point,
)

from .bag_landmarks import (
    clamp_upper_hemisphere,
    enforce_wide_bottom,
    estimate_bag_landmarks,
)
from .interfaces import (
    BagGrasp2D,
    BagGraspReference3D,
    BagObservation,
    compute_entry_start,
    compute_travel_range,
    Detector,
    DETECTORS,
    POSE_PIPELINES,
    PosePipeline,
    Segmenter,
    SEGMENTERS,
    TOOL_GEOMETRY,
    ToolGeometry,
)


# === timing.py ===

class RateEstimator:
    """
    帧/事件间隔 EMA 估计器（帧率以运行状态为准）.

    异常间隔过滤沿用 peach_scene_perception_node 帧率 EMA 纪律：间隔 ≤1ms（同帧
    重复/时钟噪声）或 >30s（暂停后首帧/时钟跳变）不进 EMA，防污染
    估计；但「上次时刻」始终更新，保证暂停恢复后下一帧间隔重新有效。

    生命周期：构造后可长期持有，随每个事件调用 update；无重置需求
    （EMA 自然跟踪缓变）。线程安全：无内部锁，单写者使用。
    """

    def __init__(self, alpha: float = 0.3, *,
                 min_interval_s: float = 1e-3,
                 max_interval_s: float = 30.0):
        """
        创建估计器；alpha 为新样本权重（现网三处均为 0.3）.

        Raises
        ------
            ValueError: alpha 不在 (0, 1] 或异常过滤区间非法.

        """
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha 必须在 (0, 1] 内: {alpha}')
        if min_interval_s <= 0.0 or min_interval_s >= max_interval_s:
            raise ValueError(
                f'过滤区间非法: ({min_interval_s}, {max_interval_s})')
        self._alpha = float(alpha)
        self._min_interval_s = float(min_interval_s)
        self._max_interval_s = float(max_interval_s)
        self._last_now: Optional[float] = None
        self._ema: Optional[float] = None

    def update(self, now: float) -> None:
        """
        注入一个事件的单调时钟秒；内部完成间隔计算与 EMA 更新.

        异常间隔（≤min_interval_s 或 >max_interval_s）不进 EMA；
        首个合法样本直接作 EMA 初值（与现网 ``ema if None else …`` 一致）。
        now 为注入时钟的当前秒（协议 I3：禁止内部自行取时钟）。
        """
        if self._last_now is not None:
            dt = now - self._last_now
            if self._min_interval_s < dt < self._max_interval_s:
                self._ema = (
                    dt if self._ema is None
                    else (1.0 - self._alpha) * self._ema + self._alpha * dt)
        # 上次时刻始终更新：暂停后首帧虽不进 EMA，但恢复后下一帧间隔有效
        self._last_now = now

    @property
    def interval(self) -> Optional[float]:
        """间隔 EMA（秒）；尚无有效样本时返回 None."""
        return self._ema

    @property
    def rate_hz(self) -> Optional[float]:
        """估计频率（Hz）；尚无有效样本时返回 None（None 安全）."""
        if self._ema is None or self._ema <= 0.0:
            return None
        return 1.0 / self._ema


class AdaptiveTimeout:
    """
    自适应超时取值器（协议 I4：clamp(下限, f(实测EMA), 上限)）.

    构造后不可变，可跨线程只读共享。三处现网用法映射：
      - 视点等待 frame_wait：AdaptiveTimeout(
            lower=2.0, upper=<scan.frame_wait_s 配置>, factor=4.0,
            offset=1.0)；ema 未测得时回退配置值（=upper）；
      - 收齐窗口 max_collect_s：factor=(min_collect+settle+3)、offset=0、
            lower=0.4×配置、upper=float('inf')（现网只设下限；无实测时
            由调用方保留配置值，勿用本类 None 回退档）；
      - 目标观测龄 target_observation_max_age：AdaptiveTimeout(
            lower=1.0, upper=10.0, factor=2.5, offset=0.5)。
    """

    def __init__(self, *, lower: float, upper: float,
                 factor: float, offset: float = 0.0):
        """
        创建取值器；lower ≤ upper，factor ≥ 0，均有限（upper 可为 inf）.

        Raises
        ------
            ValueError: 参数区间非法.

        """
        if lower > upper:
            raise ValueError(f'lower 不得大于 upper: {lower} > {upper}')
        if factor < 0.0:
            raise ValueError(f'factor 不得为负: {factor}')
        self._lower = float(lower)
        self._upper = float(upper)
        self._factor = float(factor)
        self._offset = float(offset)

    def value(self, estimated_interval: Optional[float]) -> float:
        """
        按实测间隔 EMA 求超时秒.

        无实测（None）返回 upper（回退档，对应现网「ema 未测得回退配置
        值」——各用法的配置上限即 upper）；有实测返回
        clamp(lower, factor×estimated_interval + offset, upper)。

        Args:
            estimated_interval: RateEstimator.interval（秒）或 None.

        Returns
        -------
            超时秒数，保证落在 [lower, upper].

        """
        if estimated_interval is None:
            return self._upper
        raw = self._factor * estimated_interval + self._offset
        return min(self._upper, max(self._lower, raw))


# === timing_metrics.py ===

class TimingMetrics:
    """
    分段耗时 EMA 记录器（键 → 毫秒 EMA）.

    构造参数 alpha 为新样本权重（0, 1]；与现网帧率 EMA 同取 0.3。
    record() 逐帧注入各段耗时；snapshot() 返回含全部分段键与 fps 的
    可序列化 dict（键固定排序，便于下游 diff/测试断言）。
    """

    def __init__(self, alpha: float = 0.3):
        """建空记录器；alpha 校验（须在 (0, 1]）."""
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha 必须在 (0, 1] 内: {alpha}')
        self._alpha = float(alpha)
        self._ema: Dict[str, float] = {}

    def record(self, key: str, sample_ms: float) -> None:
        """
        记录一段耗时样本（毫秒）；首个样本直接作 EMA 初值.

        Args:
            key: 分段名（如 'detect_ms'）.
            sample_ms: 本帧该段耗时（毫秒，调用方用注入时钟测量）.

        Returns
        -------
            无返回值（None）；脏样本（nan/inf/负值）静默丢弃.

        """
        value = float(sample_ms)
        if not math.isfinite(value) or value < 0.0:
            return
        old = self._ema.get(key)
        self._ema[key] = (
            value if old is None
            else (1.0 - self._alpha) * old + self._alpha * value)

    def snapshot(self, fps: Optional[float] = None) -> dict:
        """
        返回可 JSON 序列化快照：各段 EMA 毫秒（3 位小数）+ 实测 fps.

        Args:
            fps: 实测帧率（Hz，来自帧间隔 EMA）；None/非正数记 0.0.

        Returns
        -------
            dict：{<分段键>: EMA 毫秒, ..., 'fps': 实测帧率}；尚无样本时
            仅含 'fps' 键.

        """
        out = {key: round(value, 3) for key, value in sorted(self._ema.items())}
        out['fps'] = round(float(fps), 2) if fps and fps > 0.0 else 0.0
        return out


# === grasp_tf.py ===

def _apply_T_to_grasp3d(g3d, T: np.ndarray) -> None:
    """
    抓取几何由相机系变到输出系（默认 base_link），原地修改 g3d.

    T 为 4×4 齐次矩阵（输出系←相机系）。规则：点 R@p+t（含 entry_start /
    bag_bottom / bag_neck / suggested_travel_end / legacy position /
    points_centroid，走 peach_perception.common transform_point）；方向只乘 R 并归一化
    （transform_direction：平移不影响方向）；姿态矩阵左乘 R。None 字段
    原样保留。

    Args:
        g3d: BagGraspReference3D（相机光学系，米）；被原地改写.
        T: (4, 4) 齐次矩阵，输出系←相机系.

    Returns
    -------
        None（结果写回 g3d）.

    """
    # 行程终点、legacy position 与身份锚点（前景点云质心）也是点，必须同步
    # 变换（漏改会让 markers 的行程箭头终点留在相机系，与输出系几何错位；
    # 质心漏改则身份锚点掉到相机系，匹配半径在世界系下失真）
    g3d.entry_start = transform_point(T, g3d.entry_start)
    g3d.bag_bottom = transform_point(T, g3d.bag_bottom)
    g3d.bag_neck = transform_point(T, g3d.bag_neck)
    g3d.suggested_travel_end = transform_point(T, g3d.suggested_travel_end)
    g3d.position = transform_point(T, g3d.position)
    g3d.points_centroid = transform_point(T, g3d.points_centroid)
    g3d.translation_direction = transform_direction(
        T, g3d.translation_direction)
    if g3d.orientation is not None:
        g3d.orientation = (
            T[:3, :3] @ np.asarray(g3d.orientation, dtype=float))


def _rotation_to_quat(R: np.ndarray) -> Quaternion:
    """
    3×3 旋转矩阵 → geometry_msgs/Quaternion（peach_perception.common 值对象的消息包装）.

    数值路径与重构前完全一致：官方 quaternion_from_matrix（见
    peach_perception.common.geometry.rotation_to_quat），此处仅把 QuaternionValue
    组装成消息（纯核不 import geometry_msgs）。

    Args:
        R: (3, 3) 旋转矩阵.

    Returns
    -------
        单位四元数 Quaternion 消息（x, y, z, w）.

    """
    q = rotation_to_quat(R)
    return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)


# === assignment.py ===

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
    # apt python3-scipy，与 common/geometry.py 的惰性导入约定一致
    from scipy.optimize import linear_sum_assignment
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


# === observation_quality.py ===

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


class LightingMeter:
    """
    锁定集目标光照质量统计：逐帧均值 + 跨帧 EMA + 连续低质判定.

    每帧由节点注入两个样本序列（仅锁定集中本帧带掩膜观测的目标）：
    掩膜内有效深度占比与检测置信度；帧内取均值后以 EMA（α 默认 0.3，
    与帧率/耗时埋点同纪律）平滑。判定：深度占比 EMA < min_depth_ratio
    或置信度 EMA < min_conf_mean 的帧记一帧低质，连续 bad_frames 帧
    低质 → low_quality=True；一帧达标即清零连击（与摆动判定的对称
    连击同一风格）。无样本帧（锁定集目标全部无掩膜观测）不进 EMA也
    不计低质——没有观测不等于低质。

    线程安全：无内部锁，与调用方（节点 _plan_lock 保护区）同一把锁。
    """

    def __init__(self, alpha: float = 0.3, min_depth_ratio: float = 0.35,
                 min_conf_mean: float = 0.3, bad_frames: int = 5):
        """建表；α∈(0,1]、阈值∈[0,1]、连击帧数≥1 校验."""
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha 必须在 (0, 1] 内: {alpha}')
        if not 0.0 <= min_depth_ratio <= 1.0:
            raise ValueError(f'min_depth_ratio 须在 [0,1]: {min_depth_ratio}')
        if not 0.0 <= min_conf_mean <= 1.0:
            raise ValueError(f'min_conf_mean 须在 [0,1]: {min_conf_mean}')
        if bad_frames < 1:
            raise ValueError(f'bad_frames 须 ≥ 1: {bad_frames}')
        self.alpha = float(alpha)
        self.min_depth_ratio = float(min_depth_ratio)
        self.min_conf_mean = float(min_conf_mean)
        self.bad_frames = int(bad_frames)
        self._depth_ema: Optional[float] = None
        self._conf_ema: Optional[float] = None
        self._bad_streak = 0

    @staticmethod
    def _finite_mean(samples: Iterable[float]) -> Optional[float]:
        """有限样本均值；空集/全非有限返回 None（脏样本不进 EMA）."""
        values = [float(s) for s in samples if math.isfinite(float(s))]
        if not values:
            return None
        return sum(values) / len(values)

    def update(self, depth_ratios: Iterable[float],
               confidences: Iterable[float]) -> None:
        """
        注入本帧锁定集目标的观测样本并刷新 EMA 与低质连击.

        Args:
            depth_ratios: 各目标掩膜内有效深度占比 [0,1]（可空序列）.
            confidences: 各目标检测置信度 [0,1]（可空序列）.

        Returns
        -------
            无返回值（None）；两序列均空（本帧无有效观测）时整帧跳过，
            EMA 与连击保持不变.

        """
        depth_mean = self._finite_mean(depth_ratios)
        conf_mean = self._finite_mean(confidences)
        if depth_mean is None and conf_mean is None:
            return
        if depth_mean is not None:
            self._depth_ema = (
                depth_mean if self._depth_ema is None
                else (1.0 - self.alpha) * self._depth_ema
                + self.alpha * depth_mean)
        if conf_mean is not None:
            self._conf_ema = (
                conf_mean if self._conf_ema is None
                else (1.0 - self.alpha) * self._conf_ema
                + self.alpha * conf_mean)
        # 尚无 EMA 的分量按达标处理（无法证明低质时不冤枉现场光照）
        bad = (
            (self._depth_ema is not None
             and self._depth_ema < self.min_depth_ratio)
            or (self._conf_ema is not None
                and self._conf_ema < self.min_conf_mean))
        self._bad_streak = self._bad_streak + 1 if bad else 0

    @property
    def low_quality(self) -> bool:
        """连续 bad_frames 帧低质（EMA 维度任一不达标）."""
        return self._bad_streak >= self.bad_frames

    def snapshot(self) -> dict:
        """
        harvest_state JSON 的 lighting 子对象.

        Returns
        -------
            dict：depth_ratio / conf_mean 为 EMA（无样本为 None）、
            bad_streak 为当前低质连击帧数、low_quality 为判定结果.

        """
        return {
            'depth_ratio': self._depth_ema,
            'conf_mean': self._conf_ema,
            'bad_streak': self._bad_streak,
            'low_quality': self.low_quality,
        }


# === segmentation_gate.py ===

# 检测框外扩比例（每边各扩 10% 宽高）：容忍锚点投影与 YOLO 框边的贴边
# 误差（质心投影理论上在框内，外扩只为深度噪声/框回归抖动兜底）
DEFAULT_MARGIN_FRAC = 0.1


def project_positions_to_pixels(
        positions: Dict[str, np.ndarray],
        T_cam_world: np.ndarray,
        camera_K: dict) -> Dict[str, Tuple[float, float]]:
    """
    世界系锚点集 → 本帧像素坐标（pinhole 投影）.

    Args:
        positions: target_id → (3,) 世界系（output_frame）锚点（米）.
        T_cam_world: (4, 4) 世界系→相机光学系齐次变换（即 output←camera
            的逆；调用方负责取逆）.
        camera_K: 内参 dict（fx/fy/cx/cy；width/height 存在时用于裁剪
            视野外投影）.

    Returns
    -------
        target_id → (u, v) 像素坐标；非有限、相机后方（z≤0）或明确
        落在图像外的锚点被剔除（剔除即视为本帧不可见，不参与门控）.

    """
    T = np.asarray(T_cam_world, dtype=float).reshape(4, 4)
    fx = float(camera_K['fx'])
    fy = float(camera_K['fy'])
    cx = float(camera_K['cx'])
    cy = float(camera_K['cy'])
    width = camera_K.get('width')
    height = camera_K.get('height')
    out: Dict[str, Tuple[float, float]] = {}
    for target_id, pos in positions.items():
        p = np.asarray(pos, dtype=float).reshape(3)
        if not np.all(np.isfinite(p)):
            continue
        pc = T[:3, :3] @ p + T[:3, 3]
        if not np.all(np.isfinite(pc)) or pc[2] <= 1e-8:
            continue
        u = fx * pc[0] / pc[2] + cx
        v = fy * pc[1] / pc[2] + cy
        if width is not None and height is not None:
            if not (0.0 <= u < float(width) and 0.0 <= v < float(height)):
                continue
        out[target_id] = (float(u), float(v))
    return out


def plan_segmentation_bboxes(
        detections: List[dict],
        locked_only: bool,
        locked: bool,
        anchor_px: Optional[Dict[str, Tuple[float, float]]],
        margin_frac: float = DEFAULT_MARGIN_FRAC) -> List[Tuple[int, int, int, int]]:
    """
    决定本帧送 SAM 的检测框集（2.13-E1 门控策略，纯函数）.

    语义矩阵（详见模块 docstring 降级语义）：
      - ``locked_only=False`` 或 ``locked=False`` → 全量框（旧行为）；
      - 已锁定但 ``anchor_px=None``（锚点不可投影，如 TF 不可用帧）
        → 全量框（无法识别哪些框属于锁定集，宁多勿漏）；
      - 已锁定且 ``anchor_px`` 为空 dict → 空列表（锁定目标本帧均不
        可见，SAM 零推理）；
      - 否则只选「外扩 margin 后包含至少一个锁定锚点像素」的框。

    Args:
        detections: 本帧入管线检测 dict 列表（须带 'bbox' 键，
            (x1, y1, x2, y2) 像素框）.
        locked_only: 锁定后 selected-only 开关（yaml
            pipeline.locked_only_segmentation）.
        locked: 目标集合是否已锁定（harvest_plan.locked）.
        anchor_px: 锁定目标锚点像素集（project_positions_to_pixels
            输出）；None 表示本帧锚点不可投影.
        margin_frac: 框外扩比例（每边各扩 margin_frac×宽/高）.

    Returns
    -------
        送 SAM 的 (x1, y1, x2, y2) 框列表，顺序与 detections 一致.

    """
    all_bboxes = [tuple(d['bbox']) for d in detections]
    if not locked_only or not locked or anchor_px is None:
        return all_bboxes
    if not anchor_px:
        return []
    points = list(anchor_px.values())
    selected = []
    for bbox in all_bboxes:
        x1, y1, x2, y2 = (float(v) for v in bbox)
        mx = (x2 - x1) * margin_frac
        my = (y2 - y1) * margin_frac
        if any(x1 - mx <= u <= x2 + mx and y1 - my <= v <= y2 + my
               for u, v in points):
            selected.append(bbox)
    return selected


# === pipeline.py ===

def grasp_frame_from_axis(axis) -> np.ndarray:
    """右手抓取系 R=[Xg,Yg,Zg]，无点云时由袋轴与参考轴叉积得到."""
    zg = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(zg))
    zg = zg / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])
    ref = (np.array([1.0, 0.0, 0.0]) if abs(zg[0]) < 0.9
           else np.array([0.0, 0.0, 1.0]))
    xg = np.cross(zg, ref)
    xg /= np.linalg.norm(xg)
    if xg[0] < 0:
        xg = -xg
    yg = np.cross(zg, xg)
    return np.column_stack((xg, yg, zg))


@dataclass
class TargetPoseResult:
    """单个目标的估计结果；只索引本检测，绝不混入其他目标."""

    target_id: str                    # 目标 ID（如 'target_0' / 'frame:idx'）
    grasp_2d: BagGrasp2D              # 图像平面参考（像素坐标）
    grasp_3d: BagGraspReference3D     # 3D 抓取参考（相机光学系，米）
    mask_source: str                  # 前景掩膜来源标签（诊断追溯用）
    metrics: dict                     # 诊断指标；缺项约定 None（消息层填 -1）
    target_kind: str = 'bag'  # "bag" | "fruit"


def clip_bbox(bbox, shape):
    """
    检测框裁剪到图像范围内.

    Args:
        bbox: (x1, y1, x2, y2) 像素框（可越界，先取整）.
        shape: 图像 shape（取前两维 h, w）.

    Returns
    -------
        (x1, y1, x2, y2) 裁剪后的整数框（可能退化）.

    """
    h, w = shape[:2]
    x1, y1, x2, y2 = (int(round(v)) for v in bbox)
    return max(0, x1), max(0, y1), min(w, x2), min(h, y2)


def valid_depth_mask(depth, min_depth_m: float, max_depth_m: float):
    """
    有效深度掩膜：非 0、非饱和，且在 [min_depth_m, max_depth_m] 内.

    Args:
        depth: (h, w) uint16 深度（毫米）.
        min_depth_m: 有效深度下限 (m)，过近视为噪声.
        max_depth_m: 有效深度上限 (m)，过远视为背景.

    Returns
    -------
        (h, w) bool 掩膜.

    """
    z = depth.astype(np.float32) / 1000.0
    return (depth > 0) & (depth < 65535) & (z > min_depth_m) & (z < max_depth_m)


def foreground_mask(depth, valid, supplied_mask, bbox, source):
    """
    前景掩膜：优先外部掩膜 ∩ 有效深度；否则深度带连通域显式降级.

    降级路线：ROI 中心 1/3 区域的深度中位数 ± max(25mm, 3·MAD) 带
    → 8 连通域，取中心所在域（中心无域则取最大域）。

    Args:
        depth: (h, w) uint16 ROI 深度（毫米）.
        valid: (h, w) bool 有效深度掩膜.
        supplied_mask: 外部掩膜（全图或本 ROI）；形状相符且与有效深度
            交集 ≥50 像素才采用，否则忽略走降级.
        bbox: 全图坐标下的 (x1, y1, x2, y2)，用于裁全图掩膜.
        source: 外部掩膜来源标签.

    Returns
    -------
        ((h, w) bool 前景掩膜, 来源标签)；降级时标签为 'depth_fallback'.

    """
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


class RobustBagPosePipeline(PosePipeline):
    kind = 'bag'
    """
    袋装桃的保守位姿估计器（圆柱套入工具）.

    所有安全判定只用实测深度。优先使用外部实例掩膜（SAM），
    深度带连通域是显式、可检查的降级来源。
    """

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
                 mask_source: str = 'depth_fallback') -> TargetPoseResult:
        """
        估计 bbox 内单个袋装目标，返回显式安全状态的结果.

        Args:
            obs: 单帧输入（深度 uint16 毫米；gravity_hint 为相机系方向或 None）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素，自动裁剪到图内）.
            mask: 外部前景掩膜（全图或 ROI，bool/0-1）；None 走深度带降级.
            mask_source: 掩膜来源标签，写入诊断.

        Returns
        -------
            TargetPoseResult；status ∈ ACCEPT/REOBSERVE/REJECT，硬性失败
            （点太少/净空不足等）直接 REJECT，诊断指标经 metrics 暴露.

        """
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
        _bpx, _npx = self._project(bottom, obs.camera_K), self._project(neck, obs.camera_K)
        if _bpx is not None and _npx is not None:
            disagreement_deg = self._mask_axis_disagreement(local_mask, _bpx, _npx)

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
        bpx = self._project(bottom, obs.camera_K)
        npx = self._project(neck, obs.camera_K)
        if self._mask_axis_against_taper(local_mask, x1, y1, bpx, npx):
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
        length = float(np.dot(neck - bottom, axis))

        # ── entry_start = P_bottom − (d_tool + d_s)·axis (Gürsoy 分解) ──
        standoff = self.tool.entry_d_tool + self.tool.entry_d_s
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
        radial_clearance = self.tool.D_inner / 2.0 - diameter / 2.0 - self.tool.clearance_min
        budget_m = (standoff + travel) * np.sin(np.radians(theta_err_deg))

        flags = []
        flags.extend(landmark_flags)
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
        g3d = BagGraspReference3D(
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
        return TargetPoseResult(target_id, base_2d, g3d, source, metrics)

    def _failed(self, target_id, g2d, reason, source, **metrics):
        """
        构造袋线 REJECT 结果（status=REJECT + 单一诊断标记）.

        Args:
            target_id: 目标 ID.
            g2d: 已建的 BagGrasp2D（被改写为 REJECT）.
            reason: 失败原因标记（写入 diagnostic_flags）.
            source: 掩膜来源标签.
            **metrics: 已采集的诊断指标，原样透传.

        Returns
        -------
            TargetPoseResult（target_kind='bag'）.

        """
        g2d.status = 'REJECT'
        g2d.diagnostic_flags = [reason]
        g3d = BagGraspReference3D(status='REJECT', diagnostic_flags=[reason],
                                  strategy_id='robust_bag_pose', tool_version=self.tool.version,
                                  diagnostic_info={**metrics, 'mask_source': source})
        return TargetPoseResult(target_id, g2d, g3d, source, metrics)

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

    def _to_points(self, depth, mask, xoff, yoff, K):
        """
        前景像素反投影为相机系 3D 点（米）.

        Args:
            depth: (h, w) uint16 ROI 深度（毫米）.
            mask: (h, w) bool 前景掩膜（内部再与有效深度求交）.
            xoff: ROI 在全图的 x 像素偏移.
            yoff: ROI 在全图的 y 像素偏移.
            K: 相机内参 {"fx","fy","cx","cy"}.

        Returns
        -------
            (points, pixels)：points 为 (N, 3) float 相机系坐标（米），
            pixels 为 (N, 2) int ROI 内像素坐标 (x, y).

        """
        ys, xs = np.where(mask & self._valid_depth(depth))
        z = depth[ys, xs].astype(float) / 1000.0
        points = np.column_stack(((xs + xoff - K['cx']) * z / K['fx'],
                                 (ys + yoff - K['cy']) * z / K['fy'], z))
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
        相机系 3D 点 → 像素 (u, v).

        Args:
            point: (3,) 点（米）；None 或 z≤1e-8 给 None.
            K: 相机内参 {"fx","fy","cx","cy"}.

        Returns
        -------
            (u, v) float 像素；不可投影返回 None.

        """
        if point is None or point[2] <= 1e-8:
            return None
        return (float(point[0] * K['fx'] / point[2] + K['cx']),
                float(point[1] * K['fy'] / point[2] + K['cy']))


# ═══════════════════════════════════════════════════════════════
# 裸果管线 (class 1, peach_nobag) — 同一圆柱剪切工具的第二条流程线
# ═══════════════════════════════════════════════════════════════

class RobustFruitPosePipeline(RobustBagPosePipeline):
    kind = 'fruit'

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

    CAVITY_HALF_ANGLE_COS = np.cos(np.radians(20.0))
    CAVITY_MIN_DIP_M = 0.0015     # 梗洼最小下陷深度 1.5mm
    CAVITY_MIN_POINTS = 12

    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback') -> TargetPoseResult:
        """
        估计 bbox 内单个裸果目标，返回显式安全状态的结果.

        Args:
            obs: 单帧输入（深度 uint16 毫米；gravity_hint 为相机系方向或 None）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素，自动裁剪到图内）.
            mask: 外部前景掩膜（全图或 ROI）；None 走深度带降级.
            mask_source: 掩膜来源标签，写入诊断.

        Returns
        -------
            TargetPoseResult（target_kind='fruit'）；metrics 追加
            fruit_radius_m / sphere_rms_m / sphere_inlier_ratio /
            cavity_dip_mm / axis_polarity_corrected（球拟合失败时前三项为 None）.

        """
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
        flags.append('unbagged_display_only')
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
        pos_cov, dir_cov = estimate_pose_covariance(
            points, axis, float(theta_err_deg))
        g3d = BagGraspReference3D(
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
        """
        构造果线 REJECT 结果（同 _failed，target_kind='fruit'）.

        Args:
            target_id: 目标 ID.
            g2d: 已建的 BagGrasp2D（被改写为 REJECT）.
            reason: 失败原因标记（写入 diagnostic_flags）.
            source: 掩膜来源标签.
            **metrics: 已采集的诊断指标，原样透传.

        Returns
        -------
            TargetPoseResult（target_kind='fruit'）.

        """
        g2d.status = 'REJECT'
        g2d.diagnostic_flags = [reason]
        g3d = BagGraspReference3D(status='REJECT', diagnostic_flags=[reason],
                                  strategy_id='robust_fruit_pose',
                                  tool_version=self.tool.version,
                                  diagnostic_info={**metrics, 'mask_source': source})
        return TargetPoseResult(target_id, g2d, g3d, source, metrics,
                                target_kind='fruit')


# 默认实现登记在本文件末尾（2.14：POSE_PIPELINES.register
# ('robust_bag' / 'robust_fruit', ...)）


# === inference.py ===

_logger = logging.getLogger(__name__)


def _resolve_device() -> str:
    """
    选推理设备：有 CUDA 用 'cuda:0'，否则 'cpu'.

    Returns
    -------
        设备字符串（torch 未安装时视为无卡，回退 'cpu'）.

    """
    try:
        import torch
        if torch.cuda.is_available():
            return 'cuda:0'
    except ImportError:
        pass
    return 'cpu'


class UltralyticsYolo(Detector):
    """
    Ultralytics YOLO 检测器（Detector 默认实现，注册名 'yolo'）.

    懒加载：首次 detect 才读权重。所有推理经 self._lock 序列化，确保同一
    时刻仅一个线程占用 GPU 模型。
    """

    def __init__(self, yolo_model: str = '', yolo_conf: float = 0.3,
                 yolo_iou: float = 0.5, class_names: dict = None):
        """
        构造检测器（模型懒加载，首次推理时才读权重）.

        Args:
            yolo_model: YOLO 权重路径（.pt）；空串行为取决于 ultralytics.
            yolo_conf: YOLO 置信度阈值 [0, 1].
            yolo_iou: YOLO NMS IoU 阈值 [0, 1].
            class_names: {class_id: 名称}；None 用默认 {0: peach_bag,
                1: peach_nobag}.

        Returns
        -------
            无返回值（None）.

        """
        self._yolo_model_path = yolo_model
        self._yolo_conf = yolo_conf
        self._yolo_iou = yolo_iou
        self._class_names = class_names or {0: 'peach_bag', 1: 'peach_nobag'}
        # 懒加载: None 表示尚未 load 权重
        self._yolo = None
        # 推理设备：默认优先 CUDA（peach_scene_perception 要求 GPU）；无卡时回退 CPU
        self._device = _resolve_device()
        # CUDA 线程安全: 锁序列化 load + forward
        self._lock = threading.Lock()

    def detect(self, rgb: np.ndarray) -> List[dict]:
        """
        对 RGB 图像运行 YOLO 目标检测 (管线步骤 ①).

        Args:
            rgb: (H, W, 3) BGR 图像 (OpenCV 惯例)

        Returns
        -------
        [{"class_id", "class_name", "bbox": (x1,y1,x2,y2), "conf"}, ...]
        按置信度降序排列

        """
        with self._lock:
            if self._yolo is None:
                from ultralytics import YOLO
                self._yolo = YOLO(self._yolo_model_path)
                # 权重迁到目标设备；后续 predict 显式传 device，避免默认漂到 CPU
                try:
                    self._yolo.to(self._device)
                except Exception:
                    pass

            results = self._yolo(
                rgb, conf=self._yolo_conf, iou=self._yolo_iou,
                device=self._device, verbose=False)

        # 锁外解析: 纯 CPU 后处理，不涉及 CUDA
        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for i in range(len(r.boxes)):
                ci = int(r.boxes.cls[i])
                cf = float(r.boxes.conf[i])
                x1, y1, x2, y2 = clip_bbox(
                    r.boxes.xyxy[i].tolist(), rgb.shape)
                if x2 <= x1 or y2 <= y1:
                    continue
                dets.append({
                    'class_id': ci,
                    'class_name': self._class_names.get(ci, f'cls_{ci}'),
                    'bbox': (x1, y1, x2, y2),
                    'conf': cf,
                })

        dets.sort(key=lambda d: d['conf'], reverse=True)
        return dets

    def reset(self):
        """释放 YOLO 缓存 (切换模型路径或数据集后调用)。线程安全."""
        with self._lock:
            self._yolo = None


class MobileSam(Segmenter):
    """
    Ultralytics MobileSAM 分割器（Segmenter 默认实现，注册名 'mobile_sam'）.

    懒加载：首次 segment 才读权重。SAM 以 bbox 为 box prompt，在框内生成
    二值前景掩码；面积 < sam_min_area 的掩码被丢弃。所有推理经
    self._lock 序列化（CUDA 线程安全，同 UltralyticsYolo）。
    """

    def __init__(self, sam_model: str = 'mobile_sam.pt',
                 sam_max_bboxes: int = 16, sam_min_area: int = 100):
        """
        构造分割器（模型懒加载，首次推理时才读权重）.

        Args:
            sam_model: SAM 权重路径或模型名.
            sam_max_bboxes: 单次 SAM 推理的最大 prompt 框数（超出截断）；
                默认 16（阶段 D1 由 8 上调并参数化为 yaml sam_max_bboxes：
                室外多果场景一帧目标数常超 8，截断目标无掩膜被判 OCCLUDED）.
            sam_min_area: 掩膜最小像素数，过小丢弃.

        Returns
        -------
            无返回值（None）.

        """
        self._sam_model_name = sam_model
        self._sam_max_bboxes = sam_max_bboxes
        self._sam_min_area = sam_min_area
        # 懒加载: None 表示尚未 load 权重
        self._sam = None
        self._device = _resolve_device()
        self._lock = threading.Lock()

    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        对 RGB 图像运行 SAM 实例分割 (管线步骤 ②).

        Args:
            rgb: (H, W, 3) BGR 图像
            bboxes: [(x1, y1, x2, y2), ...]，超过 sam_max_bboxes 时截断

        Returns
        -------
        [(binary_mask, bbox), ...]，面积 < sam_min_area 的掩码被丢弃

        """
        if not bboxes:
            return []

        with self._lock:
            if self._sam is None:
                from ultralytics import SAM
                self._sam = SAM(self._sam_model_name)
                try:
                    self._sam.to(self._device)
                except Exception:
                    pass

            # 限制 bbox 数量: SAM 批量推理显存与耗时随 N 增长
            if len(bboxes) > self._sam_max_bboxes:
                bboxes = bboxes[:self._sam_max_bboxes]

            try:
                results = self._sam(
                    rgb, bboxes=bboxes, device=self._device, verbose=False)
            except Exception as e:
                # 纯核不能 import ROS，走 stdlib logging（print 会污染 stdout）
                _logger.warning('SAM 分割失败: %s', e)
                return []

        if not results or results[0].masks is None:
            return []

        masks = results[0].masks.data.cpu().numpy()  # GPU→CPU: (N, H, W) 概率图
        ih, iw = rgb.shape[:2]

        output = []
        for i, mask in enumerate(masks):
            bin_mask = mask > 0.5  # 阈值化得布尔前景掩码
            if bin_mask.shape != (ih, iw):
                bin_mask = cv2.resize(
                    bin_mask.astype(np.uint8), (iw, ih),
                    interpolation=cv2.INTER_NEAREST).astype(bool)
            if bin_mask.sum() > self._sam_min_area:
                output.append((bin_mask, bboxes[i]))

        return output

    def reset(self):
        """释放 SAM 缓存 (切换模型路径或数据集后调用)。线程安全."""
        with self._lock:
            self._sam = None


class InferenceEngine:
    """
    检测/分割组合引擎（调用端）：只持有 Detector/Segmenter 接口引用.

    detect/segment/reset 全部委托给构造期注入的接口实现；引擎自身不含
    任何模型逻辑，可替换性由接口层注册表（2.14）保证。

    用法::

        engine = InferenceEngine(
            detector=UltralyticsYolo(yolo_model='best.pt'),
            segmenter=MobileSam(sam_model='mobile_sam.pt'),
        )
        dets = engine.detect(rgb)               # → list[dict]
        masks = engine.segment(rgb, bboxes)     # → list[(mask, bbox)]
    """

    def __init__(self, detector: Detector, segmenter: Segmenter):
        """
        装配检测器与分割器（接口引用，不绑死具体实现）.

        Args:
            detector: Detector 接口实现（如 UltralyticsYolo）.
            segmenter: Segmenter 接口实现（如 MobileSam）.

        Returns
        -------
            无返回值（None）.

        """
        self._detector = detector
        self._segmenter = segmenter

    def detect(self, rgb: np.ndarray) -> List[dict]:
        """委托注入的 Detector（签名与语义见 Detector.detect）."""
        return self._detector.detect(rgb)

    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """委托注入的 Segmenter（签名与语义见 Segmenter.segment）."""
        return self._segmenter.segment(rgb, bboxes)

    def reset(self):
        """释放两个模型的缓存（逐接口委托；实现方各自保证线程安全）."""
        self._detector.reset()
        self._segmenter.reset()


# === candidates.py ===

@dataclass(frozen=True)
class ForegroundMode:
    """前景模式描述（目前仅 hybrid_dilated）."""

    mode_id: str      # 模式 ID（如 'hybrid_dilated'），估计路由键
    label: str        # 中文短标签（报告/界面显示）
    description: str  # 一句话说明


FOREGROUND_MODES = (
    ForegroundMode(
        'hybrid_dilated', 'SAM∩膨胀深度',
        'SAM 掩膜与膨胀后的实测深度连通域求交'),
)
MODE_IDS = tuple(mode.mode_id for mode in FOREGROUND_MODES)
MODE_LABELS = {mode.mode_id: mode.label for mode in FOREGROUND_MODES}


class CandidateEstimator:
    """
    构造收敛前景掩膜，并送入安全管线评估.

    按检测类别分流：
      - ``peach_bag`` (class_id=0) → 圆柱轴袋线 ``RobustBagPosePipeline``
      - ``peach_nobag`` (class_id=1) → 球+梗腔果线 ``RobustFruitPosePipeline``
    两线共用同一圆柱刀具、入口/行程公式与安全门控。
    """

    def __init__(self, pipeline: Optional[RobustBagPosePipeline] = None,
                 fruit_pipeline: Optional[RobustBagPosePipeline] = None,
                 dilate_px: int = 5, min_mask_points: int = 50):
        """
        构造估计器；两条管线只按 PosePipeline 接口持有（2.14 装配）.

        Args:
            pipeline: 袋线实例；None 时按注册表默认实现（'robust_bag'）
                新建.
            fruit_pipeline: 果线实例；None 时按注册表默认实现
                （'robust_fruit'）新建并复用袋线的 ToolGeometry，保证刀具
                契约一致.
            dilate_px: 深度连通域膨胀半径（像素，≥1；核边长 2*(p//2)+1）.
            min_mask_points: 掩膜最小像素数，不足判 mask_unavailable.

        Returns
        -------
            无返回值（None）.

        """
        self.pipeline = pipeline or POSE_PIPELINES.create('robust_bag')
        self.fruit_pipeline = fruit_pipeline or POSE_PIPELINES.create(
            'robust_fruit', tool=self.pipeline.tool)
        # 类别路由用的实例表：kind（注册表键）→ 已建实例（YOLO 标签契约：
        # class_id==1 → 'fruit'，其余 → 'bag'，见 _pipeline_for）
        self._estimator_by_kind = {
            'bag': self.pipeline,
            'fruit': self.fruit_pipeline,
        }
        self.dilate_px = max(1, int(dilate_px))
        self.min_mask_points = max(1, int(min_mask_points))
        self.last_timings_ms: dict[str, float] = {}
        self._last_mask_timings_ms: dict[str, float] = {}

    def _pipeline_for(self, obs: BagObservation, bbox: tuple | None = None
                      ) -> tuple:
        """
        按与 bbox 匹配的检测 class_id 选择袋线 / 果线.

        Args:
            obs: 单帧输入.
            bbox: 当前目标框；None 时才回退 detections[0].

        Returns
        -------
            (kind, pipeline)：class_id==1 → fruit，否则 bag.

        """
        class_id = 0
        dets = list(obs.detections or [])
        if bbox is not None and dets:
            bx = np.asarray(bbox, dtype=float).reshape(4)
            best_iou, best = -1.0, None
            for det in dets:
                db = np.asarray(det.get('bbox', (0, 0, 0, 0)), dtype=float)
                if db.size != 4:
                    continue
                ix1 = max(bx[0], db[0])
                iy1 = max(bx[1], db[1])
                ix2 = min(bx[2], db[2])
                iy2 = min(bx[3], db[3])
                inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
                union = ((bx[2] - bx[0]) * (bx[3] - bx[1])
                         + (db[2] - db[0]) * (db[3] - db[1]) - inter)
                iou = inter / union if union > 1e-6 else 0.0
                if iou > best_iou:
                    best_iou, best = iou, det
            if best is not None:
                class_id = int(best.get('class_id', 0))
        elif dets:
            class_id = int(dets[0].get('class_id', 0))
        kind = 'fruit' if class_id == 1 else 'bag'
        return kind, self._estimator_by_kind[kind]

    def estimate_modes(self, obs: BagObservation, target_id: str, bbox: tuple,
                       sam_mask: Optional[np.ndarray],
                       modes: Optional[Iterable[str]] = None
                       ) -> dict[str, TargetPoseResult]:
        """
        对请求的前景模式跑同一套几何与安全门控，返回 mode→结果.

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            sam_mask: 全图 SAM 掩膜或 None（None → 各模式 mask_unavailable）.
            modes: 要跑的模式 ID 可迭代；None 跑全部已注册模式；
                含未知 ID 抛 ValueError.

        Returns
        -------
            {mode_id: TargetPoseResult}；掩膜不可用时结果为显式 REOBSERVE；
            副作用：刷新 last_timings_ms（毫秒，含掩膜构造耗时）.

        """
        selected = tuple(modes or MODE_IDS)
        unknown = set(selected) - set(MODE_IDS)
        if unknown:
            raise ValueError(f'unknown foreground modes: {sorted(unknown)}')

        masks = self.build_masks(obs, bbox, sam_mask)
        kind, pipeline = self._pipeline_for(obs, bbox)
        results = {}
        self.last_timings_ms = {}
        for mode in selected:
            started = time.perf_counter()
            mask = masks.get(mode)
            if mask is None:
                # SAM 缺失或交后像素不足：显式 REOBSERVE，不走深度-only 回退
                results[mode] = self._unavailable(
                    obs, target_id, bbox, mode, 'mask_unavailable')
            else:
                results[mode] = pipeline.estimate(
                    obs, target_id, bbox, mask, self._source(mode))
            pose = results[mode].grasp_3d
            results[mode].target_kind = kind
            pose.strategy_id = f'robust_{kind}_pose:{mode}'
            pose.model_version = str(obs.metadata.get('model_version', 'unknown'))
            pose.calibration_version = str(obs.metadata.get(
                'calibration_version', 'unknown'))
            pose.tool_version = self.pipeline.tool.version
            geometry_ms = (time.perf_counter() - started) * 1000.0
            # 总耗时 = 掩膜构造 + 本模式几何
            self.last_timings_ms[mode] = (
                self._last_mask_timings_ms.get(mode, 0.0) + geometry_ms)
        return results

    def build_masks(self, obs: BagObservation, bbox: tuple,
                    sam_mask: Optional[np.ndarray]) -> dict[str, Optional[np.ndarray]]:
        """
        在 bbox ROI 内构造 hybrid_dilated 掩膜（实测深度单位：毫米 uint16）.

        hybrid_dilated = (SAM ∩ 有效深度) ∩ 膨胀后的深度连通域；
        交后像素 < min_mask_points 给 None。

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            bbox: (x1, y1, x2, y2) 检测框（像素，自动裁剪到图内）.
            sam_mask: 全图或 ROI 掩膜；None 或裁剪失败则结果为 None.

        Returns
        -------
            {mode_id: (h, w) bool ROI 掩膜或 None}；副作用：刷新
            _last_mask_timings_ms（毫秒）.

        """
        started = time.perf_counter()
        self._last_mask_timings_ms = {mode: 0.0 for mode in MODE_IDS}
        x1, y1, x2, y2 = clip_bbox(bbox, obs.depth.shape)
        roi = obs.depth[y1:y2, x1:x2]
        if roi.size == 0:
            return {mode: None for mode in MODE_IDS}
        # 有效深度区间取袋线管线参数（两条线共用同一相机/深度约定）
        valid = valid_depth_mask(
            roi, self.pipeline.min_depth_m, self.pipeline.max_depth_m)
        # 深度连通前景：作为「膨胀母体」，限制 SAM 不漂到背景
        depth_mask, _ = foreground_mask(
            roi, valid, None, bbox, source='depth_fallback')

        mask = None
        sam_roi = self._crop_mask(sam_mask, (x1, y1, x2, y2), obs.depth.shape)
        if sam_roi is not None:
            # 只保留有实测深度的 SAM 像素
            measured_sam = sam_roi & valid
            k = 2 * (self.dilate_px // 2) + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            expanded_depth = cv2.dilate(depth_mask.astype(np.uint8), kernel) > 0
            # SAM ∩ 膨胀深度；像素过少则视为不可用
            mask = self._enough(measured_sam & expanded_depth)

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self._last_mask_timings_ms = {mode: elapsed_ms for mode in MODE_IDS}
        return {'hybrid_dilated': mask}

    def _crop_mask(self, mask: Optional[np.ndarray], bbox: tuple,
                   image_shape: tuple) -> Optional[np.ndarray]:
        """
        把全图或 ROI 掩膜裁成与 bbox 同尺寸；尺寸不符返回 None.

        Args:
            mask: bool/0-1 掩膜（全图尺寸则裁 ROI）；None 原样返回 None.
            bbox: (x1, y1, x2, y2) 已裁剪到图内的整数框（像素）.
            image_shape: 全图 shape（判全图/ROI 用）.

        Returns
        -------
            (y2-y1, x2-x1) bool 掩膜；尺寸对不上给 None.

        """
        if mask is None:
            return None
        x1, y1, x2, y2 = bbox
        arr = np.asarray(mask, dtype=bool)
        if arr.shape[:2] == image_shape[:2]:
            arr = arr[y1:y2, x1:x2]
        expected = (y2 - y1, x2 - x1)
        return arr if arr.shape == expected else None

    def _enough(self, mask: np.ndarray) -> Optional[np.ndarray]:
        """
        像素数不足 min_mask_points 时丢弃（触发 mask_unavailable）.

        Args:
            mask: (h, w) bool 掩膜.

        Returns
        -------
            原掩膜或 None.

        """
        return mask if int(mask.sum()) >= self.min_mask_points else None

    @staticmethod
    def _source(mode: str) -> str:
        """
        写入结果的 mask_source 标签（便于诊断追溯）.

        Args:
            mode: 已注册模式 ID（未知 ID 抛 KeyError）.

        Returns
        -------
            来源标签字符串.

        """
        return {
            'hybrid_dilated': 'mobile_sam_dilated_depth_intersection',
        }[mode]

    def _unavailable(self, obs: BagObservation, target_id: str, bbox: tuple,
                     mode: str, reason: str) -> TargetPoseResult:
        """
        构造显式失败结果（REOBSERVE + diagnostic_flags）.

        Args:
            obs: 单帧输入（取版本元数据）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            mode: 前景模式 ID（写入 strategy_id）.
            reason: 原因标记（如 'mask_unavailable'）.

        Returns
        -------
            TargetPoseResult（status=REOBSERVE，metrics 为空）.

        """
        x1, y1, x2, y2 = map(int, bbox)
        g2d = BagGrasp2D(
            detection_bbox=(x1, y1, x2 - x1, y2 - y1),
            status='REOBSERVE', diagnostic_flags=[reason])
        g3d = BagGraspReference3D(
            status='REOBSERVE', diagnostic_flags=[reason],
            # strategy_id 与成功路径同名（袋线/果线经管线 kind 区分，不恒为 bag）
            strategy_id=f'robust_{self.pipeline.kind}_pose:{mode}',
            model_version=str(obs.metadata.get('model_version', 'unknown')),
            calibration_version=str(obs.metadata.get(
                'calibration_version', 'unknown')),
            tool_version=self.pipeline.tool.version)
        return TargetPoseResult(target_id, g2d, g3d, mode, {})


def dedup_overlapping_detections(
        dets, ios_threshold: float = 0.6,
        frag_ios_threshold: float = 0.2,
        frag_area_ratio: float = 0.5) -> list:
    """
    重叠检测框去重：IoS（交集/较小框面积）≥ 阈值判同一物理目标，保留大框.

    规则 1（基本包含）：IoS ≥ ios_threshold → 抑制小框（原有）；
    规则 2（碎片残枝，09-01「先做大框」定夺）：IoS ≥ frag_ios_threshold
    且面积比（小/大）≤ frag_area_ratio → 抑制小框——叶片遮挡碎片框 IoS
    达不到包含阈值，但「明显更小+可见重叠」足以判为同一颗的残片；相邻
    两颗袋面积相当（比值≈1）不会被误删。面积并列时保留置信度高者；
    跨类别同样生效——YOLO 按类 NMS，同一颗桃可同时出 bag/nobag 两框，
    都会在身份注册表上重复占号。用 IoS 而非 IoU：部分重叠的相邻两颗桃
    IoS 低不误删。
    贪心顺序为面积降序（置信度次之），后遍历到的高重叠框被抑制。

    Args:
        dets: 检测 dict 列表（须含 'bbox'=(x1,y1,x2,y2)；'conf' 可选）.
        ios_threshold: IoS 阈值；≥1.0 时永不命中，等效关闭去重.

    Returns
    -------
        去重后的检测 dict 列表（按面积降序；元素为原 dict 引用，不改原对象）.

    """
    if not dets or ios_threshold >= 1.0:
        return list(dets)
    boxes = np.asarray([d['bbox'] for d in dets], dtype=float).reshape(-1, 4)
    areas = (np.maximum(0.0, boxes[:, 2] - boxes[:, 0])
             * np.maximum(0.0, boxes[:, 3] - boxes[:, 1]))
    confs = np.array([float(d.get('conf', 0.0)) for d in dets])
    order = sorted(range(len(dets)), key=lambda i: (-areas[i], -confs[i]))
    kept: list = []
    for i in order:
        suppress = False
        for j in kept:
            if areas[i] <= 0.0 or areas[j] <= 0.0:
                continue
            ix1 = max(boxes[i, 0], boxes[j, 0])
            iy1 = max(boxes[i, 1], boxes[j, 1])
            ix2 = min(boxes[i, 2], boxes[j, 2])
            iy2 = min(boxes[i, 3], boxes[j, 3])
            inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
            if inter / min(areas[i], areas[j]) >= ios_threshold:
                suppress = True
                break
            if frag_area_ratio > 0.0:
                small_over_big = min(areas[i], areas[j]) / max(
                    areas[i], areas[j])
                if (small_over_big <= frag_area_ratio
                        and inter / areas[i] >= frag_ios_threshold):
                    suppress = True
                    break
        if not suppress:
            kept.append(i)
    return [dets[i] for i in kept]


DETECTORS.register('yolo', UltralyticsYolo)
SEGMENTERS.register('mobile_sam', MobileSam)
POSE_PIPELINES.register('robust_bag', RobustBagPosePipeline)
POSE_PIPELINES.register('robust_fruit', RobustFruitPosePipeline)
