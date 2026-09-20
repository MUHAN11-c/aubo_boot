"""精化：候选契约、柱/球 refit、袋融合、预抓取残差."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import (
    Callable,
    Iterable,
    List,
    Mapping,
    Optional,
    Tuple,
)

import numpy as np
from peach_harvester.vision.common.bag_landmarks import (
    BagLandmarks,
    clamp_upper_hemisphere,
    enforce_wide_bottom,
    estimate_bag_landmarks,
    OCCLUSION_BRANCH,
    OCCLUSION_DAMAGED,
    OCCLUSION_NEIGHBOR,
    OCCLUSION_UNKNOWN,
)
from peach_harvester.vision.common.geometry import (
    angle_between_deg,
    axis_radial_distance,
    fit_cylinder_robust,
    fit_sphere_robust,
    unit_vector,
    unit_vector as _unit,
)
from peach_harvester.vision.common.tool_budget import ToolBudgetParams
from peach_harvester.vision.domain.budget import (
    CAPABILITY_INVALID,
    CAPABILITY_UNKNOWN,
    evaluate_capabilities,
)
from peach_harvester.vision.domain.evidence import (
    corridor_clear as corridor_is_clear,
    corridor_status,
    occlusion_class as evidence_occlusion,
)
from peach_harvester.vision.domain.model_contract import allowed_from_capabilities
from peach_harvester.vision.target_reconstruction.integrate import (
    require_open3d,
    summarize_view_coverage,
)


_STATUS_ACCEPT = 0
_STATUS_REJECT = 2
_BLOCKING_CANDIDATE_FLAGS = frozenset({
    'tf_stale', 'tf_unavailable', 'target_untracked',
})


def select_reconstruction_candidate(
        msg, base_frame: str, preferred_target_id: str = ''):
    """
    选择可安全用于重建的候选，并返回 (target_id, center_base).

    ``preferred_target_id`` 非空时是身份硬约束：只允许返回该 ID，不能因
    其他目标质量更高而静默换目标。重建同时消费 selected ID 的掩膜，因此
    ID 软排序会把一个目标的掩膜与另一个目标的身份/ROI 混在同一 session。
    """
    if msg is None or not msg.candidates:
        return '', None
    if msg.header.frame_id != base_frame:
        return '', None
    eligible = []
    for cand in msg.candidates:
        candidate_frame = cand.header.frame_id or msg.header.frame_id
        flags = set(cand.diagnostic_flags)
        if candidate_frame != base_frame:
            continue
        if flags & _BLOCKING_CANDIDATE_FLAGS:
            continue
        eligible.append(cand)
    if preferred_target_id:
        eligible = [cand for cand in eligible
                    if cand.target_id == preferred_target_id]
    best = next(
        (cand for cand in eligible if cand.status == _STATUS_ACCEPT), None)
    if best is None:
        best = next(
            (cand for cand in eligible if cand.status != _STATUS_REJECT), None)
    if best is None:
        return '', None
    bottom = np.array([best.bag_bottom.x, best.bag_bottom.y,
                       best.bag_bottom.z], dtype=np.float64)
    neck = np.array([best.bag_neck.x, best.bag_neck.y,
                     best.bag_neck.z], dtype=np.float64)
    if not np.all(np.isfinite(bottom)) or not np.all(np.isfinite(neck)):
        return '', None
    center = 0.5 * (bottom + neck)
    if not np.any(center):
        center = bottom if np.any(bottom) else neck
    if not np.any(center):
        center = None
    return best.target_id, center


# 边界注：本文件 axis_from_vector3 / axis_angle_deg / _normalize_axis_hint
# 原实现按 norm <= 1.0e-9 判废，收敛到共享 unit_vector 后统一为 < 1e-9——
# 恰等于 1e-9 的输入为浮点测度零边界（旧拒/新收一个单位向量），工程上
# 不可观测，按多数实现口径（<）统一。
def axis_from_vector3(direction):
    """geometry_msgs/Vector3 → 有限单位轴；缺失或退化时返回 None."""
    if direction is None:
        return None
    return unit_vector([direction.x, direction.y, direction.z])


def candidate_axis_hint(msg, target_id: str):
    """读取已绑定候选的有限单位轴；缺失或退化时返回 None."""
    if msg is None or not target_id:
        return None
    candidate = next(
        (item for item in msg.candidates if item.target_id == target_id), None)
    if candidate is None:
        return None
    return axis_from_vector3(candidate.translation_direction)


class TargetKindMemory:
    """保存最新类别映射，并在目标离场后保持本轮绑定类别."""

    def __init__(self):
        """初始化为空映射和未绑定状态."""
        self.latest = {}
        self.bound_target_id = ''
        self.bound_kind = ''

    def update(self, fittings) -> None:
        """用当前感知帧更新类别，并刷新已绑定目标的类别."""
        self.latest = {
            fitting.target_id: fitting.target_kind
            for fitting in fittings if fitting.target_id
        }
        if self.bound_target_id in self.latest:
            self.bound_kind = self.latest[self.bound_target_id]

    def bind(self, target_id: str) -> None:
        """开始一轮重建时绑定目标及其当前类别."""
        self.bound_target_id = target_id or ''
        self.bound_kind = self.latest.get(self.bound_target_id, '')

    def reset(self) -> None:
        """结束当前绑定；最新感知映射保留给下一轮启动."""
        self.bound_target_id = ''
        self.bound_kind = ''

    def resolve(self):
        """返回规范化类别及是否发生缺省."""
        kind = self.bound_kind or self.latest.get(self.bound_target_id, '')
        if not kind:
            return 'bag', True
        return ('fruit' if kind == 'fruit' else 'bag'), False


# ── 拟合常量（半径窗与感知包 scene 的设定一致）─────────────────
CYLINDER_RADIUS_RANGE = (0.025, 0.050)  # 袋桃圆柱半径窗 [m]
SPHERE_RADIUS_RANGE = (0.025, 0.045)    # 裸桃球半径窗 [m]
# base_link 重力方向约定（z 向上、重力向下）；消歧规则：轴指向地下即取反
GRAVITY_BASE = np.array([0.0, 0.0, -1.0])

# 输出候选/诊断消息的 status 枚举（与 peach_interfaces 一致）
STATUS_ACCEPT = 0
STATUS_REOBSERVE = 1
STATUS_REJECT = 2


@dataclass
class RefitConfig:
    """
    refit 门控与行为参数（长度 [m]，比率为无量纲）.

    入口/预抓取后撤（refit.entry_standoff_m / pregrasp_standoff_m）不在
    本配置：refit 只出 bottom/neck/axis 等纯几何，入口与预抓取由
    bag_model.fuse_bag_views 按同一组参数构造（grasp_standoffs 注入）。
    """

    cylinder_inlier_min: float = 0.35
    """ACCEPT 门控：内点率下限（圆柱/球共用）."""
    rmse_max_m: float = 0.005
    """ACCEPT 门控：拟合 RMSE 上限 [m]."""
    max_axis_angle_deg: float = 35.0
    """检测轴 vs 精化轴夹角上限 [deg]."""
    normal_neighbors: int = 24
    """法线估计 kNN 邻域点数."""
    seed: int = 0
    """RANSAC 随机种子（固定保证可复现）."""


@dataclass
class RefitResult:
    """
    RefitResult：refit+融合产物唯一载体（节点 ``_refined`` 缓存的类型）.

    W4 起 refit 线与 merge 线共用本 dataclass；键集按 §7 消费表裁剪
    （axis_point / diagnostic_axis_mismatch 等死键已删）。N1：旧单键
    ``axis_angle_deg`` 在 merge 阶段被融合值覆写（refit 值丢失），拆
    ``refit_axis_angle_deg``（refit 轴 vs 感知 hint，_apply_axis_consistency
    写）与 ``fused_axis_angle_deg``（融合轴 vs hint，merge 写）双字段；
    ``as_dict()`` 的兼容投影 ``axis_angle_deg`` 保持旧 JSON 语义
    （融合值优先，融合未发生时回退 refit 值）。
    """

    ok: bool = False
    """refit+融合是否产出可用几何（merge 成功才置 True）."""
    reason: str = ''
    """失败原因（英文标记，写诊断 JSON）；成功为空串."""
    kind: str = ''
    """拟合线名 'cylinder'/'sphere'（N6：merge 不再强制 'cylinder'）."""
    status: int = STATUS_REJECT
    """ACCEPT/REOBSERVE/REJECT 枚举值."""
    center: Optional[np.ndarray] = None
    """(3,) 几何中心 [m]（base 系）."""
    axis: Optional[np.ndarray] = None
    """(3,) bottom→neck 单位轴 [m]."""
    bottom: Optional[np.ndarray] = None
    """(3,) 袋底点 [m]."""
    neck: Optional[np.ndarray] = None
    """(3,) 袋颈点 [m]."""
    entry: Optional[np.ndarray] = None
    """(3,) 套入入口 [m]（仅 merge 写入：bottom − standoff×axis）."""
    pregrasp: Optional[np.ndarray] = None
    """(3,) 预抓取点 [m]（仅 merge 写入）."""
    cut_pose: Optional[np.ndarray] = None
    """(3,) 剪切参考点 [m]（袋口）."""
    cut_plane_point: Optional[np.ndarray] = None
    """(3,) 剪切平面参考点 [m]."""
    cut_travel_m: float = 0.0
    """入口沿轴到剪切参考的行程 [m]."""
    cut_to_fruit_m: float = 0.0
    """剪切参考到果包络上缘的轴向距离 [m]."""
    radius: float = -1.0
    """半径 [m]；merge 后 = 0.5×d95（无效 -1）."""
    diameter: float = -1.0
    """直径 [m]；merge 后 = d95（无效 -1）."""
    d95_m: float = -1.0
    """袋径 95 分位 [m]（仅 merge 写入；无效 -1）."""
    span_m: float = -1.0
    """底→颈跨度 [m]；merge 后 = 融合 length_m（无效 -1）."""
    rmse: float = -1.0
    """拟合 RMSE [m]；merge 后为融合关键点 RMSE（无效 -1）."""
    inlier_ratio: float = -1.0
    """内点率 [0,1]；merge 后为融合一致率（无效 -1）."""
    n_points: int = 0
    """输入点数."""
    radial_margin_m: float = 0.0
    """径向预算余量 [m]（仅 merge 写入）."""
    axial_margin_m: float = 0.0
    """轴向预算余量 [m]（仅 merge 写入）."""
    corridor_clear: bool = False
    """套入走廊净空（仅 merge 写入）."""
    budget: dict = field(default_factory=dict)
    """动态接触预算（evaluate_capabilities 产物）；空=无许可."""
    occlusion_class: str = ''
    """遮挡分类（仅 merge 写入）."""
    fruit_prior_radius_m: float = 0.0
    """果先验半径 [m]（仅 merge 写入）."""
    model_revision: str = ''
    """模型版本号 '<target_id>:<视图数>'（仅 merge 写入）."""
    flags: List[str] = field(default_factory=list)
    """诊断标记（refit 线先写，merge 追加融合标记）."""
    final: bool = False
    """finalize 定稿标记（编排层置位）."""
    envelope_conditioned: bool = False
    """体积包络是否可判定轴向（仅 merge 写入）."""
    envelope_reason: str = ''
    """包络不可判定原因（仅 merge 写入）."""
    axis_conflict_deg: float = 0.0
    """关键点轴 vs 体积包络轴夹角 [deg]（仅 merge 写入）."""
    perception_axis: Optional[List[float]] = None
    """感知 hint 轴 [m]（_apply_axis_consistency 写；无 hint 为 None）."""
    refit_axis_angle_deg: Optional[float] = None
    """N1：refit 轴 vs 感知 hint 夹角 [deg]；无 hint 为 None."""
    fused_axis_angle_deg: Optional[float] = None
    """N1：融合轴 vs 感知 hint 夹角 [deg]；无 hint 为 None."""

    @property
    def axis_angle_deg(self) -> Optional[float]:
        """旧单键语义：融合值优先，融合未发生时回退 refit 值."""
        if self.fused_axis_angle_deg is not None:
            return self.fused_axis_angle_deg
        return self.refit_axis_angle_deg

    @staticmethod
    def _xyz(value) -> Optional[List[float]]:
        if value is None:
            return None
        return [float(v) for v in np.asarray(value).reshape(-1)[:3]]

    def as_dict(self) -> dict:
        """
        兼容投影：diagnostics JSON / metadata.yaml（numpy → 原生类型）.

        键集 = 旧 ``_refined_diag_dict`` 成功投影 + ``reason``/``final``
        + N1 双字段；``axis_angle_deg`` 保持旧值语义（见类 docstring）。
        """
        return {
            'ok': bool(self.ok),
            'kind': str(self.kind),
            'status': int(self.status),
            'reason': str(self.reason),
            'center': self._xyz(self.center),
            'axis': self._xyz(self.axis),
            'bottom': self._xyz(self.bottom),
            'neck': self._xyz(self.neck),
            'diameter': float(self.diameter),
            'span_m': float(self.span_m),
            'rmse': float(self.rmse),
            'inlier_ratio': float(self.inlier_ratio),
            'n_points': int(self.n_points),
            'flags': list(self.flags),
            'axis_angle_deg': (
                None if self.axis_angle_deg is None
                else float(self.axis_angle_deg)),
            'refit_axis_angle_deg': (
                None if self.refit_axis_angle_deg is None
                else float(self.refit_axis_angle_deg)),
            'fused_axis_angle_deg': (
                None if self.fused_axis_angle_deg is None
                else float(self.fused_axis_angle_deg)),
            'axis_conflict_deg': float(self.axis_conflict_deg),
            'envelope_conditioned': bool(self.envelope_conditioned),
            'envelope_reason': str(self.envelope_reason),
            'perception_axis': (
                None if self.perception_axis is None
                else list(self.perception_axis)),
            'final': bool(self.final),
        }


@dataclass
class BagModel:
    """
    多视角袋关键点融合模型（节点 ``_bag_model`` 缓存的类型）.

    W4 起为 dataclass；按 §7 消费表裁掉六个无消费键
    （cut_normal/fruit_prior_auxiliary/envelope_span_m/envelope_d95_m/
    detection_conflict_deg 与 refit 侧 axis_point）。失败路径仅填
    ok/reason/allowed，几何字段保持默认。
    """

    ok: bool = False
    """融合是否成功."""
    reason: str = ''
    """失败原因（英文标记）；成功取预算 reason."""
    bottom: Optional[np.ndarray] = None
    """(3,) 融合袋底 [m]."""
    neck: Optional[np.ndarray] = None
    """(3,) 融合袋颈 [m]."""
    axis: Optional[np.ndarray] = None
    """(3,) 融合轴（bottom→neck 单位向量）."""
    entry: Optional[np.ndarray] = None
    """(3,) 套入入口 [m]."""
    pregrasp: Optional[np.ndarray] = None
    """(3,) 预抓取点 [m]."""
    cut_plane_point: Optional[np.ndarray] = None
    """(3,) 剪切平面参考点 [m]."""
    cut_pose: Optional[np.ndarray] = None
    """(3,) 剪切参考点 [m]."""
    cut_to_fruit_m: float = 0.0
    """剪切参考到果上缘距离 [m]."""
    cut_travel_m: float = 0.0
    """入口到剪切参考行程 [m]."""
    d95_m: float = 0.0
    """袋径 95 分位 [m]."""
    length_m: float = 0.0
    """底→颈轴向长度 [m]."""
    fruit_prior_radius_m: float = 0.0
    """果先验半径 [m]."""
    sigma_position_m: float = 0.0
    """位置 σ [m]."""
    sigma_axis_deg: float = 0.0
    """轴向 σ [deg]."""
    rmse: float = 0.0
    """融合关键点 RMSE [m]."""
    inlier_ratio: float = 0.0
    """关键点一致率 [0,1]."""
    axis_conflict_deg: float = 0.0
    """关键点轴 vs 包络轴夹角 [deg]."""
    envelope_conditioned: bool = False
    """体积包络是否可判定轴向."""
    envelope_reason: str = ''
    """包络不可判定原因."""
    occlusion_class: str = ''
    """遮挡分类."""
    view_count: int = 0
    """参与融合的视角数（对打否决剔除后）."""
    flags: List[str] = field(default_factory=list)
    """融合诊断标记."""
    budget: dict = field(default_factory=dict)
    """动态接触预算（evaluate_capabilities 产物）."""
    allowed: bool = False
    """接触许可（budget.allowed 投影）."""
    radial_margin_m: float = 0.0
    """径向预算余量 [m]."""
    axial_margin_m: float = 0.0
    """轴向预算余量 [m]."""
    corridor_clear: bool = False
    """套入走廊净空."""


def estimate_normals_knn(xyz: np.ndarray, k: int = 24) -> np.ndarray:
    """
    无序点云法线估计（open3d 官方 estimate_normals，单位向量，朝向任意）.

    官方 KDTreeSearchParamKNN(knn=k+1)：kNN 集合含查询点自身，与旧手写
    「cKDTree 查 k+1 个（含自身）→ 批量 eigh」同邻域同算法（协方差最小
    特征向量）；fast_normal_computation=False 走完整特征分解，与 numpy
    eigh 数值路径一致。法线符号由 open3d 内部决定（不保证统一朝向），
    圆柱 RANSAC 对符号不敏感，无需再定向。

    Args:
        xyz: (N, 3) 点 [m]（N ≥ k+1，由调用方保证）.
        k: 邻域点数（不含自身；open3d 侧 knn=k+1 含自身）.

    Returns
    -------
        (N, 3) 单位法线（符号未定向）.

    """
    o3d = require_open3d()
    xyz = np.asarray(xyz, dtype=np.float64)
    # knn 含自身；点数不足 k+1 时夹到全点集（沿用旧手写的小云夹紧语义）
    knn = min(int(k) + 1, xyz.shape[0])
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn),
        fast_normal_computation=False)
    return np.asarray(pcd.normals, dtype=np.float64)


def orient_axis_bottom_to_neck(axis: np.ndarray) -> np.ndarray:
    """
    轴方向消歧：统一为 bottom→neck（neck 在上，抗重力方向）.

    base_link 重力约定 [0,0,-1]：axis·gravity > 0 表示轴指向地下，取反。
    水平轴（点积 ≈0）保持原向——仅保证不指下，由调用方再用投影定底/颈。

    Args:
        axis: (3,) 轴向（无需单位化）.

    Returns
    -------
        (3,) 单位向量，满足 axis·[0,0,1] ≥ 0（不指下）.

    """
    axis = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0])
    axis = axis / norm
    if axis @ GRAVITY_BASE > 0.0:
        axis = -axis
    return axis


def axis_angle_deg(first, second):
    """两轴夹角 [deg]；退化向量返回 None（不取绝对值，翻转算 180°）."""
    return angle_between_deg(first, second)


def _cylinder_ends(points_inl: np.ndarray, axis_point: np.ndarray,
                   axis: np.ndarray) -> Tuple[np.ndarray, np.ndarray,
                                              np.ndarray]:
    """
    圆柱底/颈定位：内点沿轴投影 P10/P90 分位带中位点（投回轴线）.

    先对轴做 bottom→neck 消歧，使投影坐标 t 向上递增：P10 端为底、
    P90 端为颈。分位带半宽取轴向跨度的 5%（下限 2 mm，防退化）；
    带内点取坐标中位数后再投回轴线——圆柱面点带中位点有径向偏移
    （可见面不足整周时尤甚），抓取语义要求 bottom/neck 落在轴上。

    Args:
        points_inl: (M, 3) 拟合内点 [m].
        axis_point: (3,) 轴上一点 [m].
        axis: (3,) 轴向（任意符号，内部消歧）.

    Returns
    -------
        (axis, bottom, neck)：消歧后单位轴与轴上底/颈点 [m].

    """
    axis = orient_axis_bottom_to_neck(axis)
    axis_point = np.asarray(axis_point, dtype=np.float64)
    t = (points_inl - axis_point) @ axis
    lo, hi = np.percentile(t, [10.0, 90.0])
    band = max(0.05 * (hi - lo), 0.002)
    bottom_surf = np.median(points_inl[t <= lo + band], axis=0)
    neck_surf = np.median(points_inl[t >= hi - band], axis=0)
    bottom = axis_point + ((bottom_surf - axis_point) @ axis) * axis
    neck = axis_point + ((neck_surf - axis_point) @ axis) * axis
    return axis, bottom, neck


def _fail(reason: str, n_points: int) -> RefitResult:
    """
    统一失败返回：ok=False、status=REJECT、几何字段全 None/−1.

    Args:
        reason: 失败原因（英文标记，写诊断 JSON）.
        n_points: 输入点数.

    Returns
    -------
        精化失败 RefitResult.

    """
    return RefitResult(
        ok=False, reason=reason, kind='', status=STATUS_REJECT,
        n_points=int(n_points),
        radius=-1.0, diameter=-1.0, span_m=-1.0,
        rmse=-1.0, inlier_ratio=-1.0,
        flags=['refit_failed'])


def _precheck(cloud_xyz) -> Tuple[np.ndarray, Optional[dict]]:
    """
    两条拟合线共用的输入预检：空云/少点优雅失败.

    圆柱 RANSAC 最少 20 点（ransac_cylinder 下限），球线同样要求。

    Args:
        cloud_xyz: 任意形状点集 [m].

    Returns
    -------
        (xyz, fail)：xyz 为 (N, 3) float64；fail 非 None 时应直接返回.

    """
    xyz = np.asarray(cloud_xyz, dtype=np.float64)
    if xyz.size == 0:
        return xyz.reshape(0, 3), _fail('empty_cloud', 0)
    xyz = xyz.reshape(-1, 3)
    if xyz.shape[0] < 20:
        return xyz, _fail('insufficient_points', xyz.shape[0])
    return xyz, None


def _gated_result(kind: str, n_points: int, center: np.ndarray,
                  axis: np.ndarray, bottom: np.ndarray, neck: np.ndarray,
                  span_m: float, est: dict, config: RefitConfig,
                  flags: List[str]) -> RefitResult:
    """
    两条拟合线共用的成功结果组装 + ACCEPT/REOBSERVE 门控.

    inlier_ratio ≥ config.cylinder_inlier_min 且 rmse ≤ config.rmse_max_m
    → ACCEPT，否则 REOBSERVE（标记 low_inlier_ratio/high_rmse）。
    entry/pregrasp/cut_* 不在此写（N11 修正旧 docstring 谎称写 entry）：
    入口与剪切参考由 fuse_bag_views 按同一组 standoff 参数构造、经
    merge_fused_bag_model 并入。

    Args:
        kind: 'cylinder'/'sphere'.
        n_points: 输入点数.
        center: (3,) 几何中心 [m].
        axis: (3,) bottom→neck 单位轴.
        bottom: (3,) 底端点 [m].
        neck: (3,) 颈端点 [m].
        span_m: 底→颈跨度 [m].
        est: 拟合原语结果（radius/rms/inlier_ratio 键）.
        config: 门控参数.
        flags: 拟合线已置的诊断标记（本函数原地追加门控标记）.

    Returns
    -------
        ok=True 的 RefitResult.

    """
    radius = float(est['radius'])
    rmse = float(est['rms'])
    inlier_ratio = float(est['inlier_ratio'])
    status = STATUS_ACCEPT
    if inlier_ratio < config.cylinder_inlier_min:
        flags.append('low_inlier_ratio')
        status = STATUS_REOBSERVE
    if rmse > config.rmse_max_m:
        flags.append('high_rmse')
        status = STATUS_REOBSERVE
    return RefitResult(
        ok=True, reason='', kind=kind, status=status,
        n_points=int(n_points),
        center=center, axis=axis, bottom=bottom, neck=neck,
        radius=radius, diameter=2.0 * radius, span_m=float(span_m),
        rmse=rmse, inlier_ratio=inlier_ratio,
        flags=flags)


def _normalize_axis_hint(axis_hint):
    """可选轴先验 → 有限单位向量；无效给 None."""
    return unit_vector(axis_hint)


def _apply_axis_consistency(result: RefitResult, axis_hint,
                            config: RefitConfig) -> RefitResult:
    """
    Flag axis mismatch; do not change ACCEPT/REOBSERVE here.

    Missing axis hint only records refit_axis_angle_deg=None. Contact still
    follows GraspDecision.allowed from the fused bag budget. N1: the refit
    angle lives in refit_axis_angle_deg — the fused angle is written later by
    merge_fused_bag_model (the old single axis_angle_deg key was overwritten
    there, losing the refit value).

    Args:
        result: _gated_result ok=True result, updated in place.
        axis_hint: optional detection axis, normalized inside.
        config: gate parameters including max_axis_angle_deg.

    Returns
    -------
        The same RefitResult.

    """
    hint = _normalize_axis_hint(axis_hint)
    if hint is None:
        result.refit_axis_angle_deg = None
        return result
    angle = axis_angle_deg(result.axis, hint)
    result.refit_axis_angle_deg = angle
    result.perception_axis = [float(v) for v in hint]
    max_deg = float(config.max_axis_angle_deg)
    if angle is not None and angle > max_deg:
        result.flags.append('perception_reconstruction_axis_mismatch')
    return result


class CylinderRefitter:
    """
    袋桃圆柱精化：法线估计 + 圆柱 RANSAC + 消歧.

    无状态（RefitConfig 随调用传入）；target_kind 仅为选线对齐，
    本实现恒走圆柱线。axis_hint 只做夹角诊断，不授权接触。
    """

    def refit(self, cloud_xyz: np.ndarray, target_kind: str = 'bag',
              config: Optional[RefitConfig] = None,
              axis_hint=None) -> dict:
        """
        圆柱 RANSAC 精化 + bottom→neck 消歧 + ACCEPT/REOBSERVE 门控.

        Args:
            cloud_xyz: (N, 3) 点 [m]（base_frame）；空云/少点优雅失败.
            target_kind: 忽略（恒圆柱线；选线由 select_refitter 负责）.
            config: RefitConfig；None 用默认.
            axis_hint: 可选 bottom→neck 单位方向（base_frame）；用于
                与圆柱精化轴做夹角门，不再忽略.

        Returns
        -------
            RefitResult.

        """
        del target_kind  # 圆柱线不使用（选线在 select_refitter）
        config = config or RefitConfig()
        xyz, fail = _precheck(cloud_xyz)
        if fail is not None:
            return fail
        n = xyz.shape[0]
        normals = estimate_normals_knn(xyz, config.normal_neighbors)
        est = fit_cylinder_robust(
            xyz, normals, radius_range=CYLINDER_RADIUS_RANGE,
            seed=config.seed)
        if est is None:
            return _fail('cylinder_fit_failed', n)
        axis, bottom, neck = _cylinder_ends(
            xyz[est['inliers']], est['q0'], est['axis'])
        center = 0.5 * (bottom + neck)
        span = float(np.linalg.norm(neck - bottom))
        result = _gated_result(
            'cylinder', n, center, axis, bottom, neck, span,
            est, config, flags=[])
        return _apply_axis_consistency(result, axis_hint, config)


class SphereRefitter:
    """
    裸桃球精化：球拟合 + 果梗方向先验消歧.

    球面旋转对称，球拟合只能精化 center/radius，不能凭空产生果梗轴：
    优先沿用 axis_hint（绑定目标在 base 系的单帧果梗/凹陷方向），先验
    缺失才显式退 +Z 并打 `fruit_axis_defaulted` 诊断标记。
    """

    def refit(self, cloud_xyz: np.ndarray, target_kind: str = 'fruit',
              config: Optional[RefitConfig] = None,
              axis_hint=None) -> dict:
        """
        球拟合精化 + 方向先验消歧 + ACCEPT/REOBSERVE 门控.

        Args:
            cloud_xyz: (N, 3) 点 [m]（base_frame）；空云/少点优雅失败.
            target_kind: 忽略（恒球线；选线由 select_refitter 负责）.
            config: RefitConfig；None 用默认.
            axis_hint: 可选 bottom→neck 单位方向（base_frame）.

        Returns
        -------
            RefitResult.

        """
        del target_kind  # 球线不使用（选线在 select_refitter）
        config = config or RefitConfig()
        xyz, fail = _precheck(cloud_xyz)
        if fail is not None:
            return fail
        n = xyz.shape[0]
        est = fit_sphere_robust(
            xyz, None, radius_prior=None,
            radius_range=SPHERE_RADIUS_RANGE, seed=config.seed)
        if est is None:
            return _fail('sphere_fit_failed', n)
        center = np.asarray(est['center'], dtype=np.float64)
        # 球面旋转对称，球拟合只能精化 center/radius，不能凭空产生果梗轴。
        # 优先沿用绑定目标在 base 系的果梗/凹陷方向；先验缺失才显式退 +Z。
        hint = None if axis_hint is None else np.asarray(
            axis_hint, dtype=np.float64).reshape(-1)
        if (hint is not None and hint.size == 3
                and np.all(np.isfinite(hint))
                and np.linalg.norm(hint) > 1.0e-9):
            axis = hint / np.linalg.norm(hint)
            axis_flag = 'fruit_axis_from_perception'
        else:
            axis = np.array([0.0, 0.0, 1.0])
            axis_flag = 'fruit_axis_defaulted'
        bottom = center - est['radius'] * axis
        neck = center + est['radius'] * axis
        span = 2.0 * float(est['radius'])
        result = _gated_result(
            'sphere', n, center, axis, bottom, neck, span,
            est, config, flags=[axis_flag])
        return _apply_axis_consistency(result, axis_hint, config)


def select_refitter(refitters: Mapping[str, object],
                    target_kind: str):
    """
    按 target_kind 选 refitter：'fruit'→球线，其余一律圆柱线.

    未知/空 kind 缺省袋桃（圆柱线），与感知包 `target_kind or 'bag'`
    语义一致。refitters 至少含 'cylinder' 与 'sphere' 两键（节点构造
    期由 yaml refitter.*_impl 装配）。

    Args:
        refitters: {'cylinder': CylinderRefitter, 'sphere': SphereRefitter}.
        target_kind: 'bag'/'fruit'（来自感知 diagnostics）.

    Returns
    -------
        选中的 refitter 实例.

    """
    return refitters['sphere' if target_kind == 'fruit' else 'cylinder']


# 柱/球两条精化线的实现映射（yaml refitter.cylinder_impl / sphere_impl）。
# 新增拟合线 = 加一个类 + 这里一项。
REFITTERS_BY_IMPL = {
    'cylinder_refit': CylinderRefitter,
    'sphere_refit': SphereRefitter,
}


def make_refitter(impl_name: str):
    """按 yaml 实现名构造精化器；未知名列出全部可用名后抛错."""
    cls = REFITTERS_BY_IMPL.get(impl_name)
    if cls is None:
        raise ValueError(
            f'未知精化器实现 {impl_name!r}，可用: {sorted(REFITTERS_BY_IMPL)}')
    return cls()


def _huber_mean(points: np.ndarray, k: float = 0.02) -> Optional[np.ndarray]:
    """三维点 Huber 加权均值."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return None
    center = np.median(pts, axis=0)
    for _ in range(8):
        delta = pts - center
        dist = np.linalg.norm(delta, axis=1)
        weights = np.ones(pts.shape[0])
        far = dist > k
        weights[far] = k / np.maximum(dist[far], 1e-9)
        denom = float(weights.sum())
        if denom < 1e-9:
            break
        center = (weights[:, None] * pts).sum(axis=0) / denom
    return center


def _signed_angle_deg(first, second) -> float:
    """有符号夹角 [deg]；反向为 180°，不取绝对值；退化输入按 180°."""
    angle = angle_between_deg(first, second)
    return 180.0 if angle is None else angle


def median_absolute_deviation_m(points: np.ndarray, center: np.ndarray) -> float:
    """点到中心距离的 MAD→σ [m]."""
    delta = np.linalg.norm(
        np.asarray(points, dtype=np.float64) - center, axis=1)
    if delta.size == 0:
        return 0.0
    return float(1.4826 * np.median(delta))


def _finite_cloud(points) -> Optional[np.ndarray]:
    """有限三维点；不足则 None."""
    if points is None:
        return None
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 8 or pts.shape[1] != 3:
        return None
    finite = pts[np.isfinite(pts).all(axis=1)]
    if finite.shape[0] < 8:
        return None
    return finite


def slice_centroid(points, axis, origin, t_m: float,
                   half_width_m: float = 0.02) -> Optional[np.ndarray]:
    """沿轴在 t 处取截面质心；点数不足则 None."""
    axis_u = _unit(axis)
    cloud = _finite_cloud(points)
    origin_v = np.asarray(origin, dtype=np.float64).reshape(3)
    if axis_u is None or cloud is None:
        return None
    axial = (cloud - origin_v) @ axis_u
    selected = cloud[np.abs(axial - float(t_m)) <= float(half_width_m)]
    if selected.shape[0] < 8:
        return None
    return selected.mean(axis=0)


def snap_lateral(point, axis, centroid) -> np.ndarray:
    """把点沿垂直于轴的方向贴到截面质心，轴向坐标不变."""
    axis_u = _unit(axis)
    src = np.asarray(point, dtype=np.float64).reshape(3)
    if axis_u is None or centroid is None:
        return src
    delta = np.asarray(centroid, dtype=np.float64).reshape(3) - src
    return src + delta - float(delta @ axis_u) * axis_u


def envelope_axis_from_cloud(
        points, keypoint_axis, min_aspect: float = 1.0,
        min_length_m: float = 0.05, min_slices: int = 4) -> dict:
    """
    沿关键点轴切 TSDF/点云截面，用截面质心拟合包络主方向.

    接触轴仍由关键点融合给出。本函数只回答体积是否沿该轴延伸。
    轴向跨度小于直径或切片不足时 conditioned=False，不得用 12° 否决.
    """
    empty = {
        'conditioned': False, 'axis': None, 'span_m': 0.0, 'd95_m': 0.0,
        'reason': 'envelope_cloud_insufficient'}
    axis = _unit(keypoint_axis)
    finite = _finite_cloud(points)
    if axis is None or finite is None or finite.shape[0] < 30:
        return empty
    origin = np.median(finite, axis=0)
    axial = (finite - origin) @ axis
    t_lo, t_hi = np.percentile(axial, [5.0, 95.0])
    span = float(t_hi - t_lo)
    d95 = float(2.0 * np.percentile(
        axis_radial_distance(finite, axis, origin), 95))
    if span < min_length_m or (d95 > 1e-6 and span / d95 < min_aspect):
        return {
            'conditioned': False, 'axis': None, 'span_m': span, 'd95_m': d95,
            'reason': 'envelope_axis_ill_conditioned'}
    bins = 8
    edges = np.linspace(t_lo, t_hi, bins + 1)
    centroids = []
    for index in range(bins):
        selected = finite[(axial >= edges[index]) & (axial < edges[index + 1])]
        if selected.shape[0] < 8:
            continue
        centroids.append(selected.mean(axis=0))
    if len(centroids) < min_slices:
        return {
            'conditioned': False, 'axis': None, 'span_m': span, 'd95_m': d95,
            'reason': 'envelope_too_few_slices'}
    centered = np.stack(centroids) - np.mean(centroids, axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    envelope = _unit(vt[0])
    if envelope is None:
        return {
            'conditioned': False, 'axis': None, 'span_m': span, 'd95_m': d95,
            'reason': 'envelope_axis_undefined'}
    if float(envelope @ axis) < 0.0:
        envelope = -envelope
    return {
        'conditioned': True, 'axis': envelope, 'span_m': span, 'd95_m': d95,
        'reason': 'ok'}


def _corridor_clear(
        points, axis, origin, t0: float, t1: float,
        d_inner: float, wall_clearance: float) -> bool:
    """沿套入区间切片，袋径不得超过工具内净空."""
    axis_u = _unit(axis)
    cloud = _finite_cloud(points)
    origin_v = np.asarray(origin, dtype=np.float64).reshape(3)
    if axis_u is None or cloud is None:
        return False
    lo = min(float(t0), float(t1))
    hi = max(float(t0), float(t1))
    if hi - lo < 0.02:
        hi = lo + 0.02
    axial = (cloud - origin_v) @ axis_u
    limit = 0.5 * float(d_inner) - float(wall_clearance)
    bins = 6
    edges = np.linspace(lo, hi, bins + 1)
    seen = 0
    for index in range(bins):
        selected = cloud[(axial >= edges[index]) & (axial < edges[index + 1])]
        if selected.shape[0] < 8:
            continue
        seen += 1
        mid = selected.mean(axis=0)
        if float(np.percentile(
                axis_radial_distance(selected, axis_u, mid), 95)) > limit:
            return False
    return seen >= 3


def _cut_station(
        bottom, neck, axis, length_m: float,
        fruit_center, fruit_r: float,
        params: ToolBudgetParams,
        neck_margin_m: float = 0.0) -> dict:
    """
    轴上剪切参考：袋口（分割贴检测框极限），不取果包络与袋颈中点.

    果距不足时刀仍放在口，只把 safe_band 置假（拦套入，不挪刀）。
    看不见果先验时给出袋口参考，但不宣称果距。
    """
    del neck_margin_m
    bottom_v = np.asarray(bottom, dtype=np.float64).reshape(3)
    axis_u = _unit(axis)
    t_neck = float(length_m)
    fruit_hi = None
    if axis_u is None:
        return {
            'cut': np.asarray(neck, dtype=np.float64).reshape(3),
            't_cut_m': t_neck,
            'cut_to_fruit_m': 0.0,
            'safe_band': False,
        }
    t_cut = t_neck
    t_min = 0.0
    if fruit_center is not None and float(fruit_r) > 1e-6:
        t_fruit = float(
            (np.asarray(fruit_center, dtype=np.float64).reshape(3)
             - bottom_v) @ axis_u)
        fruit_hi = t_fruit + float(fruit_r)
        t_min = fruit_hi + float(params.fruit_safety_clearance)
    cut_to_fruit = 0.0
    if fruit_hi is not None:
        cut_to_fruit = float(t_cut - fruit_hi)
    return {
        'cut': bottom_v + t_cut * axis_u,
        't_cut_m': float(t_cut),
        'cut_to_fruit_m': cut_to_fruit,
        'safe_band': bool(fruit_hi is not None and t_cut >= t_min),
    }


def _majority_sense_views(items: list) -> Optional[list]:
    """口/底朝向多数一致的视角；对打且无多数则 None（否决不平均）."""
    if len(items) <= 1:
        return list(items)
    axes = []
    for item in items:
        axis = _unit(item.bag_axis)
        if axis is None:
            axis = _unit(item.neck_center - item.bottom_center)
        axes.append(axis)
    usable = [(i, ax) for i, ax in enumerate(axes) if ax is not None]
    if not usable:
        return None
    best_i = usable[0][0]
    best_count = -1
    for i, ax in usable:
        count = sum(1 for _, other in usable if float(np.dot(ax, other)) > 0.0)
        if count > best_count:
            best_count = count
            best_i = i
    ref = axes[best_i]
    cluster = [
        items[i] for i, ax in enumerate(axes)
        if ax is not None and float(np.dot(ax, ref)) > 0.0]
    if len(cluster) * 2 <= len(items):
        return None
    return cluster


def fuse_bag_views(
        views: Iterable[BagLandmarks],
        params: ToolBudgetParams | None = None,
        cloud_xyz=None,
        detection_axis=None,
        entry_standoff_m: float = 0.0,
        pregrasp_standoff_m: float = 0.0) -> BagModel:
    """
    融合多视角袋关键点.

    方向：袋底→袋颈 Huber，且只许上半球（左右最多水平）。口底对打的视角否决不平均。
    定位：体积截面质心只改侧向。
    剪切站：袋口 / 分割贴检测框极限；果距不足只否决 allowed，不挪刀。
    包络长径比不足则跳过 12° 否决。检测轴夹角只诊断，不进接触预算。
    后撤量由调用方传入（节点从 ROS 参数读，不在本函数写死米数）。
    W4：返回 BagModel（§7 消费表裁掉 detection_conflict_deg /
    envelope_span_m / envelope_d95_m / cut_normal / fruit_prior_auxiliary
    五个无消费输出键；detection_axis 形参保留调用方契约，不再产出键）。
    """
    cfg = params or ToolBudgetParams()
    items = [item for item in views if item.neck_center is not None
             and item.bottom_center is not None]
    if not items:
        return BagModel(ok=False, reason='no_landmark_views', allowed=False)
    aligned = _majority_sense_views(items)
    if not aligned:
        return BagModel(ok=False, reason='landmark_axis_conflict', allowed=False)
    n_dropped = len(items) - len(aligned)
    items = aligned
    bottoms = np.stack([item.bottom_center for item in items])
    necks = np.stack([item.neck_center for item in items])
    axes = []
    for item in items:
        axis = _unit(item.bag_axis)
        if axis is not None:
            axes.append(axis)
    bottom = _huber_mean(bottoms)
    neck = _huber_mean(necks)
    axis = _unit(neck - bottom)
    if axis is None and axes:
        axis = _unit(np.mean(np.stack(axes), axis=0))
    if bottom is None or neck is None or axis is None:
        return BagModel(ok=False, reason='fusion_failed', allowed=False)
    d95_values = [item.d95_m for item in items if item.d95_m > 0]
    # 全部视角 d95 缺失（0/负）时回退 0：下游预算把 0 当「无径向散布数据」
    # 处理，不得让 np.median([]) 的 NaN 流进许可与 diagnostics JSON。
    d95 = float(np.median(d95_values)) if d95_values else 0.0
    length = float(np.dot(neck - bottom, axis))
    if length < 0.0:
        axis = -axis
        length = -length
        bottom, neck = neck, bottom
    mid = slice_centroid(cloud_xyz, axis, bottom, 0.5 * max(length, 0.02))
    if mid is None:
        mid = slice_centroid(cloud_xyz, axis, bottom, 0.0)
    if mid is not None:
        bottom = snap_lateral(bottom, axis, mid)
        neck = snap_lateral(neck, axis, mid)
        length = float(np.dot(neck - bottom, axis))
        if length < 0.0:
            axis = -axis
            length = -length
            bottom, neck = neck, bottom
    bottom, neck, axis, width_flipped = enforce_wide_bottom(
        bottom, neck, axis, cloud_xyz)
    bottom, neck, axis, up_flipped = clamp_upper_hemisphere(
        bottom, neck, axis, np.array([0.0, 0.0, -1.0], dtype=np.float64))
    length = float(np.dot(neck - bottom, axis))
    view_sigma = float(np.median([item.sigma_position_m for item in items]))
    sig_p = max(
        view_sigma,
        median_absolute_deviation_m(bottoms, bottom),
        median_absolute_deviation_m(necks, neck), 0.006)
    sig_a = float(np.median([item.sigma_axis_deg for item in items]))
    bottom_err = np.linalg.norm(bottoms - bottom, axis=1)
    neck_err = np.linalg.norm(necks - neck, axis=1)
    combined = np.concatenate([bottom_err, neck_err])
    rmse = float(np.sqrt(np.mean(combined * combined)))
    inlier_ratio = float(np.mean(combined < 0.02))
    envelope = envelope_axis_from_cloud(cloud_xyz, axis)
    axis_conflict_deg = 0.0
    if envelope.get('conditioned'):
        axis_conflict_deg = _signed_angle_deg(axis, envelope.get('axis'))
    axis_error_deg = max(sig_a, axis_conflict_deg)
    env_d95 = float(envelope.get('d95_m') or 0.0)
    if env_d95 > 1e-6:
        d95 = max(d95, env_d95) if d95 > 1e-6 else env_d95
    fruit_centers = [
        item.fruit_prior_center for item in items
        if item.fruit_prior_center is not None]
    fruit_r_vals = [
        item.fruit_prior_radius_m for item in items
        if item.fruit_prior_radius_m > 0]
    fruit_center = (
        _huber_mean(np.stack(fruit_centers)) if fruit_centers else None)
    fruit_r = float(np.median(fruit_r_vals)) if fruit_r_vals else 0.0
    cut = _cut_station(
        bottom, neck, axis, length, fruit_center, fruit_r, cfg)
    standoff = float(entry_standoff_m)
    entry = bottom - standoff * axis
    pregrasp = entry - float(pregrasp_standoff_m) * axis
    cut_travel = float(np.dot(cut['cut'] - entry, axis))
    if d95 <= 1e-6:
        # 无任何径向尺度证据（逐视角 d95 与体积包络全缺）：d_bag95=0 会拿到
        # 最宽松的径向预算（袋当零宽），保守拒绝而不是放行。12=几何超限族。
        budget = {
            'allowed': False, 'reason': 'bag_d95_missing', 'failure_code': 12,
            'geometry_capability': CAPABILITY_INVALID,
            'pregrasp_capability': CAPABILITY_INVALID,
            'sleeve_capability': CAPABILITY_INVALID,
            'cut_capability': CAPABILITY_INVALID}
    else:
        budget = evaluate_capabilities(
            d_bag95=d95, length_m=max(length, 0.05),
            center_lateral95=sig_p, axis_error_deg=axis_error_deg,
            neck_position95=sig_p,
            cut_to_fruit_m=float(cut['cut_to_fruit_m']),
            params=cfg)
    raw_occlusion = next(
        (str(item.occlusion_class or '') for item in items
         if str(item.occlusion_class or '')),
        '')
    occlusion = evidence_occlusion(
        inputs_available=bool(raw_occlusion), classified=raw_occlusion)
    flags = []
    if n_dropped:
        flags.append('landmark_views_vetoed')
    if width_flipped:
        flags.append('taper_polarity_swapped')
    if up_flipped:
        flags.append('polarity_upper_hemisphere')
    if not envelope.get('conditioned'):
        flags.append(str(
            envelope.get('reason') or 'envelope_axis_ill_conditioned'))
    if envelope.get('conditioned') and axis_conflict_deg > 12.0:
        flags.append('keypoint_cloud_axis_conflict')
        budget['sleeve_capability'] = CAPABILITY_INVALID
        budget['reason'] = 'keypoint_cloud_axis_conflict'
        budget['failure_code'] = 3
    if not cut['safe_band']:
        flags.append('cut_band_unavailable')
        budget['cut_capability'] = CAPABILITY_INVALID
        budget['reason'] = 'cut_plane_fruit_clearance'
        budget['failure_code'] = 16
    if occlusion in (
            OCCLUSION_BRANCH, OCCLUSION_NEIGHBOR, OCCLUSION_DAMAGED,
            OCCLUSION_UNKNOWN):
        flags.append(occlusion)
        budget['sleeve_capability'] = CAPABILITY_INVALID
        budget['reason'] = 'occlusion_' + occlusion
        budget['failure_code'] = 3
    points_present = _finite_cloud(cloud_xyz) is not None
    blocked = points_present and not _corridor_clear(
        cloud_xyz, axis, bottom, 0.0, cut['t_cut_m'],
        cfg.d_inner, cfg.wall_clearance)
    corridor = corridor_is_clear(
        corridor_status(points_present=points_present, blocked=blocked))
    if not corridor:
        budget['sleeve_capability'] = CAPABILITY_INVALID
    budget['allowed'] = allowed_from_capabilities(
        int(budget.get('geometry_capability', CAPABILITY_UNKNOWN)),
        int(budget.get('pregrasp_capability', CAPABILITY_UNKNOWN)),
        int(budget.get('sleeve_capability', CAPABILITY_UNKNOWN)),
        int(budget.get('cut_capability', CAPABILITY_UNKNOWN)))
    return BagModel(
        ok=True,
        bottom=bottom,
        neck=neck,
        axis=axis,
        entry=entry,
        pregrasp=pregrasp,
        cut_plane_point=cut['cut'],
        cut_pose=cut['cut'],
        cut_to_fruit_m=float(cut['cut_to_fruit_m']),
        cut_travel_m=cut_travel,
        d95_m=d95,
        length_m=length,
        fruit_prior_radius_m=fruit_r,
        sigma_position_m=sig_p,
        sigma_axis_deg=sig_a,
        rmse=rmse,
        inlier_ratio=inlier_ratio,
        axis_conflict_deg=axis_conflict_deg,
        envelope_conditioned=bool(envelope.get('conditioned')),
        envelope_reason=str(envelope.get('reason') or ''),
        occlusion_class=occlusion,
        view_count=len(items),
        flags=flags,
        budget=budget,
        allowed=bool(budget.get('allowed')),
        reason=budget.get('reason', ''),
        radial_margin_m=float(budget.get('radial_margin_m', 0.0)),
        axial_margin_m=float(budget.get('axial_margin_m', 0.0)),
        corridor_clear=bool(corridor),
    )


def collect_bag_views(frames, target_center,
                      on_view: Optional[Callable] = None) -> List[BagLandmarks]:
    """
    Extract bag landmarks, one cloud per camera pose cluster.

    W4 自节点 ``_collect_bag_views`` 下沉（数学零改动）：按机位聚类选取
    代表帧（每簇取有效深度占比最高者），对代表帧 base 系点云估计袋
    关键点。on_view(landmarks, frame) 在每个视角估计成功后回调（节点
    用于追加 geometry.jsonl 视角行；无 IO 时传 None）。

    Args:
        frames: 已采帧列表（collector.frames 快照）.
        target_center: (3,) 绑定目标中心（base 系 [m]）；None 时退全帧.
        on_view: 可选逐视角回调（不得抛出——视角行写失败不拦融合）.

    Returns
    -------
        BagLandmarks 列表（每个代表视角一条）.

    """
    coverage = summarize_view_coverage(frames, target_center)
    selected = []
    if coverage.get('views'):
        for pose in coverage['views']:
            indices = pose.get('member_indices') or [pose['index']]
            valid = [
                index for index in indices
                if 0 <= int(index) < len(frames)]
            if not valid:
                continue
            best = max(
                valid,
                key=lambda index: float(frames[index].valid_depth_ratio))
            selected.append(frames[best])
    else:
        selected = list(frames)
    views = []
    for frame in selected:
        cloud = getattr(frame, 'cloud_base', None)
        if cloud is None:
            continue
        cloud = np.asarray(cloud, dtype=np.float64)
        if cloud.ndim != 2 or cloud.shape[0] < 30:
            continue
        landmarks = estimate_bag_landmarks(
            cloud,
            gravity=np.array([0.0, 0.0, -1.0], dtype=np.float64),
            valid_depth_ratio=float(frame.valid_depth_ratio))
        views.append(landmarks)
        if on_view is not None:
            on_view(landmarks, frame)
    return views


def merge_fused_bag_model(result: RefitResult, fused: BagModel,
                          views_count: int, bound_axis_hint,
                          target_id: str = '') -> RefitResult:
    """
    Merge fused geometry into refit result; drop budget on fusion fail.

    W4 自节点 ``_merge_fused_bag_model`` 下沉改纯函数（数学零改动）。
    不写任何缓存：``_refined``/``_bag_model`` 的成对写入权在编排层
    ``_run_refit``（见其注释），否则 keep_last_good 分支会留下新旧
    混合的缓存对。N1：融合轴夹角写 ``fused_axis_angle_deg``（不再覆写
    refit 值）。N6：不再强制 kind='cylinder'——保留 refit 线原 kind
    （cylinder/sphere），下游 _refined_fitting_msg 按 kind=='sphere' 判
    'fruit'，果目标不再误报袋。旧 merge 写的 diagnostic_axis_mismatch
    为死键（无消费），随裁剪删除——诊断 mismatch 由 _grasp_decision
    现场计算。
    """
    if not fused.ok:
        result.flags.append('bag_fusion_required')
        result.ok = False
        result.budget = {}
        result.corridor_clear = False
        result.status = STATUS_REOBSERVE
        result.reason = str(
            fused.reason or result.reason or 'bag_model_unavailable')
        return result
    result.ok = True
    result.bottom = fused.bottom
    result.neck = fused.neck
    result.axis = fused.axis
    result.center = 0.5 * (
        np.asarray(fused.bottom) + np.asarray(fused.neck))
    result.d95_m = fused.d95_m
    result.diameter = fused.d95_m
    result.radius = 0.5 * float(fused.d95_m)
    result.rmse = float(fused.rmse or 0.0)
    result.inlier_ratio = float(fused.inlier_ratio or 0.0)
    result.radial_margin_m = fused.radial_margin_m
    result.axial_margin_m = fused.axial_margin_m
    result.corridor_clear = fused.corridor_clear
    result.budget = fused.budget or {}
    result.occlusion_class = fused.occlusion_class
    result.fruit_prior_radius_m = fused.fruit_prior_radius_m
    result.model_revision = f'{target_id}:{views_count}'
    result.span_m = float(fused.length_m or 0.0)
    result.fused_axis_angle_deg = axis_angle_deg(
        fused.axis, bound_axis_hint)
    # 融合成功即可接近预抓取；接触许可仍只看 budget.allowed。
    result.status = STATUS_ACCEPT
    result.flags.extend(fused.flags or [])
    result.envelope_conditioned = bool(fused.envelope_conditioned)
    result.envelope_reason = str(fused.envelope_reason or '')
    result.axis_conflict_deg = float(fused.axis_conflict_deg or 0.0)
    result.entry = fused.entry
    result.pregrasp = fused.pregrasp
    result.cut_pose = (
        fused.cut_pose if fused.cut_pose is not None else fused.neck)
    result.cut_plane_point = (
        fused.cut_plane_point if fused.cut_plane_point is not None
        else fused.neck)
    result.cut_travel_m = float(fused.cut_travel_m or 0.0)
    result.cut_to_fruit_m = float(fused.cut_to_fruit_m or 0.0)
    return result


def _pregrasp_axis_angle_deg(tool_axis, bag_axis) -> float:
    """两轴夹角（度）；退化输入按 180°."""
    angle = angle_between_deg(tool_axis, bag_axis)
    return 180.0 if angle is None else angle


def lateral_axial_errors(
        tool_point, model_point, bag_axis) -> tuple:
    """工具点相对模型点的侧向/轴向残差 [m]."""
    axis = _unit(bag_axis)
    if axis is None or tool_point is None or model_point is None:
        return 1.0, 1.0
    delta = np.asarray(tool_point, dtype=np.float64) - np.asarray(
        model_point, dtype=np.float64)
    axial = float(np.dot(delta, axis))
    lateral = float(np.linalg.norm(delta - axial * axis))
    return lateral, axial


def evaluate_pregrasp(
        tool_axis, bag_axis, sleeve_mouth, bag_bottom, cutting_plane,
        cut_plane_point, radial_margin_m: float, axial_margin_m: float,
        previous=None, max_angle_deg: float = 2.0,
        max_lateral_m: float = 0.003) -> dict:
    """
    停稳后的预抓取定量残差（工具变换相对冻结模型）.

    径向/轴向预算是接触许可，不参与本残差是否通过。
    previous 为上一帧同结构 dict 时检查两帧一致性。
    """
    angle = _pregrasp_axis_angle_deg(tool_axis, bag_axis)
    lat_b, _ = lateral_axial_errors(sleeve_mouth, bag_bottom, bag_axis)
    lat_c, ax_c = lateral_axial_errors(
        cutting_plane, cut_plane_point, bag_axis)
    lateral = max(lat_b, lat_c)
    consistent = True
    if previous is not None:
        consistent = (
            abs(angle - float(previous.get('axis_angle_deg', angle))) < 1.5
            and abs(lateral - float(previous.get('lateral_error_m', lateral)))
            < 0.004)
    passed = (
        consistent
        and angle <= max_angle_deg
        and lateral <= max_lateral_m)
    needs = (not passed) and consistent
    reason = 'pregrasp_verified'
    failure_code = 0
    if not consistent:
        reason = 'pregrasp_frames_inconsistent'
        failure_code = 4
    elif not passed:
        reason = 'pregrasp_residual'
        failure_code = 4
    return {
        'axis_angle_deg': float(angle),
        'lateral_error_m': float(lateral),
        'axial_error_m': float(ax_c),
        'radial_margin_m': float(radial_margin_m),
        'axial_margin_m': float(axial_margin_m),
        'frames_consistent': bool(consistent),
        'needs_correction': bool(needs),
        'passed': bool(passed),
        'reason': reason,
        'failure_code': int(failure_code),
    }
