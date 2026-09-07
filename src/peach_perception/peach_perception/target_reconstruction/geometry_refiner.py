from __future__ import annotations
"""精化：柱/球 refit 与候选契约。"""

from dataclasses import dataclass
from typing import (
    List,
    Mapping,
    Optional,
    Tuple,
)

import numpy as np
from peach_perception.common.geometry import (
    angle_between_deg,
    fit_cylinder_robust,
    fit_sphere_robust,
    unit_vector,
)
from peach_perception.target_reconstruction.integrate import require_open3d

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

    cylinder_inlier_min: float = 0.35  # ACCEPT 门控：内点率下限（圆柱/球共用）
    rmse_max_m: float = 0.005          # ACCEPT 门控：拟合 RMSE 上限 [m]
    max_axis_angle_deg: float = 35.0   # 检测轴 vs 精化轴夹角上限 [deg]
    normal_neighbors: int = 24         # 法线估计 kNN 邻域点数
    seed: int = 0                      # RANSAC 随机种子（固定保证可复现）


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


def _fail(reason: str, n_points: int) -> dict:
    """
    统一失败返回：ok=False、status=REJECT、几何字段全 None/−1.

    Args:
        reason: 失败原因（英文标记，写诊断 JSON）.
        n_points: 输入点数.

    Returns
    -------
        精化失败返回 dict.

    """
    return {
        'ok': False, 'reason': reason, 'kind': '', 'status': STATUS_REJECT,
        'n_points': int(n_points),
        'center': None, 'axis': None, 'axis_point': None,
        'bottom': None, 'neck': None, 'entry': None,
        'radius': -1.0, 'diameter': -1.0, 'span_m': -1.0,
        'rmse': -1.0, 'inlier_ratio': -1.0,
        'flags': ['refit_failed'],
    }


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
                  axis: np.ndarray, axis_point: np.ndarray,
                  bottom: np.ndarray, neck: np.ndarray, span_m: float,
                  est: dict, config: RefitConfig,
                  flags: List[str]) -> dict:
    """
    两条拟合线共用的成功结果组装 + ACCEPT/REOBSERVE 门控.

    inlier_ratio ≥ config.cylinder_inlier_min 且 rmse ≤ config.rmse_max_m
    → ACCEPT，否则 REOBSERVE（标记 low_inlier_ratio/high_rmse）。

    Args:
        kind: 'cylinder'/'sphere'.
        n_points: 输入点数.
        center: (3,) 几何中心 [m].
        axis: (3,) bottom→neck 单位轴.
        axis_point: (3,) 轴上一点 [m].
        bottom: (3,) 底端点 [m].
        neck: (3,) 颈端点 [m].
        span_m: 底→颈跨度 [m].
        est: 拟合原语结果（radius/rms/inlier_ratio 键）.
        config: 门控参数.
        flags: 拟合线已置的诊断标记（本函数原地追加门控标记）.

    Returns
    -------
        ok=True 的 RefitResult dict（entry = bottom − axis×standoff）.

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
    return {
        'ok': True, 'reason': '', 'kind': kind, 'status': status,
        'n_points': int(n_points),
        'center': center, 'axis': axis, 'axis_point': axis_point,
        'bottom': bottom, 'neck': neck,
        'radius': radius, 'diameter': 2.0 * radius, 'span_m': float(span_m),
        'rmse': rmse, 'inlier_ratio': inlier_ratio,
        'flags': flags,
    }


def _normalize_axis_hint(axis_hint):
    """可选轴先验 → 有限单位向量；无效给 None."""
    return unit_vector(axis_hint)


def _apply_axis_consistency(result: dict, axis_hint, config: RefitConfig) -> dict:
    """
    Flag axis mismatch; do not change ACCEPT/REOBSERVE here.

    Missing axis hint only records axis_angle_deg=None. Contact still
    follows GraspDecision.allowed from the fused bag budget.

    Args:
        result: _gated_result ok=True dict, updated in place.
        axis_hint: optional detection axis, normalized inside.
        config: gate parameters including max_axis_angle_deg.

    Returns
    -------
        The same result dict.

    """
    hint = _normalize_axis_hint(axis_hint)
    if hint is None:
        result['axis_angle_deg'] = None
        return result
    angle = axis_angle_deg(result['axis'], hint)
    result['axis_angle_deg'] = angle
    result['perception_axis'] = [float(v) for v in hint]
    max_deg = float(config.max_axis_angle_deg)
    if angle is not None and angle > max_deg:
        result['flags'].append('perception_reconstruction_axis_mismatch')
        result['diagnostic_axis_mismatch'] = True
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
            RefitResult dict.

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
        axis_point = np.asarray(est['q0'], dtype=np.float64)
        result = _gated_result(
            'cylinder', n, center, axis, axis_point, bottom, neck, span,
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
            RefitResult dict.

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
            'sphere', n, center, axis, center.copy(), bottom, neck, span,
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


def refine_geometry(xyz: np.ndarray, target_kind: str = 'bag',
                    config: Optional[RefitConfig] = None,
                    axis_hint=None) -> dict:
    """
    TSDF 云几何二次拟合门面：按 kind 分派到默认圆柱/球 refitter.

    输入为 finalize 的 tsdf_cloud（xyz，base_frame，米）。target_kind 除
    'fruit' 外一律按袋桃走圆柱线（未知/空值缺省袋桃，与感知包
    `target_kind or 'bag'` 语义一致）。拟合成功后按门控定 status：
    inlier_ratio ≥ config.cylinder_inlier_min 且 rmse ≤ config.rmse_max_m
    → ACCEPT，否则 REOBSERVE；拟合本身失败 → ok=False / REJECT。
    编排层按 REFITTERS_BY_IMPL 映射注入实现；本函数
    保留为模块级 workhorse 锚点（单测直接锚定）。

    Args:
        xyz: (N, 3) 点 [m]（base_frame）；空云/少点优雅失败不抛异常.
        target_kind: 'bag'/'fruit'（来自 /peach/perception/diagnostics）.
        config: 门控与行为参数；None 用 RefitConfig 默认.
        axis_hint: 可选 bottom→neck 单位方向（base_frame）。球体自身旋转
            对称，无法只靠球面恢复果梗方向；有效先验来自绑定目标的单帧
            果梗/凹陷估计。缺失时才退回 base +Z，并显式打诊断标记.

    Returns
    -------
        dict：ok/reason/kind/status/n_points/center/axis/axis_point/
        bottom/neck/radius/diameter/span_m/rmse/inlier_ratio/flags。
        几何量单位均 [m]；axis 为 bottom→neck 单位向量；入口/预抓取
        由 bag_model.fuse_bag_views 构造（refit 不出这两个点）.

    """
    refitters = {'cylinder': CylinderRefitter(), 'sphere': SphereRefitter()}
    return select_refitter(refitters, target_kind).refit(
        xyz, target_kind, config, axis_hint)


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
