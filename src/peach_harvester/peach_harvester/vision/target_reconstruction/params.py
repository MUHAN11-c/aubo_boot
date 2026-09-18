"""
TargetReconstructionParams: yaml 直读，frames / session 字符串 strip.

部署事实源 ``config/target_reconstruction.yaml``。主节点一行
``TargetReconstructionParams.attach(node)``：声明叶子并挂规则校验（越界值
启动期拒绝、运行期非法 set 即拒），``ros2 param set`` 原地刷新。构造期
捕获进 TSDF/ICP/采集门的键改后须重启（on_configure 重建）。
"""
from __future__ import annotations

from peach_harvester.vision.param_rules import check as _check

_RULES = {  # 键 -> 校验规则表（启动期非法即拒启；运行期非法 set 即拒）
    'sync_slop_s': (('gt_eq', 0.0),),
    'tf_timeout_sec': (('gt_eq', 0.0),),
    'depth_scale_unit': (('gt', 0.0),),
    'capture.min_views': (('gt_eq', 1),),
    'capture.recommended_views': (('gt_eq', 1),),
    'capture.max_views': (('gt_eq', 1),),
    'capture.minimum_baseline_deg': (('gt_eq', 0.0),),
    'capture.minimum_mean_nearest_baseline_deg': (('gt_eq', 0.0),),
    'capture.minimum_mean_depth_ratio': (('bounds', 0.0, 1.0),),
    'capture.static_joint_vel_thresh': (('gt_eq', 0.0),),
    'capture.max_frame_age_s': (('gt', 0.0),),
    'capture.auto_min_interval_s': (('gt_eq', 0.0),),
    'capture.min_mask_pixels': (('gt_eq', 1),),
    'capture.min_mask_depth_ratio': (('bounds', 0.0, 1.0),),
    'capture.max_target_drift_m': (('gt_eq', 0.0),),
    'capture.neighbor_gap_area_ratio': (('gt_eq', 0.0),),
    'capture.build_timeout_s': (('gt', 0.0),),
    'bind.switch_holdoff_s': (('gt_eq', 0.0),),
    'view_filter.min_translation': (('gt_eq', 0.0),),
    'view_filter.min_rotation_deg': (('gt_eq', 0.0),),
    'icp.min_points': (('gt_eq', 1),),
    'icp.coarse_voxel': (('gt', 0.0),),
    'icp.fine_voxel': (('gt', 0.0),),
    'icp.coarse_correspondence': (('gt', 0.0),),
    'icp.fine_correspondence': (('gt', 0.0),),
    'icp.coarse_iterations': (('gt_eq', 1),),
    'icp.fine_iterations': (('gt_eq', 1),),
    'icp.min_fitness': (('bounds', 0.0, 1.0),),
    'icp.max_rmse': (('gt', 0.0),),
    'icp.max_translation': (('gt_eq', 0.0),),
    'icp.max_rotation_deg': (('gt_eq', 0.0),),
    'icp.target_refresh_min_period': (('gt_eq', 1),),
    'icp.target_refresh_max_period': (('gt_eq', 1),),
    'icp.target_refresh_drift_ratio': (('gt', 0.0),),
    'local_volume.size_x': (('gt', 0.0),),
    'local_volume.size_y': (('gt', 0.0),),
    'local_volume.size_z': (('gt', 0.0),),
    'tsdf.voxel_length': (('gt', 0.0),),
    'tsdf.sdf_trunc': (('gt', 0.0),),
    'tsdf.depth_trunc': (('gt', 0.0),),
    'cloud_filter.voxel_size': (('gt_eq', 0.0),),
    'refit.cylinder_inlier_min': (('bounds', 0.0, 1.0),),
    'refit.rmse_max_m': (('gt', 0.0),),
    'refit.entry_standoff_m': (('gt_eq', 0.0),),
    'refit.pregrasp_standoff_m': (('gt_eq', 0.0),),
    'refit.max_axis_angle_deg': (('gt', 0.0),),
    'publish.min_interval_s': (('gt_eq', 0.0),),
    'tool.budget.d_inner': (('bounds', 0.01, 0.5),),
}


def _validate(name, value):
    """逐条规则校验；返回拒绝理由或 None."""
    for rule in _RULES.get(name, ()):
        why = _check(rule, value, name)
        if why:
            return why
    return None


class _StripProxy:
    """Forward a nested group; listed string fields are stripped on read."""

    def __init__(self, raw, names: tuple):
        """Raw nested group; names are whitespace-trimmed on access."""
        object.__setattr__(self, '_raw', raw)
        object.__setattr__(self, '_names', names)

    def __getattr__(self, name):
        """Strip listed fields; forward the rest."""
        value = getattr(self._raw, name)
        if name in self._names:
            return str(value).strip()
        return value


class TargetReconstructionParams:
    """Live yaml snapshot; frames.base_frame and session.root_dir strip."""

    def __init__(self, raw):
        """Wrap the yaml namespace with strip proxies."""
        object.__setattr__(self, '_raw', raw)
        object.__setattr__(
            self, 'frames',
            _StripProxy(raw.frames, ('base_frame',)))
        object.__setattr__(
            self, 'session',
            _StripProxy(raw.session, ('root_dir',)))

    def __getattr__(self, name):
        """Forward undeclared names to the yaml namespace."""
        return getattr(self._raw, name)

    @classmethod
    def attach(cls, node) -> 'TargetReconstructionParams':
        """Declare yaml leaves with range checks; return a live wrapper."""
        from peach_harvester.yaml_params import attach, package_yaml
        raw = attach(
            node,
            package_yaml('peach_harvester', 'target_reconstruction.yaml'),
            validate=_validate)
        return cls(raw)

    @classmethod
    def from_params(cls, snapshot) -> 'TargetReconstructionParams':
        """Pure-core wrapper for tests (no ROS node)."""
        return cls(snapshot)
