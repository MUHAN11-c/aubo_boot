"""
SceneObstacles: yaml 直读 + 一行 attach（决策 0024 模式）.

部署事实源 ``config/scene_obstacles.yaml``。节点
``SceneObstaclesParams.attach(node)`` 声明叶子并挂校验：启动期非法即拒启。
本节点为无状态快照服务（不进 lifecycle 名单），运行期改参按需重启生效。
"""
from __future__ import annotations

from dataclasses import dataclass

from peach_harvester.vision.param_rules import check as _check
from peach_harvester.yaml_params import package_yaml


_RULES = {  # 键 -> 校验规则表（启动期非法即拒启）
    'object_id': (('nonempty',),),
    'base_frame': (('nonempty',),),
    'voxel_size_m': (('gt', 0.0),),
    'self_filter_margin_m': (('gt_eq', 0.0),),
    'capsule_radial_margin_m': (('gt_eq', 0.0),),
    'capsule_axial_margin_m': (('gt_eq', 0.0),),
    'workspace_radius_m': (('gt', 0.0),),
    'max_boxes': (('gt_eq', 1),),
    'cloud_max_age_s': (('gt', 0.0),),
    'apply_service_timeout_s': (('gt', 0.0),),
}


def _validate(name, value):
    for rule in _RULES.get(name, ()):
        why = _check(rule, value, name)
        if why:
            return why
    return None


@dataclass(frozen=True)
class SnapshotTuning:
    """core.SnapshotParams 的 yaml 侧来源（纯核可测，不 import ROS）."""

    voxel_size_m: float
    self_filter_margin_m: float
    capsule_radial_margin_m: float
    capsule_axial_margin_m: float
    workspace_radius_m: float
    max_boxes: int


class SceneObstaclesParams:
    """
    scene_obstacles.yaml 叶子（attach 返回的实时命名空间的轻包装）.

    叶子属性经 __getattr__ 代理（yaml_params.attach 返回 SimpleNamespace
    实时命名空间，非本类）；tuning() 把纯核参数提为 SnapshotTuning。
    """

    def __init__(self, namespace) -> None:
        object.__setattr__(self, '_namespace', namespace)

    def __getattr__(self, name: str):
        """叶子属性代理到底层 yaml 实时命名空间."""
        return getattr(self._namespace, name)

    @staticmethod
    def attach(node) -> 'SceneObstaclesParams':
        """Declare yaml leaves with range checks; live namespace."""
        from peach_harvester.yaml_params import attach as _attach

        namespace = _attach(
            node,
            package_yaml('peach_harvester', 'scene_obstacles.yaml'),
            validate=_validate)
        return SceneObstaclesParams(namespace)

    def tuning(self) -> SnapshotTuning:
        """纯核参数快照（build_snapshot 入参）."""
        namespace = self._namespace
        return SnapshotTuning(
            voxel_size_m=namespace.voxel_size_m,
            self_filter_margin_m=namespace.self_filter_margin_m,
            capsule_radial_margin_m=namespace.capsule_radial_margin_m,
            capsule_axial_margin_m=namespace.capsule_axial_margin_m,
            workspace_radius_m=namespace.workspace_radius_m,
            max_boxes=namespace.max_boxes)
