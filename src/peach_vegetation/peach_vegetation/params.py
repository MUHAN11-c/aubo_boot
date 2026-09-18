"""
peach_vegetation: yaml 直读 + 一行 attach.

部署事实源 ``config/vegetation.yaml``。节点 ``peach_vegetation.attach(node)``：
声明叶子并挂规则校验（device 白名单、HSV 窗、Frangi 尺度表；启动期非法即
拒启、运行期非法 set 即拒），``ros2 param set`` 原地刷新；leaf.h_min <
leaf.h_max 跨字段经 preview 整批拒绝。掩膜阈值热生效；device / sigmas 改后
须重启（on_configure 重建 splitter）。
"""
from __future__ import annotations

from peach_vegetation.param_rules import check as _check, check_min_max
from peach_vegetation.yaml_params import attach, package_yaml

_RULES = {  # 键 -> 校验规则表（启动期非法即拒启；运行期非法 set 即拒）
    'device': (('one_of', ('auto', 'cpu', 'cuda', 'cuda:0')),),
    'leaf.exg_min': (('bounds', -255.0, 510.0),),
    'leaf.h_min': (('bounds', 0, 179),),
    'leaf.h_max': (('bounds', 0, 179),),
    'leaf.s_min': (('bounds', 0, 255),),
    'leaf.v_min': (('bounds', 0, 255),),
    'branch.sigmas': (('seq_gt', 0.0),),
    'branch.beta': (('gt', 0.0),),
    'branch.gamma': (('gt', 0.0),),
    'branch.percentile': (('bounds', 0.0, 100.0),),
    'branch.dilate_px': (('gt_eq', 0),),
    'branch.dark_max': (('bounds', 0.0, 255.0),),
}


def _validate(name, value):
    """逐条规则校验；返回拒绝理由或 None."""
    for rule in _RULES.get(name, ()):
        why = _check(rule, value, name)
        if why:
            return why
    return None


def _preview(trial):
    """跨字段 HSV 绿窗 h_min < h_max：非法整批拒绝."""
    why = check_min_max(
        trial.leaf.h_min, trial.leaf.h_max, 'leaf.h_min', 'leaf.h_max')
    if why:
        raise ValueError(why)


class VegetationParams:
    """vegetation.yaml 叶子（attach 返回的实时命名空间）."""

    device: str
    """'auto' | 'cpu' | 'cuda' | 'cuda:0'；改后须重启 splitter."""


class peach_vegetation:
    """peach_vegetation parameters from vegetation.yaml."""

    @staticmethod
    def attach(node) -> VegetationParams:
        """Declare yaml leaves with device/HSV/Frangi checks; live namespace."""
        return attach(
            node,
            package_yaml('peach_vegetation', 'vegetation.yaml'),
            validate=_validate,
            preview=_preview)
