"""
Supervisor / lifecycle: yaml 直读 + 一行 attach.

部署事实源 ``config/{peach_supervisor,lifecycle_manager}.yaml``。
节点 ``Xxx.attach(node)`` 声明叶子并挂校验：启动期非法覆盖即抛（拒绝启动）；
``ros2 param set`` 原地刷新，整批合法才提交（跨字段选果窗经 preview 拒绝）。
observability 的参数/规则随 ``peach_observability/params.py`` 自持（W11 起
yaml 迁 ``peach_observability/config``，本包不再代持）。
"""
from __future__ import annotations

from peach_harvester.supervisor.param_rules import check as _check, check_min_max
from peach_harvester.yaml_params import attach, package_yaml


def _yaml(filename: str):
    """Share-tree config path for this package."""
    return package_yaml('peach_harvester', filename)


_SUPERVISOR_RULES = {  # 键 -> 校验规则表（启动期非法即拒启；运行期非法 set 即拒）
    'survey_wait_s': (('gt_eq', 0.0),),
    'survey_dwell_s': (('gt_eq', 0.0),),
    'service_timeout_s': (('gt', 0.0),),
    'action_timeout_s': (('gt', 0.0),),
    'empty_survey_limit': (('gt_eq', 1),),
    'reconstruction_min_views': (('gt_eq', 1),),
    'build_start_timeout_s': (('gt', 0.0),),
    'observe_build_grace_s': (('gt_eq', 0.0),),
}

_LIFECYCLE_RULES = {
    'startup_timeout_s': (('gt', 0.0),),
}


def _validator(rules: dict):
    """规则表 -> validate 回调（返回拒绝理由或 None）."""
    def _validate(name, value):
        for rule in rules.get(name, ()):
            why = _check(rule, value, name)
            if why:
                return why
        return None
    return _validate


def _supervisor_preview(trial):
    """跨字段选果窗 min < max：非法整批拒绝，不改当前一致快照."""
    for low, high in (
            ('selection_reach_min_m', 'selection_reach_max_m'),
            ('selection_depth_min_m', 'selection_depth_max_m')):
        why = check_min_max(getattr(trial, low), getattr(trial, high), low, high)
        if why:
            raise ValueError(why)


class SupervisorParams:
    """peach_supervisor.yaml 叶子（attach 返回的实时命名空间）."""

    execution_enabled: bool
    """False 时 WAIT_LOCK 后直接结算，不选果、不派 ExecuteTarget."""
    survey_wait_s: float
    """BeginScene 后等待锁定集关闭 [s]."""
    survey_dwell_s: float
    """已锁回访到位后驻留 [s]，让几何来自拍照位."""
    service_timeout_s: float
    """BeginScene 等服务等待上限 [s]."""
    action_timeout_s: float
    """Survey/Build/Execute 动作等待上限 [s]."""
    empty_survey_limit: int
    """连续空扫次数上限，达到则结算."""
    persist_ledger: bool
    """是否把 TargetOutcome 写入 runs/ 账本."""
    reconstruction_min_views: int
    """OBSERVE 成功后 Build 至少机位数."""
    skip_reconstruction: bool
    reconstruct_in_trajectory: bool
    """True 时 DISPATCH 跳过 Build/观察，用场景观测几何进接触."""
    build_start_timeout_s: float
    """Build 进入 COLLECTING 的等待上限 [s]."""
    observe_build_grace_s: float
    """OBSERVE 结束后等机位数追上的宽限 [s]."""
    execute_pregrasp_only: bool
    """True=接触段只到预抓取验证；False=FULL."""
    selection_reach_min_m: float
    """选果可达窗下限（base_link 半径）[m]."""
    selection_reach_max_m: float
    """可达性回退窗上限 [m]；IK 预检在线时不参与."""
    selection_depth_min_m: float
    """选果深度窗下限（相机到目标）[m]."""
    selection_depth_max_m: float
    """选果深度窗上限 [m]."""
    require_managed_stack: bool
    """True 时须等 managed_nodes_activated 才接受 RunHarvest."""
    begin_scene_service: str
    """BeginScene 服务名."""
    survey_scene_action: str
    """SurveyScene 动作名."""
    execute_target_action: str
    """ExecuteTarget 动作名."""
    build_target_model_action: str
    """BuildTargetModel 动作名."""
    check_reachability_service: str
    """TCP IK 可达性预检服务名."""


class peach_supervisor:
    """peach_supervisor parameters from peach_supervisor.yaml."""

    @staticmethod
    def attach(node) -> SupervisorParams:
        """Declare yaml leaves with range/window checks; live namespace."""
        return attach(
            node,
            _yaml('peach_supervisor.yaml'),
            validate=_validator(_SUPERVISOR_RULES),
            preview=_supervisor_preview)


class peach_lifecycle_manager:
    """peach_lifecycle_manager parameters from lifecycle_manager.yaml."""

    @staticmethod
    def attach(node):
        """Declare yaml leaves; live namespace."""
        return attach(
            node, _yaml('lifecycle_manager.yaml'),
            validate=_validator(_LIFECYCLE_RULES))
