from __future__ import annotations
"""监控状态缓存、作业票推导、性能采样。"""

import math
from typing import Any


# 作业环节顺序即监控页过程线。id 与 web data-stage 对齐。
STAGE_SPECS = (
    ('discover', '发现'),
    ('photo', '拍照'),
    ('lock', '锁定'),
    ('observe', '观察'),
    ('permit', '许可'),
    ('approach', '靠近'),
    ('tool', '工具'),
    ('retreat', '撤离'),
    ('done', '完成'),
)
STAGE_IDS = tuple(item[0] for item in STAGE_SPECS)

# HarvestState.batch_state / target_phase 与消息常量一致。
_BATCH_COMPLETED = 6
_BATCH_RECOVERY = 7
_BATCH_INTERRUPTED = 8
_PHASE_SELECTING = 1
_PHASE_OBSERVING = 2
_PHASE_FINALIZING = 3
_PHASE_VALIDATING = 4
_PHASE_APPROACHING = 5
_PHASE_TOOL = 6
_PHASE_RETREATING = 7
_PHASE_COMPLETING = 8
_PHASE_SUCCEEDED = 9
_PHASE_SKIPPED = 10
_PHASE_FAILED = 11

_SKILL_OBSERVE = {
    'PLAN_OBSERVATION', 'MOVE_TO_VIEW', 'WAIT_FRAME',
}
_SKILL_PERMIT = {'FINALIZE', 'RECONFIRM', 'READY_FOR_GRASP'}
_SKILL_APPROACH = {'MTC_APPROACH_INSERT', 'PREVIEW_CONTACT_PLANNING'}
_SKILL_NO_CONTACT = {
    'READY_FOR_GRASP', 'PLAN_READY', 'PREVIEW_READY',
}


def build_harvest_job(snapshot: dict) -> dict:
    """
    把调度/感知/重建/技能最新值折成一颗果实的作业票.

    Args:
        snapshot: ObservabilityState.snapshot() 尚未写入 job 键的浅拷贝。

    Returns
    -------
        含 stages/why/coords/grasp/motion/flags 的 dict；无目标时 stages
        仍给出过程线（多半 pending）。

    """
    executor = (snapshot.get('task_executor') or {}).get('state') or {}
    perception = snapshot.get('perception') or {}
    targets = perception.get('targets') or {}
    harvest = perception.get('harvest') or {}
    recon = snapshot.get('reconstruction') or {}
    diag = recon.get('diagnostics') or {}
    decision = recon.get('grasp_decision') or {}
    refined = snapshot.get('refined') or {}
    manipulation = (snapshot.get('manipulation') or {}).get('status') or {}
    hypothesis = (snapshot.get('manipulation') or {}).get('hypothesis') or {}

    batch = _int(executor.get('batch_state'), -1)
    phase = _int(executor.get('target_phase'), 0)
    target_id = str(
        executor.get('target_id')
        or targets.get('selected_target_id')
        or harvest.get('selected_target_id')
        or '')
    obs = _observation(targets, target_id)
    if not target_id and obs:
        target_id = str(obs.get('target_id') or '')

    flags = {
        'execution_enabled': bool(executor.get('execution_enabled')),
        'grasp_enabled': bool(executor.get('grasp_enabled')),
        'tool_enabled': bool(executor.get('tool_enabled')),
        'skill_grasp_enabled': manipulation.get('grasp_enabled'),
        'skill_execution_enabled': manipulation.get('execution_enabled'),
    }
    grasp_on = bool(flags['grasp_enabled'])
    tool_on = bool(flags['tool_enabled'])
    skill_state = str(manipulation.get('state') or '')
    message = str(executor.get('message') or '')
    locked = bool(targets.get('target_set_locked') or harvest.get(
        'target_set_locked'))
    allowed = bool(decision.get('allowed'))
    recon_state = str(diag.get('state') or '')
    recovery = bool(executor.get('recovery_required')) or batch == _BATCH_RECOVERY
    if recovery and batch != _BATCH_INTERRUPTED:
        batch = _BATCH_RECOVERY

    active, fail_at = _active_stage(
        batch, phase, message, skill_state, locked,
        str(manipulation.get('message') or ''))
    statuses = _fill_statuses(active, fail_at, phase, batch)
    _apply_gates(statuses, grasp_on, tool_on, skill_state, phase)
    if not _contact_reached(phase, skill_state, grasp_on):
        for stage_id in ('approach', 'tool'):
            if statuses.get(stage_id) == 'done':
                statuses[stage_id] = 'gated' if not grasp_on else 'skipped'
        if statuses.get('retreat') == 'done' and phase != _PHASE_SUCCEEDED:
            statuses['retreat'] = 'skipped'

    why = _why(
        batch, phase, message, skill_state, grasp_on, tool_on,
        allowed, decision.get('reason') or '', recon_state,
        manipulation.get('message') or '', fail_at)

    stages = []
    for stage_id, label in STAGE_SPECS:
        status = statuses.get(stage_id, 'pending')
        stages.append({
            'id': stage_id,
            'label': label,
            'status': status,
            'detail': _stage_detail(
                stage_id, status, allowed, decision, recon_state,
                skill_state, grasp_on, tool_on),
        })

    axis_xyz = _xyz(decision.get('axis'))
    has_geom = (
        axis_xyz is not None and
        axis_xyz[0] * axis_xyz[0] + axis_xyz[1] * axis_xyz[1] +
        axis_xyz[2] * axis_xyz[2] > 0.25)
    grasp = {
        'allowed': allowed,
        'reason': str(decision.get('reason') or ''),
        'target_id': str(decision.get('target_id') or ''),
        'entry': _xyz(decision.get('entry')) if (allowed or has_geom) else None,
        'pregrasp': _xyz(decision.get('pregrasp')) if (
            allowed or has_geom) else None,
        'axis': _xyz(decision.get('axis')) if (allowed or has_geom) else None,
        'cut_pose': _xyz(decision.get('cut_pose')) if (
            allowed or has_geom) else None,
        'diameter_m': decision.get('diameter_m') if (
            allowed or has_geom) else None,
        'rmse_m': decision.get('rmse_m') if (allowed or has_geom) else None,
        'inlier_ratio': decision.get('inlier_ratio') if (
            allowed or has_geom) else None,
    }
    candidate = (obs or {}).get('candidate') or {}
    coords = {
        'frame_id': str(
            targets.get('frame_id') or hypothesis.get('frame_id')
            or 'base_link'),
        'perception_entry': _xyz(candidate.get('entry_position')),
        'perception_bottom': _xyz(candidate.get('bag_bottom')),
        'perception_neck': _xyz(candidate.get('bag_neck')),
        'reconstruction_center': _xyz(diag.get('target_center_base')),
        'refined_axis': _xyz((refined.get('axis') or {}).get('xyz')),
        'grasp_entry': grasp['entry'],
        'grasp_pregrasp': _xyz(decision.get('pregrasp')) if (
            allowed or has_geom) else None,
        'hypothesis_entry': _xyz(hypothesis.get('entry_position')),
        'hypothesis_travel_m': hypothesis.get('travel_m'),
        'camera_distance_m': (obs or {}).get('camera_distance_m'),
    }
    quality = manipulation.get('quality') or {}
    coverage = diag.get('view_coverage') or {}
    return {
        'target_id': target_id,
        'cycle_id': str(executor.get('cycle_id') or ''),
        'batch_state': batch,
        'target_phase': phase,
        'active_id': active,
        'why': why,
        'flags': flags,
        'stages': stages,
        'coords': coords,
        'grasp': grasp,
        'motion': {
            'state': skill_state,
            'message': str(manipulation.get('message') or ''),
            'running': manipulation.get('running'),
        },
        'reconstruction': {
            'state': recon_state,
            'captured_views': diag.get('captured_views'),
            'rejected_views': diag.get('rejected_views'),
            'max_baseline_deg': coverage.get('max_baseline_deg')
            or quality.get('max_baseline_deg'),
            'mean_nearest_baseline_deg': coverage.get(
                'mean_nearest_baseline_deg'),
        },
        'perception': {
            'locked': locked,
            'target_count': targets.get('target_count')
            or harvest.get('target_count'),
            'harvest_status': (obs or {}).get('harvest_status'),
            'tracking_status': (obs or {}).get('tracking_status'),
            'confidence': (obs or {}).get('confidence'),
        },
    }


def _int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _xyz(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        out = [float(value[0]), float(value[1]), float(value[2])]
    except (TypeError, ValueError):
        return None
    if any(not math.isfinite(item) for item in out):
        return None
    if out == [-1.0, -1.0, -1.0]:
        return None
    return out


def _observation(targets: dict, target_id: str) -> dict | None:
    items = targets.get('observations') or []
    if target_id:
        for item in items:
            if str(item.get('target_id') or '') == target_id:
                return item
    for item in items:
        if item.get('selected'):
            return item
    selected = str(targets.get('selected_target_id') or '')
    if selected:
        for item in items:
            if str(item.get('target_id') or '') == selected:
                return item
    return items[0] if items else None


def _active_stage(batch: int, phase: int, message: str, skill: str,
                  locked: bool, skill_message: str = '') -> tuple[str, str | None]:
    """返回 (当前环节, 失败环节或 None)."""
    blob = f'{message} {skill_message} {skill}'
    if batch == _BATCH_COMPLETED:
        if skill == 'FAILED' or any(token in blob.lower() for token in (
                'observe_failed', '扫描上限', '未收敛')):
            failed = _fail_stage(blob, skill, phase)
            return failed, failed
        return 'done', None
    if batch == _BATCH_RECOVERY:
        return 'approach', None
    if batch == _BATCH_INTERRUPTED:
        return 'done', 'done'
    if phase == _PHASE_FAILED:
        return _fail_stage(message, skill, phase), _fail_stage(
            message, skill, phase)
    if phase == _PHASE_SKIPPED:
        failed = _fail_stage(message, skill, phase)
        return failed, failed
    if phase == _PHASE_SUCCEEDED:
        return 'done', None
    if skill in _SKILL_APPROACH or skill == 'ACTUATE_TOOL':
        return ('approach' if skill in _SKILL_APPROACH else 'tool'), None
    if skill in _SKILL_OBSERVE:
        return 'observe', None
    if skill in _SKILL_PERMIT:
        return 'permit', None
    if skill == 'MTC_RETREAT':
        return 'retreat', None
    if batch <= 0:
        return 'discover', None
    if batch == 1:
        if '拍照' in message:
            return 'photo', None
        if locked:
            return 'lock', None
        return 'discover', None
    if phase <= 0:
        return 'lock' if locked else 'discover', None
    if phase == _PHASE_SELECTING:
        return 'lock', None
    if phase == _PHASE_OBSERVING:
        return 'observe', None
    if phase in (_PHASE_FINALIZING, _PHASE_VALIDATING):
        return 'permit', None
    if phase == _PHASE_APPROACHING:
        return 'approach', None
    if phase == _PHASE_TOOL:
        return 'tool', None
    if phase in (_PHASE_RETREATING, _PHASE_COMPLETING):
        return 'retreat', None
    return 'observe', None


def _fail_stage(message: str, skill: str, phase: int) -> str:
    text = f'{message} {skill}'.lower()
    if any(token in text for token in (
            'observe', '扫描', '视点', '重建', 'build_target', 'stale')):
        return 'observe'
    if any(token in text for token in (
            '许可', 'quality', 'grasp_decision', 'axis')):
        return 'permit'
    if any(token in text for token in (
            'approach', 'mtc', 'unreachable', '靠近')):
        return 'approach'
    if 'tool' in text or '工具' in message:
        return 'tool'
    if phase >= _PHASE_TOOL:
        return 'tool'
    if phase >= _PHASE_APPROACHING:
        return 'approach'
    if phase >= _PHASE_FINALIZING:
        return 'permit'
    return 'observe'


def _fill_statuses(active: str, fail_at: str | None, phase: int,
                   batch: int) -> dict[str, str]:
    statuses = {stage_id: 'pending' for stage_id in STAGE_IDS}
    try:
        active_index = STAGE_IDS.index(active)
    except ValueError:
        active_index = 0
    for index, stage_id in enumerate(STAGE_IDS):
        if index < active_index:
            statuses[stage_id] = 'done'
        elif index == active_index:
            statuses[stage_id] = 'active'
        else:
            statuses[stage_id] = 'pending'
    if fail_at:
        try:
            fail_index = STAGE_IDS.index(fail_at)
        except ValueError:
            fail_index = active_index
        for index, stage_id in enumerate(STAGE_IDS):
            if index < fail_index:
                statuses[stage_id] = 'done'
            elif index == fail_index:
                statuses[stage_id] = 'failed'
            else:
                statuses[stage_id] = 'skipped'
    if phase == _PHASE_SUCCEEDED and batch != _BATCH_RECOVERY:
        for stage_id in STAGE_IDS:
            statuses[stage_id] = 'done'
    elif batch == _BATCH_COMPLETED:
        statuses['done'] = 'done'
    return statuses


def _contact_reached(phase: int, skill: str, grasp_on: bool) -> bool:
    if skill in _SKILL_APPROACH or skill == 'ACTUATE_TOOL':
        return True
    if phase in (_PHASE_APPROACHING, _PHASE_TOOL, _PHASE_RETREATING):
        return True
    return phase == _PHASE_SUCCEEDED and grasp_on


def _apply_gates(statuses: dict[str, str], grasp_on: bool, tool_on: bool,
                 skill: str, phase: int) -> None:
    """抓取/工具档关闭时，未真正接触的环节标 gated，禁止显示成已勾上."""
    contact_ran = (
        skill in _SKILL_APPROACH
        or skill == 'ACTUATE_TOOL'
        or phase in (_PHASE_APPROACHING, _PHASE_TOOL)
        or (phase == _PHASE_SUCCEEDED and grasp_on))
    if not grasp_on and not contact_ran:
        for stage_id in ('approach', 'tool'):
            if statuses.get(stage_id) in ('done', 'active', 'pending'):
                statuses[stage_id] = 'gated'
        if skill in _SKILL_NO_CONTACT or phase == _PHASE_COMPLETING:
            if statuses.get('retreat') == 'done':
                statuses['retreat'] = 'skipped'
    elif grasp_on and not tool_on:
        tool_ran = skill == 'ACTUATE_TOOL' or phase == _PHASE_TOOL
        if not tool_ran and statuses.get('tool') in (
                'done', 'active', 'pending'):
            statuses['tool'] = 'gated'


def _why(batch: int, phase: int, message: str, skill: str, grasp_on: bool,
         tool_on: bool, allowed: bool, reason: str, recon_state: str,
         skill_message: str, fail_at: str | None) -> str:
    if batch == _BATCH_RECOVERY:
        blob = f'{message} {skill_message}'
        if '预抓取' in blob or 'PREGRASP' in blob:
            return '停在预抓取，ACK 后再 Survey'
        return '接触恢复未确认，不派下一颗'
    if batch == _BATCH_INTERRUPTED:
        return message or '批次已中断'
    if fail_at == 'observe':
        return skill_message or message or '观察/建模未收敛，未进入抓取'
    if fail_at == 'permit':
        return reason or skill_message or '抓取许可未过'
    if fail_at:
        return skill_message or message or '本颗未完成抓取'
    if phase == _PHASE_SUCCEEDED:
        if not grasp_on:
            return '本颗观察完成；抓取档关闭，未接触'
        return message or '本颗完成'
    if not grasp_on and phase >= _PHASE_OBSERVING:
        if allowed:
            return '抓取已许可，但抓取档关闭：可靠近规划，本轮不接触'
        return '抓取档关闭：过程线停在观察/许可，不接触'
    if grasp_on and not tool_on and phase >= _PHASE_APPROACHING:
        return '工具档关闭：可靠近，不打末端 IO'
    if skill == 'FAILED':
        return skill_message or message
    if not allowed:
        if recon_state in ('COLLECTING', 'REFINING'):
            return f'重建{recon_state}，尚未给出抓取几何'
        if reason:
            return f'接触未许可（{reason}）；有几何则可去预抓取'
        if phase <= _PHASE_OBSERVING:
            return '仍在观察/建模，尚未进入靠近'
    if allowed and phase < _PHASE_APPROACHING:
        return '抓取已许可，等待进入靠近'
    return skill_message or message or '等待调度状态'


def _stage_detail(stage_id: str, status: str, allowed: bool, decision: dict,
                  recon_state: str, skill: str, grasp_on: bool,
                  tool_on: bool) -> str:
    if status == 'gated':
        if stage_id == 'approach':
            return '抓取档关闭，本轮不接触'
        if stage_id == 'tool':
            return '工具档关闭，不打 SetIO'
        return '本环节未使能'
    if stage_id == 'permit':
        if allowed:
            return 'GraspDecision.allowed'
        return str(decision.get('reason') or recon_state or '')
    if stage_id == 'observe' and skill:
        return skill
    if status == 'failed':
        return '本环节失败，后续未开始'
    return ''
