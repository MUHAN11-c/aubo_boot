from __future__ import annotations
"""选果、控制面、摘要与账本（批次纯函数）。"""

import json
from pathlib import Path

from builtin_interfaces.msg import Duration
from peach_interfaces.msg import HarvestSummary, TargetOutcome
from peach_perception.common.runtime import default_runs_root


# === select.py ===

def next_target_id(observations, claimed, preferred=()):
    """
    先走 goal.target_ids，否则取已确认且未入账的第一项.

    裸果（strategy 含 fruit 或 unbagged_display_only）本轮不进入执行候选.
    observations 可为 None。未锁定集合不选.
    """
    claimed = set(claimed)
    for tid in preferred:
        if tid and tid not in claimed:
            return str(tid)
    if observations is None:
        return ''
    if not bool(getattr(observations, 'target_set_locked', False)):
        return ''
    for item in observations.observations:
        tid = getattr(item, 'target_id', '')
        if not tid or tid in claimed or not getattr(item, 'confirmed', False):
            continue
        flags = list(getattr(item, 'diagnostic_flags', []) or [])
        cand = getattr(item, 'candidate', None)
        strat = str(getattr(cand, 'strategy_id', '') or '')
        if 'unbagged_display_only' in flags or 'fruit' in strat:
            continue
        return str(tid)
    return ''


# === control.py ===

def apply_control(
        state_seq: int, expected: int, command: int, paused: bool,
        allowed=None):
    """
    纯函数控制面.

    expected==0 表示不校验。
    PAUSE=0 RESUME=1 ENTER_MAINTENANCE=2 EXIT_MAINTENANCE=3
    CANCEL_NOW=4 SKIP_TARGET=5 ACKNOWLEDGE_RECOVERY=6.
    allowed 为 permissions_for 结果；None 表示不按状态机过滤。
    返回 (accepted, new_seq, paused, cancel, skip).
    """
    if expected != 0 and expected != state_seq:
        return False, state_seq, paused, False, False
    if allowed is not None and command not in allowed:
        return False, state_seq, paused, False, False
    if command in (0, 2):
        return True, state_seq + 1, True, False, False
    if command in (1, 3):
        return True, state_seq + 1, False, False, False
    if command == 4:
        return True, state_seq + 1, paused, True, False
    if command == 5:
        return True, state_seq + 1, paused, False, True
    if command == 6:
        return True, state_seq + 1, paused, False, False
    return False, state_seq, paused, False, False


# === summary.py ===

def elapsed_msg(seconds: float) -> Duration:
    """单调时钟秒数 → Duration."""
    msg = Duration()
    safe = max(0.0, float(seconds))
    msg.sec = int(safe)
    msg.nanosec = int(round((safe - msg.sec) * 1e9))
    return msg


def build_summary(run_id: str, outcomes, discovered: int,
                  elapsed_s: float) -> HarvestSummary:
    """填充 HarvestSummary 计数与账本."""
    summary = HarvestSummary()
    summary.run_id = run_id
    summary.outcomes = list(outcomes)
    summary.discovered = int(max(discovered, len(summary.outcomes)))
    summary.attempted = len(summary.outcomes)
    summary.elapsed = elapsed_msg(elapsed_s)
    for item in summary.outcomes:
        code = int(item.outcome)
        if code == TargetOutcome.SUCCEEDED:
            summary.succeeded += 1
        elif code == TargetOutcome.SKIPPED_QUALITY:
            summary.skipped_quality += 1
        elif code == TargetOutcome.SKIPPED_UNREACHABLE:
            summary.skipped_unreachable += 1
        elif code == TargetOutcome.FAILED:
            summary.failed += 1
        elif code == TargetOutcome.CANCELED:
            summary.canceled += 1
    return summary


# === ledger.py ===

def default_ledger_root() -> Path:
    """账本根目录，与观测/session 同为工作区 ``runs/``."""
    return default_runs_root()


def ledger_file(root: Path, request_id: str) -> Path:
    """单个批次的 ledger.json 路径."""
    safe = request_id.strip() or 'harvest'
    return Path(root) / safe / 'ledger.json'


def target_artifact_dir(root: Path, request_id: str, target_id: str) -> Path:
    """runs/<request_id>/targets/<target_id>/ 过程目录."""
    safe_run = request_id.strip() or 'harvest'
    safe_tid = target_id.strip() or 'target'
    return Path(root) / safe_run / 'targets' / safe_tid


def elapsed_s(outcome) -> float | None:
    """TargetOutcome.elapsed → 秒；零/缺省给 None."""
    elapsed = getattr(outcome, 'elapsed', None)
    if elapsed is None:
        return None
    value = (float(getattr(elapsed, 'sec', 0) or 0)
             + float(getattr(elapsed, 'nanosec', 0) or 0) * 1e-9)
    return round(value, 3) if value > 0.0 else None


def set_elapsed(outcome, seconds: float) -> None:
    """把墙钟秒写入 TargetOutcome.elapsed."""
    value = max(float(seconds or 0.0), 0.0)
    outcome.elapsed.sec = int(value)
    outcome.elapsed.nanosec = int(round((value - int(value)) * 1e9))


def outcome_to_dict(outcome: TargetOutcome, extra: dict | None = None) -> dict:
    """Serialize TargetOutcome plus optional telemetry extra."""
    data = {
        'target_id': str(outcome.target_id),
        'outcome': int(outcome.outcome),
        'reason': str(outcome.reason),
        'quality_score': float(getattr(outcome, 'quality_score', 0.0) or 0.0),
        'elapsed_s': elapsed_s(outcome),
    }
    extra = extra or {}
    for key in (
            'failure_code', 'stage_names', 'stage_durations',
            'build_view_count', 'build_status', 'build_duration_s',
            'timeout_source'):
        if extra.get(key) is not None:
            data[key] = extra[key]
    return data


def dict_to_outcome(data: dict) -> TargetOutcome:
    """Build TargetOutcome from a ledger dict."""
    item = TargetOutcome()
    item.target_id = str(data.get('target_id', ''))
    item.outcome = int(data.get('outcome', TargetOutcome.FAILED))
    item.reason = str(data.get('reason', ''))
    item.quality_score = float(data.get('quality_score', 0.0) or 0.0)
    if data.get('elapsed_s') is not None:
        set_elapsed(item, float(data['elapsed_s']))
    return item


def load_ledger(path: Path) -> tuple:
    """Return claimed IDs and outcomes; missing file yields empty."""
    if not path.is_file():
        return set(), []
    raw = json.loads(path.read_text(encoding='utf-8'))
    claimed = {str(tid) for tid in raw.get('claimed', []) if tid}
    outcomes = [dict_to_outcome(item) for item in raw.get('outcomes', [])]
    for item in outcomes:
        if item.target_id:
            claimed.add(item.target_id)
    return claimed, outcomes


def save_ledger(path: Path, claimed, outcomes, details=None) -> None:
    """原子写 ledger.json（details 与 outcomes 等长的遥测附加字段）."""
    path.parent.mkdir(parents=True, exist_ok=True)
    extras = list(details or [])
    rows = []
    for index, item in enumerate(outcomes):
        extra = extras[index] if index < len(extras) else {}
        rows.append(outcome_to_dict(item, extra))
    document = {
        'claimed': sorted(str(tid) for tid in claimed if tid),
        'outcomes': rows,
    }
    tmp = path.with_suffix('.json.tmp')
    tmp.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    tmp.replace(path)
