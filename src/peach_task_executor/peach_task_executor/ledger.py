"""批次账本落盘：同 request_id 重启后续跑未入账目标."""

from __future__ import annotations

import json
from pathlib import Path

from peach_interfaces.msg import TargetOutcome
from peach_perception.common.harvest_data import default_runs_root


def default_ledger_root() -> Path:
    """账本根目录，与观测/session 同为工作区 ``runs/``."""
    return default_runs_root()


def ledger_file(root: Path, request_id: str) -> Path:
    """单个批次的 ledger.json 路径."""
    safe = request_id.strip() or 'harvest'
    return Path(root) / safe / 'ledger.json'


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
