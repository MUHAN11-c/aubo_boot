"""批次账本落盘：同 request_id 重启后续跑未入账目标."""

from __future__ import annotations

import json
import os
from pathlib import Path

from peach_interfaces.msg import TargetOutcome


def default_ledger_root() -> Path:
    """账本根目录，可由 AUBO_HARVEST_DATA_DIR 覆盖."""
    override = os.environ.get('AUBO_HARVEST_DATA_DIR')
    if override:
        return Path(override)
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'peach_task_executor').is_dir():
            return parent / 'harvest_runs'
    return Path.cwd() / 'harvest_runs'


def ledger_file(root: Path, request_id: str) -> Path:
    """单个批次的 ledger.json 路径."""
    safe = request_id.strip() or 'harvest'
    return Path(root) / safe / 'ledger.json'


def outcome_to_dict(outcome: TargetOutcome) -> dict:
    """Serialize TargetOutcome to a JSON-friendly dict."""
    return {
        'target_id': str(outcome.target_id),
        'outcome': int(outcome.outcome),
        'reason': str(outcome.reason),
        'quality_score': float(getattr(outcome, 'quality_score', 0.0) or 0.0),
    }


def dict_to_outcome(data: dict) -> TargetOutcome:
    """Build TargetOutcome from a ledger dict."""
    item = TargetOutcome()
    item.target_id = str(data.get('target_id', ''))
    item.outcome = int(data.get('outcome', TargetOutcome.FAILED))
    item.reason = str(data.get('reason', ''))
    item.quality_score = float(data.get('quality_score', 0.0) or 0.0)
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


def save_ledger(path: Path, claimed, outcomes) -> None:
    """原子写 ledger.json."""
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        'claimed': sorted(str(tid) for tid in claimed if tid),
        'outcomes': [outcome_to_dict(item) for item in outcomes],
    }
    tmp = path.with_suffix('.json.tmp')
    tmp.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    tmp.replace(path)
