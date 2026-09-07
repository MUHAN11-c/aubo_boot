from __future__ import annotations
"""选果、控制面、摘要与账本（批次纯函数）。"""

import json
import os
from pathlib import Path

from peach_interfaces.msg import TargetOutcome


def default_runs_root() -> Path:
    """
    过程数据根目录：工作区 ``runs/``（本包等价实现，不跨包 import）.

    优先级：AUBO_RUNS_DIR / AUBO_HARVEST_DATA_DIR 环境变量
    > 从本文件向上找含 ``src/peach_interfaces`` 的工作区根，取其 ``runs/``
    > ``Path.cwd()/runs`` 兜底。语义与感知 ``common/runtime`` 同名实现一致。
    """
    override = os.environ.get('AUBO_RUNS_DIR') or os.environ.get(
        'AUBO_HARVEST_DATA_DIR')
    if override:
        return Path(override)
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'peach_interfaces').is_dir():
            return parent / 'runs'
    return Path.cwd() / 'runs'


def resolve_runs_root(configured: str = '') -> Path:
    """参数给出绝对路径则用之，否则回 ``default_runs_root()``."""
    text = str(configured or '').strip()
    if text:
        path = Path(text)
        if path.is_absolute():
            return path
    return default_runs_root()


def default_ledger_root() -> Path:
    """账本根目录，与观测/session 同为工作区 ``runs/``."""
    return default_runs_root()


def _safe_run_component(request_id: str, fallback: str) -> str:
    """目录名段：拒绝含 / 、反斜杠或 .. 的 id（路径穿越），回退 fallback."""
    text = str(request_id or '').strip()
    if not text or any(token in text for token in ('/', '\\', '..')):
        return fallback
    return text


def ledger_file(root: Path, request_id: str) -> Path:
    """单个批次的 ledger.json 路径（request_id 过滤路径穿越）."""
    safe = _safe_run_component(request_id, 'harvest')
    return Path(root) / safe / 'ledger.json'


def target_artifact_dir(root: Path, request_id: str, target_id: str) -> Path:
    """runs/<request_id>/targets/<target_id>/ 过程目录（同规则过滤）."""
    safe_run = _safe_run_component(request_id, 'harvest')
    safe_tid = _safe_run_component(target_id, 'target')
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
    """把墙钟秒写入 TargetOutcome.elapsed（小数进位到 sec）."""
    value = max(float(seconds or 0.0), 0.0)
    sec = int(value)
    nanosec = int(round((value - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    outcome.elapsed.sec = sec
    outcome.elapsed.nanosec = nanosec


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
