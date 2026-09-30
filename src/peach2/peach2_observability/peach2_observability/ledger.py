"""Batch ledger read-only access with request_id sanitization (zero ROS)."""
from __future__ import annotations

import json
from pathlib import Path

OUTCOME_NAMES = {
    0: 'SUCCEEDED',
    1: 'SKIPPED_QUALITY',
    2: 'SKIPPED_UNREACHABLE',
    3: 'FAILED',
    4: 'CANCELED',
}

_ROW_PASSTHROUGH = (
    'failure_code', 'build_view_count', 'build_status', 'build_duration_s',
    'timeout_source', 'completion_level', 'failure_code_n',
)


def sanitize_request_id(request_id: str) -> str | None:
    """Reject path traversal and path separators."""
    text = str(request_id or '').strip()
    if not text or text in ('.', '..') or '/' in text or '\\' in text or '\0' in text:
        return None
    return text


def ledger_path(runs_root: Path, request_id: str) -> Path | None:
    safe = sanitize_request_id(request_id)
    if safe is None:
        return None
    root = Path(runs_root).resolve()
    path = (root / safe / 'ledger.json').resolve()
    try:
        path.relative_to(root)
    except ValueError:
        return None
    return path


def read_ledger(runs_root: Path, request_id: str) -> dict:
    """Read ``runs/<request_id>/ledger.json``; never escapes runs_root."""
    safe = sanitize_request_id(request_id)
    if safe is None:
        return {'error': 'invalid request_id', 'request_id': str(request_id or '')}
    path = ledger_path(runs_root, safe)
    assert path is not None
    try:
        stat = path.stat()
    except OSError:
        return {
            'request_id': safe,
            'path': str(path),
            'error': '账本尚未生成（等待首颗终局）',
            'rows': [],
            'claimed': [],
            'totals': {},
        }
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(document, dict):
            raise ValueError('ledger root is not an object')
        rows = [_row(item) for item in document.get('outcomes') or []]
        claimed = sorted(str(item) for item in document.get('claimed') or [] if item)
        return {
            'request_id': safe,
            'path': str(path),
            'mtime': stat.st_mtime,
            'error': None,
            'claimed': claimed,
            'rows': rows,
            'totals': _totals(rows, claimed),
        }
    except (OSError, ValueError, TypeError, AttributeError) as error:
        return {
            'request_id': safe,
            'path': str(path),
            'error': f'账本解析失败: {error}',
            'rows': [],
            'claimed': [],
            'totals': {},
        }


def _round_s(value: float) -> float:
    return round(float(value), 3)


def _row(item) -> dict:
    if not isinstance(item, dict):
        item = {}
    try:
        outcome = int(item.get('outcome', 3))
    except (TypeError, ValueError):
        outcome = 3
    names = [str(name) for name in item.get('stage_names') or []]
    durations = item.get('stage_durations') or []
    stages = []
    for index, name in enumerate(names):
        try:
            dur = _round_s(durations[index])
        except (IndexError, TypeError, ValueError):
            dur = None
        stages.append({'name': name, 'dur_s': dur})
    try:
        elapsed = _round_s(item.get('elapsed_s'))
    except (TypeError, ValueError):
        elapsed = None
    row = {
        'target_id': str(item.get('target_id') or ''),
        'outcome': outcome,
        'outcome_name': OUTCOME_NAMES.get(outcome, str(outcome)),
        'reason': str(item.get('reason') or ''),
        'quality_score': item.get('quality_score'),
        'elapsed_s': elapsed,
        'stages': stages,
    }
    for key in _ROW_PASSTHROUGH:
        if item.get(key) is not None:
            row[key] = item.get(key)
    return row


def _totals(rows: list[dict], claimed: list[str]) -> dict:
    totals = {
        name: 0 for name in (
            'SUCCEEDED', 'SKIPPED_QUALITY', 'SKIPPED_UNREACHABLE', 'FAILED', 'CANCELED')
    }
    for row in rows:
        key = str(row.get('outcome_name') or '')
        if key in totals:
            totals[key] += 1
    totals['attempted'] = len(rows)
    totals['claimed'] = len(claimed)
    return totals
