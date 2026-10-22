"""一次开栈一个目录：终局收进正在录的 ``session_<时间>/cycles``."""
from __future__ import annotations

import json
import re
import time
from datetime import datetime
from pathlib import Path

from peach_common.paths import safe_component

from .batch import ledger_file


def live_session_dir(root: Path, now: float | None = None,
                     fresh_s: float = 20.0) -> Path | None:
    """袋文件还在写的 ``session_*``。已封口的旧会话不接收新周期."""
    root = Path(root)
    if not root.is_dir():
        return None
    clock = time.time() if now is None else float(now)
    best: Path | None = None
    best_mtime = -1.0
    try:
        children = list(root.iterdir())
    except OSError:
        return None
    for session in children:
        if not session.is_dir() or not session.name.startswith('session_'):
            continue
        bag = session / 'bag'
        if not bag.is_dir():
            continue
        newest = _newest_mtime(bag)
        if newest < 0.0 or (clock - newest) > fresh_s:
            continue
        if newest > best_mtime:
            best = session
            best_mtime = newest
    return best


def archive_open_session(root: Path, request_id: str,
                         now: float | None = None) -> list[Path]:
    """账本刚落盘后，把这一批终局抄进当前会话。没有正在录的会话则不写."""
    session = live_session_dir(root, now=now)
    if session is None:
        return []
    return archive_ledger_into(
        session, ledger_file(root, request_id), now=now)


def archive_ledger_into(session: Path, ledger_path: Path,
                        now: float | None = None,
                        single_only: bool = False) -> list[Path]:
    """按终局墙钟写 ``cycles/<时间>_<目标>.json``。同一批次同一颗覆盖原文件."""
    ledger_path = Path(ledger_path)
    try:
        payload = json.loads(ledger_path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return []
    outcomes = payload.get('outcomes') if isinstance(payload, dict) else None
    if not isinstance(outcomes, list) or not outcomes:
        return []
    if now is None:
        try:
            wall_end = ledger_path.stat().st_mtime
        except OSError:
            return []
    else:
        wall_end = float(now)
    if single_only and sum(isinstance(item, dict) for item in outcomes) != 1:
        return []
    request_id = ledger_path.parent.name
    written = []
    for item in outcomes:
        if not isinstance(item, dict):
            continue
        path = _cycle_path(session, request_id, item, wall_end)
        if path is None:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        elapsed = _elapsed(item)
        start = wall_end - elapsed if elapsed is not None else wall_end
        previous = _read_json(path)
        if previous is not None and previous.get('end_unix') is not None:
            # 后一颗落盘会重写整本账本。已归档的墙钟保持第一次写下的终局时刻。
            wall_end_item = float(previous['end_unix'])
            start = float(previous.get('start_unix') or wall_end_item)
        else:
            wall_end_item = wall_end
        document = {
            'request_id': request_id,
            'target_id': str(item.get('target_id') or ''),
            'outcome': item.get('outcome'),
            'reason': item.get('reason') or '',
            'elapsed_s': item.get('elapsed_s'),
            'completion_level': item.get('completion_level'),
            'failure_code': item.get('failure_code'),
            'stage_names': list(item.get('stage_names') or []),
            'stage_durations': list(item.get('stage_durations') or []),
            'wall_start': _iso(start),
            'wall_end': _iso(wall_end_item),
            'start_unix': round(start, 3),
            'end_unix': round(wall_end_item, 3),
        }
        _atomic_json(path, document)
        written.append(path)
    return written


def adopt_request_ledgers(root: Path) -> list[Path]:
    """把已写在 ``web_*`` / ``grasp_*`` 的单批账本归进当时那次开栈."""
    root = Path(root)
    spans = _session_spans(root)
    written = []
    try:
        children = list(root.iterdir())
    except OSError:
        return written
    for child in children:
        if not child.is_dir() or not (
                child.name.startswith('web_') or child.name.startswith('grasp_')):
            continue
        ledger = child / 'ledger.json'
        if not ledger.is_file():
            continue
        try:
            mtime = ledger.stat().st_mtime
        except OSError:
            continue
        for session, start, end in spans:
            if start <= mtime < end:
                written.extend(archive_ledger_into(
                    session, ledger, now=mtime, single_only=True))
                break
    return written


def _session_spans(root: Path) -> list[tuple[Path, float, float]]:
    sessions = []
    try:
        children = list(root.iterdir())
    except OSError:
        return []
    for child in children:
        if not child.is_dir() or not child.name.startswith('session_'):
            continue
        clock = _clock(child.name)
        if clock is None:
            continue
        sessions.append((child, clock.timestamp()))
    sessions.sort(key=lambda item: item[1])
    spans = []
    for index, (path, start) in enumerate(sessions):
        if index + 1 < len(sessions):
            end = sessions[index + 1][1]
        else:
            end = start + 6 * 3600
        if end <= start:
            end = start + 6 * 3600
        spans.append((path, start, end))
    return spans


def _clock(name: str) -> datetime | None:
    match = re.search(r'(20\d{6})_(\d{6})', name)
    if not match:
        return None
    try:
        return datetime.strptime(match.group(1) + match.group(2), '%Y%m%d%H%M%S')
    except ValueError:
        return None


def _cycle_path(session: Path, request_id: str, item: dict,
                wall_end: float) -> Path | None:
    target = str(item.get('target_id') or '')
    if not target:
        return None
    cycles = session / 'cycles'
    if cycles.is_dir():
        for path in cycles.glob('*.json'):
            try:
                previous = json.loads(path.read_text(encoding='utf-8'))
            except (OSError, UnicodeError, json.JSONDecodeError):
                continue
            if (isinstance(previous, dict)
                    and previous.get('request_id') == request_id
                    and str(previous.get('target_id') or '') == target):
                return path
    stamp = datetime.fromtimestamp(wall_end).strftime('%Y%m%d_%H%M%S')
    safe_target = safe_component(target, 'target')
    return cycles / f'{stamp}_{safe_target}.json'


def _read_json(path: Path) -> dict | None:
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _elapsed(item: dict) -> float | None:
    try:
        value = float(item.get('elapsed_s'))
    except (TypeError, ValueError):
        return None
    if value != value or value <= 0:
        return None
    return value


def _iso(stamp: float) -> str:
    return datetime.fromtimestamp(stamp).strftime('%Y-%m-%dT%H:%M:%S')


def _atomic_json(path: Path, document: dict) -> None:
    tmp = path.with_suffix('.json.tmp')
    tmp.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    tmp.replace(path)


def _newest_mtime(bag: Path) -> float:
    newest = -1.0
    try:
        newest = bag.stat().st_mtime
    except OSError:
        return -1.0
    try:
        children = list(bag.iterdir())
    except OSError:
        return newest
    for child in children:
        try:
            newest = max(newest, child.stat().st_mtime)
        except OSError:
            continue
    return newest
