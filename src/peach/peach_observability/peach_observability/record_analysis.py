"""离线读取 runs 下的测试记录。只读，不改监控状态."""
from __future__ import annotations

import json
import os
import re
from datetime import datetime, timedelta
from pathlib import Path
from urllib.parse import quote

_MARKERS = ('ledger.json', 'geometry.jsonl', 'report.json')
_IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.webp'}
_SKIP_DIRS = {'bag', 'ros_home', 'ros_logs'}
_MAX_LINES = 4000
_MAX_IMAGE_BYTES = 8_000_000


def _under(root: Path, relative: str) -> Path:
    """只允许 runs 根目录里面的相对路径."""
    root = root.resolve()
    text = str(relative or '').strip().replace('\\', '/')
    if not text or text.startswith('/') or text.startswith('~'):
        raise ValueError('请选择 runs 里的记录目录')
    path = (root / text).resolve()
    if path != root and root not in path.parents:
        raise ValueError('路径超出测试记录目录')
    return path


def _mm(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number:
        return None
    return round(number * 1000.0, 1)


def _xyz(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        point = [round(float(item), 4) for item in value]
    except (TypeError, ValueError):
        return None
    if any(item != item for item in point):
        return None
    return point


def _stats(values: list[float]) -> dict | None:
    if not values:
        return None
    ordered = sorted(values)
    return {
        'count': len(ordered),
        'min': round(ordered[0], 1),
        'max': round(ordered[-1], 1),
        'mean': round(sum(ordered) / len(ordered), 1),
    }


def list_records(root: Path, limit: int = 60) -> list[dict]:
    """列出含账本、几何或测试报告的目录，新的在前."""
    root = root.resolve()
    found = []
    if not root.is_dir():
        return found
    # 只列 runs/ 顶层记录：runner 现在把 session/round/感知数据收进
    # 同一个 output/records 下，嵌套目录不再作为第二个分析选项。
    candidates = [root]
    candidates.extend(path for path in root.iterdir() if path.is_dir())
    seen = set()
    for directory in candidates:
        if directory in seen or directory == root:
            continue
        seen.add(directory)
        try:
            markers = [name for name in _MARKERS if (directory / name).is_file()]
            # 只列真正的开批会话（有 startup.json）；独立的 observability
            # 自会话只有 selfcheck/bag、没有作业数据，列出来会占“最新”位。
            if directory.name.startswith('session_') and (
                    (directory / 'startup.json').is_file()
                    or any((directory / 'cycles').glob('*.json'))):
                markers.append('session')
        except OSError:
            continue
        if not markers:
            continue
        try:
            stamp = directory.stat().st_mtime
        except OSError:
            stamp = 0
        found.append({
            'path': directory.relative_to(root).as_posix(),
            'markers': markers,
            'mtime': stamp,
        })
    # 同一次 runner 启动会同时留下 run report 和 observability session；
    # 分析页只留 run report（report.json + 已聚合的 round 账本），有对应
    # run report 的 session 不再重复列成第二个选项。
    run_clocks = []
    for directory in seen:
        try:
            if (directory / 'report.json').is_file():
                clock = _clock(directory.name)
                if clock is not None:
                    run_clocks.append(clock)
        except OSError:
            continue
    if run_clocks:
        unique = []
        for item in found:
            directory = root / item['path']
            if directory.name.startswith('session_'):
                start = _clock(directory.name)
                if (start is not None and any(
                        start <= clock < start + timedelta(hours=6)
                        for clock in run_clocks)):
                    continue
            unique.append(item)
        found = unique
    # 顶层 run report 已点名的 round_* 不再单独列（旧 runner 把 round 写在
    # runs/ 根上）；新 runner 的 round 已在 output/records 下，本来就不在顶层。
    covered = set()
    try:
        for directory in root.iterdir():
            if not directory.is_dir():
                continue
            report = _read_json(directory / 'report.json')
            if not isinstance(report, dict):
                continue
            for item in report.get('rounds') or []:
                if isinstance(item, dict) and item.get('request_id'):
                    covered.add(str(item['request_id']))
    except OSError:
        pass
    if covered:
        found = [item for item in found
                 if not (Path(item['path']).name.startswith('round_')
                         and Path(item['path']).name in covered)]
    owned = _requests_owned_by_sessions(root)
    if owned:
        found = [item for item in found if Path(item['path']).name not in owned]
    found.sort(key=lambda item: item['mtime'], reverse=True)
    return found[:limit]


def _attempt_ok(outcome, level: int) -> tuple[bool, bool]:
    """套入完成 = 终局成功且完成度至少 6；采摘确认 = 完成度至少 7."""
    sleeve = outcome == 0 and level >= 6
    return sleeve, level >= 7


def _attempt_row(item: dict, batch: str, when: float | None) -> dict | None:
    if not isinstance(item, dict) or 'outcome' not in item:
        return None
    try:
        level = int(item.get('completion_level') or 0)
        outcome = int(item.get('outcome'))
    except (TypeError, ValueError):
        return None
    sleeve, harvest = _attempt_ok(outcome, level)
    try:
        elapsed = float(item.get('elapsed_s'))
        if elapsed != elapsed:
            elapsed = None
    except (TypeError, ValueError):
        elapsed = None
    return {
        'batch': batch,
        'target_id': str(item.get('target_id') or ''),
        'outcome': outcome,
        'completion_level': level,
        'elapsed_s': None if elapsed is None else round(elapsed, 3),
        'sleeve_ok': sleeve,
        'harvest_ok': harvest,
        'when': when,
    }


def _rate_block(rows: list[dict]) -> dict:
    count = len(rows)
    sleeve = sum(1 for row in rows if row['sleeve_ok'])
    harvest = sum(1 for row in rows if row['harvest_ok'])
    return {
        'attempts': count,
        'sleeve_ok': sleeve,
        'harvest_ok': harvest,
        'sleeve_rate': round(sleeve / count, 4) if count else None,
        'harvest_rate': round(harvest / count, 4) if count else None,
    }


def success_summary(root: Path, session_limit: int = 12) -> dict:
    """每次抓取一条。会话 cycles 优先；同一 request_id 的账本不重复计."""
    root = root.resolve()
    rows = []
    covered = set()
    for path in root.glob('session_*/cycles/*.json'):
        item = _read_json(path)
        if not isinstance(item, dict):
            continue
        batch = str(item.get('request_id') or path.stem)
        covered.add(batch)
        when = None
        try:
            when = float(item.get('start_unix'))
        except (TypeError, ValueError):
            when = None
        row = _attempt_row(item, batch, when)
        if row is not None:
            row['session'] = path.parent.parent.name
            rows.append(row)
    ledgers = list(root.glob('*/ledger.json'))
    ledgers.extend(root.glob('*/*/ledger.json'))
    ledgers.extend(root.glob('*/*/*/ledger.json'))
    seen_ledger = set()
    for path in ledgers:
        if path in seen_ledger or any(part in _SKIP_DIRS for part in path.parts):
            continue
        seen_ledger.add(path)
        if path.parent.name in covered:
            continue
        payload = _read_json(path)
        outcomes = payload.get('outcomes') if isinstance(payload, dict) else None
        if not isinstance(outcomes, list):
            continue
        try:
            stamp = path.stat().st_mtime
        except OSError:
            stamp = None
        for item in outcomes:
            row = _attempt_row(item, path.parent.name, stamp)
            if row is not None:
                row['session'] = path.parent.name
                rows.append(row)
    today = datetime.now().date()
    today_rows = []
    for row in rows:
        if row['when'] is None:
            continue
        if datetime.fromtimestamp(row['when']).date() == today:
            today_rows.append(row)
    grouped: dict[str, list[dict]] = {}
    for row in rows:
        grouped.setdefault(str(row.get('session') or row['batch']), []).append(row)
    sessions = []
    for name, group in grouped.items():
        block = _rate_block(group)
        stamps = [row['when'] for row in group if row['when'] is not None]
        block['name'] = name
        block['when'] = max(stamps) if stamps else None
        sessions.append(block)
    sessions.sort(key=lambda item: item['when'] or 0, reverse=True)
    return {
        'today': _rate_block(today_rows),
        'all': _rate_block(rows),
        'sessions': sessions[:session_limit],
    }


def _read_json(path: Path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None


def _geometry_rows(path: Path) -> list[dict]:
    rows = []
    try:
        lines = path.read_text(encoding='utf-8').splitlines()
    except (OSError, UnicodeError):
        return rows
    for line in lines[:_MAX_LINES]:
        if not line.strip():
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict) and item.get('fused') is True:
            rows.append(item)
    return rows


def _image_item(path: Path, record: Path, runs_root: Path, relative: str) -> dict | None:
    try:
        name = path.relative_to(record).as_posix()
    except ValueError:
        try:
            name = path.relative_to(runs_root).as_posix()
        except ValueError:
            return None
    if '..' in name.split('/'):
        return None
    return {
        'name': name,
        'url': (
            '/api/analysis/image?path=' + quote(relative)
            + '&name=' + quote(name)),
    }


def _stamp_target(path: Path) -> tuple[int, str] | None:
    stamp, sep, target = path.stem.partition('_')
    if sep and stamp.isdigit() and target:
        return int(stamp), target
    return None


def _owner_batch(path: Path) -> str:
    for parent in path.parents:
        if (parent / 'ledger.json').is_file():
            return parent.name
    return ''


def _frame_index(runs_root: Path, record: Path, relative: str,
                 roots: list[Path]) -> dict:
    """按批次和目标收齐最新一帧的掩膜和检测调试图。彩色原图只留在磁盘。"""
    latest: dict[tuple[str, str], dict] = {}
    kinds = {'masks': 'mask', 'debug': 'debug'}
    for base in roots:
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [
                name for name in dirnames
                if name not in _SKIP_DIRS | {'analysis', 'colors'}]
            kind = kinds.get(Path(dirpath).name)
            if kind is None:
                continue
            for filename in filenames:
                path = Path(dirpath) / filename
                parsed = _stamp_target(path)
                if parsed is None or path.suffix.lower() not in _IMAGE_SUFFIXES:
                    continue
                try:
                    size = path.stat().st_size
                except OSError:
                    continue
                if size <= 0 or size > _MAX_IMAGE_BYTES:
                    continue
                stamp, target = parsed
                item = _image_item(path, record, runs_root, relative)
                if item is None:
                    continue
                slot = latest.setdefault((_owner_batch(path), target), {})
                current = slot.get(kind)
                if current is None or stamp >= current[0]:
                    slot[kind] = (stamp, item)
    return {
        key: {kind: pair[1] for kind, pair in shots.items()}
        for key, shots in latest.items()
    }


def _stage_rows(item: dict) -> list[dict]:
    names = item.get('stage_names') or []
    durations = item.get('stage_durations') or []
    rows = []
    for name, duration in zip(names, durations):
        try:
            rows.append({'name': str(name), 's': round(float(duration), 2)})
        except (TypeError, ValueError):
            continue
    return rows


def _cycle_stages(item: dict) -> list[dict]:
    """补上派发到再确认之前的时间。这段主要是等精化几何和许可，不是开批检测。"""
    rows = _stage_rows(item)
    if any(row['name'] in {'perceive', 'observe', 'finalize'} for row in rows):
        return rows
    try:
        elapsed = float(item.get('elapsed_s'))
    except (TypeError, ValueError):
        return rows
    if elapsed != elapsed or elapsed <= 0:
        return rows
    known = sum(row['s'] for row in rows)
    leading = round(elapsed - known, 2)
    if leading < 0.05:
        return rows
    return [{'name': 'finalize', 's': leading}] + rows


def _geometry_for(geometry: list[dict], batch, target: str) -> dict | None:
    fallback = None
    for row in geometry:
        if str(row.get('target_id') or '') != target:
            continue
        fallback = row
        if str(row.get('batch') or '') == str(batch or ''):
            return row
    return fallback


def _fruits(outcomes: list[dict], frames: dict, geometry: list[dict]) -> list[dict]:
    """每一颗都带上掩膜、检测调试图和本颗评估。彩色原图不进页面。"""
    rows = []
    for item in outcomes:
        target = str(item.get('target_id') or '')
        shots = frames.get((str(item.get('batch') or ''), target), {})
        geo = _geometry_for(geometry, item.get('batch'), target) or {}
        bottom, neck = _xyz(geo.get('bag_bottom')), _xyz(geo.get('bag_neck'))
        center = None
        if bottom is not None and neck is not None:
            center = [
                round((bottom[0] + neck[0]) / 2, 3),
                round((bottom[1] + neck[1]) / 2, 3),
            ]
        rows.append({
            'batch': item.get('batch'),
            'target_id': target,
            'outcome': item.get('outcome'),
            'reason': item.get('reason'),
            'elapsed_s': item.get('elapsed_s'),
            'completion_level': item.get('completion_level'),
            'stages': _cycle_stages(item),
            'd95_mm': _mm(geo.get('d95_m')),
            'radial_margin_mm': _mm(geo.get('radial_margin_m')),
            'axial_margin_mm': _mm(geo.get('axial_margin_m')),
            'view_count': geo.get('view_count'),
            'center_m': center,
            'points': _points(geo),
            'mask': shots.get('mask'),
            'debug': shots.get('debug'),
        })
    rows.sort(key=lambda row: str(row.get('batch') or ''))
    return rows


def _listed_names(report: dict) -> set[str]:
    names = set()
    for fruit in report.get('fruits') or []:
        for key in ('mask', 'debug'):
            shot = fruit.get(key) or {}
            if shot.get('name'):
                names.add(shot['name'])
    return names


def _points(row: dict) -> list[dict]:
    named = (
        ('袋底', row.get('bag_bottom')),
        ('袋口', row.get('bag_neck')),
        ('剪切参考', row.get('cut_pose')),
    )
    return [
        {'name': name, 'xyz': point}
        for name, value in named
        if (point := _xyz(value)) is not None
    ]


def _clock(name: str) -> datetime | None:
    match = re.search(r'(20\d{6})[_T](\d{6})', name)
    if match:
        return datetime.strptime(match.group(1) + match.group(2), '%Y%m%d%H%M%S')
    # runner 输出目录常见 HHMM（如 live_mock_unified_rviz_20261021_1813）；
    # 会话关联必须能解析，否则同一次启动的 round 账本永远挂不上。
    match = re.search(r'(20\d{6})[_-](\d{4})$', name)
    if match:
        return datetime.strptime(match.group(1) + match.group(2), '%Y%m%d%H%M')
    match = re.fullmatch(r'flow(?:10)?_(\d{6})(?:_r\d+)?', name)
    if match:
        return datetime.strptime(match.group(1), '%H%M%S')
    return None


def _report_round_roots(record: Path) -> list[Path]:
    """runner 的 report.json 把逐目标账本写在 runs/<request_id> 兄弟目录."""
    report = _read_json(record / 'report.json')
    roots = []
    if not isinstance(report, dict):
        return roots
    for item in report.get('rounds') or []:
        if not isinstance(item, dict):
            continue
        request_id = str(item.get('request_id') or '')
        if not request_id or record.parent is None:
            continue
        candidate = record.parent / request_id
        if candidate.is_dir():
            roots.append(candidate)
    return roots


def _session_spans(parent: Path) -> list[tuple[Path, float, float]]:
    """一次开栈从目录名时刻到下一次开栈；最后一次再留 6 小时."""
    sessions = []
    try:
        children = list(parent.iterdir())
    except OSError:
        return []
    for child in children:
        if not child.is_dir() or not child.name.startswith('session_'):
            continue
        clock = _clock(child.name)
        if clock is not None:
            sessions.append((child, clock.timestamp()))
    sessions.sort(key=lambda item: item[1])
    spans = []
    for index, (path, start) in enumerate(sessions):
        end = sessions[index + 1][1] if index + 1 < len(sessions) else start + 6 * 3600
        if end <= start:
            end = start + 6 * 3600
        spans.append((path, start, end))
    return spans


def _requests_owned_by_sessions(root: Path) -> set[str]:
    """已经收进某次开栈的批次目录，分析列表不再单列."""
    names = set()
    try:
        children = list(root.iterdir())
    except OSError:
        return names
    for session, start, end in _session_spans(root):
        listed = (
            (session / 'startup.json').is_file()
            or any((session / 'cycles').glob('*.json'))
            or any((session / name).is_file() for name in _MARKERS))
        if not listed:
            continue
        cycles = session / 'cycles'
        if cycles.is_dir():
            for path in cycles.glob('*.json'):
                payload = _read_json(path)
                if isinstance(payload, dict) and payload.get('request_id'):
                    names.add(str(payload['request_id']))
        for child in children:
            if not child.is_dir() or (child / 'report.json').is_file():
                continue
            if not (child.name.startswith('web_') or child.name.startswith('grasp_')):
                continue
            ledger = child / 'ledger.json'
            try:
                mtime = ledger.stat().st_mtime
            except OSError:
                continue
            if start <= mtime < end:
                names.add(child.name)
    return names


def _related_roots(record: Path) -> list[Path]:
    """会话带上同一次启动的批次；runner 目录带上 report 里点名的 round."""
    roots = [record]
    seen = {record}
    roots.extend(_report_round_roots(record))
    for path in roots[1:]:
        seen.add(path)
    if not record.name.startswith('session_') or record.parent is None:
        return roots
    owned = set()
    cycles = record / 'cycles'
    if cycles.is_dir():
        for path in cycles.glob('*.json'):
            payload = _read_json(path)
            if isinstance(payload, dict) and payload.get('request_id'):
                owned.add(str(payload['request_id']))
    if owned:
        try:
            siblings = list(record.parent.iterdir())
        except OSError:
            return roots
        for child in siblings:
            if child.is_dir() and child.name in owned and child not in seen:
                roots.append(child)
                seen.add(child)
        return roots
    start = _clock(record.name)
    if start is None:
        return roots
    end = start + timedelta(hours=6)
    span = next(
        (item for item in _session_spans(record.parent) if item[0] == record),
        None)
    if span is not None:
        end = datetime.fromtimestamp(span[2])
    try:
        siblings = list(record.parent.iterdir())
    except OSError:
        return roots
    for child in siblings:
        if not child.is_dir() or child in seen or child.name.startswith('session_'):
            continue
        clock = _clock(child.name)
        named = False
        if clock is not None:
            if clock.year == 1900:
                clock = start.replace(
                    hour=clock.hour, minute=clock.minute, second=clock.second,
                    microsecond=0)
            named = start <= clock < end
        mtimed = False
        if child.name.startswith('web_') or child.name.startswith('grasp_'):
            ledger = child / 'ledger.json'
            try:
                mtimed = start.timestamp() <= ledger.stat().st_mtime < end.timestamp()
            except OSError:
                mtimed = False
        if named or mtimed:
            roots.append(child)
            seen.add(child)
            for extra in _report_round_roots(child):
                if extra not in seen:
                    roots.append(extra)
                    seen.add(extra)
    return roots


def _outcome_rows(roots: list[Path]) -> list[dict]:
    rows = []
    for root in roots:
        for path in root.rglob('ledger.json'):
            if 'analysis' in path.parts:
                continue
            payload = _read_json(path)
            outcomes = payload.get('outcomes') if isinstance(payload, dict) else None
            if not isinstance(outcomes, list):
                continue
            for item in outcomes:
                if not isinstance(item, dict):
                    continue
                row = dict(item)
                row['batch'] = path.parent.name
                rows.append(row)
    return rows


def _all_geometry(roots: list[Path]) -> list[dict]:
    rows = []
    for root in roots:
        for path in root.rglob('geometry.jsonl'):
            if 'analysis' in path.parts:
                continue
            for row in _geometry_rows(path):
                row['batch'] = path.parent.name
                rows.append(row)
    return rows


def _frame_counts(roots: list[Path]) -> tuple[int, int, dict]:
    accepted = 0
    codes: dict[str, int] = {}
    for root in roots:
        for path in root.rglob('events.jsonl'):
            try:
                lines = path.read_text(encoding='utf-8').splitlines()
            except (OSError, UnicodeError):
                continue
            for line in lines[:_MAX_LINES]:
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                kind = event.get('event')
                if kind == 'frame_accepted':
                    accepted += 1
                elif kind == 'frame_skipped':
                    code = str(event.get('code') or 'other')
                    codes[code] = codes.get(code, 0) + 1
    return accepted, sum(codes.values()), codes


def _stage_means(outcomes: list[dict]) -> dict:
    totals: dict[str, list[float]] = {}
    for item in outcomes:
        if item.get('outcome') != 0:
            continue
        for row in _cycle_stages(item):
            totals.setdefault(row['name'], []).append(row['s'])
    return {
        name: round(sum(values) / len(values), 2)
        for name, values in totals.items() if values
    }


def _rates(outcomes: list[dict], accepted: int, skipped: int) -> dict:
    fruits = len(outcomes)
    completed = sum(
        item.get('outcome') == 0 and int(item.get('completion_level') or 0) >= 6
        for item in outcomes)
    elapsed = []
    for item in outcomes:
        if item.get('outcome') != 0:
            continue
        try:
            elapsed.append(float(item.get('elapsed_s')))
        except (TypeError, ValueError):
            continue
    elapsed.sort()
    median = round(elapsed[len(elapsed) // 2], 3) if elapsed else None
    return {
        'fruits': fruits,
        'motion_completed': completed,
        'motion_completed_rate': round(completed / fruits, 4) if fruits else None,
        'unreachable': sum(item.get('outcome') == 2 for item in outcomes),
        'quality_skipped': sum(item.get('outcome') == 1 for item in outcomes),
        'failed': sum(item.get('outcome') == 3 for item in outcomes),
        'canceled': sum(item.get('outcome') == 4 for item in outcomes),
        'harvest_confirmed': sum(
            int(item.get('completion_level') or 0) >= 7 for item in outcomes),
        'elapsed_s_median': median,
        'stage_mean_s': _stage_means(outcomes),
        'frames_accepted': accepted,
        'frames_skipped': skipped,
    }


def _centers(geometry: list[dict], outcomes: list[dict]) -> list[dict]:
    status = {
        (item.get('batch'), item.get('target_id')): item.get('outcome')
        for item in outcomes
    }
    marks = []
    seen = set()
    for row in geometry:
        bottom, neck = _xyz(row.get('bag_bottom')), _xyz(row.get('bag_neck'))
        if bottom is None or neck is None:
            continue
        target = str(row.get('target_id') or '')
        point = (
            target,
            row.get('batch'),
            round((bottom[0] + neck[0]) / 2, 4),
            round((bottom[1] + neck[1]) / 2, 4),
        )
        if point in seen:
            continue
        seen.add(point)
        marks.append({
            'target_id': target,
            'outcome': status.get((row.get('batch'), target)),
            'x': point[2],
            'y': point[3],
        })
    return marks


_FORCE_SLACK_S = 1.5
_FORCE_CHANNELS = ('f1', 'f2', 'f3', 'f5', 'f7')
# 与 serial_force.force_episode.RELEASE_N 相同：低于此值已经回到空载。
_RELEASE_N = 0.08


def _parse_wall(value) -> float | None:
    """接触段 sidecar 的 wall_start 是本机本地时间，没有时区后缀."""
    if not isinstance(value, str) or not value.strip():
        return None
    try:
        return datetime.fromisoformat(value.strip()).timestamp()
    except ValueError:
        return None


def _read_episode(path: Path) -> dict | None:
    payload = _read_json(path)
    if not isinstance(payload, dict):
        return None
    start = _parse_wall(payload.get('wall_start'))
    try:
        duration = float(payload.get('duration_s'))
    except (TypeError, ValueError):
        return None
    if start is None or duration != duration or duration <= 0:
        return None
    peaks = []
    raw_peaks = payload.get('peak_n')
    if isinstance(raw_peaks, list):
        for item in raw_peaks:
            try:
                peaks.append(float(item))
            except (TypeError, ValueError):
                peaks.append(0.0)
    peak_n = None
    peak_channel = None
    if peaks:
        index = max(range(len(peaks)), key=lambda i: peaks[i])
        peak_n = round(peaks[index], 3)
        if index < len(_FORCE_CHANNELS):
            peak_channel = _FORCE_CHANNELS[index]
    try:
        contact = float(payload.get('contact_s') or 0)
    except (TypeError, ValueError):
        contact = 0.0
    samples = _sample_times(path.with_suffix('.csv'))
    if samples is not None:
        contact = samples['contact_s']
        start = _episode_origin(
            path.with_suffix('.csv'), str(payload.get('wall_start') or ''),
            start, samples['last_s'])
        peak_s = samples['peak_s']
    else:
        peak_s = None
    return {
        'start': start,
        'end': start + duration,
        'duration_s': round(duration, 3),
        'contact_s': round(contact, 3),
        'peak_n': peak_n,
        'peak_channel': peak_channel,
        'peak_s': None if peak_s is None else round(peak_s, 3),
        'episode': path.stem,
        'reason': str(payload.get('reason') or ''),
    }


def _sample_times(csv_path: Path) -> dict | None:
    """从采样读出峰值时刻、最后一帧仍不低于空载门的时刻，和文件最后一帧.

    t_s 相对接触段第一帧，分辨率是采样间隔（约 10 ms），不经过整秒截断。
    """
    try:
        lines = csv_path.read_text(encoding='utf-8').splitlines()
    except (OSError, UnicodeError):
        return None
    if len(lines) < 2:
        return None
    header = lines[0].split(',')
    if 't_s' not in header:
        return None
    time_index = header.index('t_s')
    force_index = [
        header.index(name) for name in _FORCE_CHANNELS if name in header]
    if not force_index:
        return None
    best_t = None
    best_level = -1.0
    last_t = None
    last_hot = None
    for line in lines[1:]:
        cols = line.split(',')
        try:
            stamp = float(cols[time_index])
        except (IndexError, ValueError):
            continue
        last_t = stamp
        total = 0.0
        hot = 0.0
        for index in force_index:
            try:
                value = float(cols[index])
            except (IndexError, ValueError):
                continue
            if value != value:
                continue
            total += max(0.0, value)
            hot = max(hot, abs(value))
        if hot >= _RELEASE_N:
            last_hot = stamp
        if total > best_level:
            best_level = total
            best_t = stamp
    if last_t is None or best_t is None or best_level <= 0 or last_hot is None:
        return None
    return {'peak_s': best_t, 'contact_s': last_hot, 'last_s': last_t}


def _episode_origin(csv_path: Path, wall_text: str, wall_epoch: float,
                    last_s: float) -> float:
    """接触段起点的 unix 秒.

    带小数的 wall_start 直接用。只有整秒时，用 CSV 写完的时间减去最后一帧
    t_s，避免把起点提前到该秒的开头（最多差 1 秒）。mtime 若被后来改过，
    和整秒起点差过 1.5 秒就仍用整秒值。
    """
    clock = wall_text.strip().split('T')[-1]
    if '.' in clock:
        return wall_epoch
    try:
        written = csv_path.stat().st_mtime
    except OSError:
        return wall_epoch
    origin = written - float(last_s)
    if abs(origin - wall_epoch) <= 1.5:
        return origin
    return wall_epoch


def _force_parts(start_s: float, peak_s, contact_s) -> list[dict]:
    """上升止于合力最大的那一帧，下降止于仍不低于空载门的最后一帧.

    门之后还有约 0.35 s 归零等待才关文件。那段力已经是 0，不画进下降。
    """
    try:
        peak_at = float(start_s) + float(peak_s)
        hot_end = float(start_s) + float(contact_s)
    except (TypeError, ValueError):
        return []
    parts = []
    if peak_at - float(start_s) >= 0.005:
        parts.append({
            'kind': 'rise',
            'start_s': round(float(start_s), 3),
            'end_s': round(peak_at, 3),
        })
    if hot_end - peak_at >= 0.005:
        parts.append({
            'kind': 'fall',
            'start_s': round(peak_at, 3),
            'end_s': round(hot_end, 3),
        })
    return parts


def _episodes_in(directory: Path) -> list[dict]:
    force = directory / 'force'
    if not force.is_dir():
        return []
    rows = []
    for path in sorted(force.glob('episode_*.json')):
        row = _read_episode(path)
        if row is not None:
            rows.append(row)
    return rows


def _stages_crossed(stages: list[dict], start_s: float, end_s: float) -> list[str]:
    """力段覆盖到的非零阶段。零宽的刀具段不占时间，不算覆盖."""
    cursor = 0.0
    names = []
    for row in stages:
        width = float(row.get('s') or 0)
        nxt = cursor + width
        if width > 0 and nxt > start_s and cursor < end_s:
            names.append(str(row.get('name') or ''))
        cursor = nxt
    return [name for name in names if name]


def _single_grasp_cycles(root: Path) -> list[dict]:
    """一账本一颗才能用文件写入时间减去 elapsed 还原这一次的起点.

    多颗写在同一份账本里时，文件时间只落在最后一颗结束之后，
    不能把一段力摊到每一颗上。
    """
    cycles = []
    patterns = ('*/ledger.json', '*/*/*/ledger.json')
    seen = set()
    for pattern in patterns:
        for path in root.glob(pattern):
            if path in seen or any(part in _SKIP_DIRS for part in path.parts):
                continue
            seen.add(path)
            payload = _read_json(path)
            outcomes = payload.get('outcomes') if isinstance(payload, dict) else None
            if not isinstance(outcomes, list) or len(outcomes) != 1:
                continue
            item = outcomes[0]
            if not isinstance(item, dict):
                continue
            try:
                elapsed = float(item.get('elapsed_s'))
            except (TypeError, ValueError):
                continue
            if elapsed != elapsed or elapsed <= 0:
                continue
            try:
                end = path.stat().st_mtime
            except OSError:
                continue
            cycles.append({
                'batch': path.parent.name,
                'target_id': str(item.get('target_id') or ''),
                'elapsed_s': round(elapsed, 3),
                'stages': _cycle_stages(item),
                'start': end - elapsed,
                'end': end,
                'ledger': path.resolve(),
            })
    return cycles


def _match_force(cycles: list[dict], episodes: list[dict],
                 keep_all: bool = False) -> list[dict]:
    """接触段中点落在某一次抓取里，才把这段挂到那一次的时间轴上."""
    grouped: dict[Path, dict] = {}
    if keep_all:
        for cycle in cycles:
            grouped[cycle['ledger']] = _cycle_bucket(cycle)
    for episode in episodes:
        mid = (episode['start'] + episode['end']) / 2.0
        best = None
        best_dist = None
        for cycle in cycles:
            if mid < cycle['start'] - _FORCE_SLACK_S:
                continue
            if mid > cycle['end'] + _FORCE_SLACK_S:
                continue
            dist = abs(mid - (cycle['start'] + cycle['end']) / 2.0)
            if best is None or dist < best_dist:
                best = cycle
                best_dist = dist
        if best is None:
            continue
        offset = episode['start'] - best['start']
        start_s = round(offset, 3)
        end_s = round(episode['end'] - best['start'], 3)
        bucket = grouped.get(best['ledger'])
        if bucket is None:
            bucket = _cycle_bucket(best)
            grouped[best['ledger']] = bucket
        parts = _force_parts(
            offset, episode.get('peak_s'), episode.get('contact_s'))
        painted_end = parts[-1]['end_s'] if parts else end_s
        bucket['force'].append({
            'start_s': start_s,
            'end_s': end_s,
            'duration_s': episode['duration_s'],
            'contact_s': episode['contact_s'],
            'peak_n': episode['peak_n'],
            'peak_channel': episode['peak_channel'],
            'peak_s': episode.get('peak_s'),
            'parts': parts,
            'episode': episode['episode'],
            'reason': episode['reason'],
            'stages': _stages_crossed(best['stages'], start_s, painted_end),
        })
    rows = list(grouped.values())
    for row in rows:
        row['force'].sort(key=lambda item: item['start_s'])
        row.pop('_end', None)
    rows.sort(key=lambda row: (
        row['force'][0]['start_s'] if row['force'] else 1e12,
        row.get('when') or ''))
    return rows


def _cycle_bucket(cycle: dict) -> dict:
    when = datetime.fromtimestamp(cycle['start']).strftime('%H:%M:%S')
    return {
        'batch': cycle['batch'],
        'target_id': cycle['target_id'],
        'elapsed_s': cycle['elapsed_s'],
        'stages': cycle['stages'],
        'when': when,
        'force': [],
        '_end': cycle['end'],
    }


def _archived_cycles(record: Path) -> list[dict]:
    """会话目录里按墙钟归档的单颗周期."""
    cycles_dir = record / 'cycles'
    if not cycles_dir.is_dir():
        return []
    cycles = []
    for path in sorted(cycles_dir.glob('*.json')):
        item = _read_json(path)
        if not isinstance(item, dict):
            continue
        try:
            start = float(item.get('start_unix'))
            end = float(item.get('end_unix'))
            elapsed = float(item.get('elapsed_s'))
        except (TypeError, ValueError):
            continue
        if elapsed != elapsed or elapsed <= 0 or end < start:
            continue
        cycles.append({
            'batch': str(item.get('request_id') or path.stem),
            'target_id': str(item.get('target_id') or ''),
            'elapsed_s': round(elapsed, 3),
            'stages': _cycle_stages(item),
            'start': start,
            'end': end,
            'ledger': path.resolve(),
        })
    return cycles


def _cycle_outcome_rows(record: Path) -> list[dict]:
    cycles_dir = record / 'cycles'
    if not cycles_dir.is_dir():
        return []
    rows = []
    for path in sorted(cycles_dir.glob('*.json')):
        item = _read_json(path)
        if not isinstance(item, dict) or not item.get('target_id'):
            continue
        row = dict(item)
        row['batch'] = item.get('request_id') or path.stem
        rows.append(row)
    return rows


def _force_cycles(root: Path, record: Path) -> list[dict]:
    """这次记录自己的接触段，对上单颗账本；打开账本目录时反过来找会话里的段."""
    archived = _archived_cycles(record)
    if archived:
        return _match_force(archived, _episodes_in(record), keep_all=True)
    cycles = _single_grasp_cycles(root)
    local = _episodes_in(record)
    if local:
        return _match_force(cycles, local)
    own = [
        cycle for cycle in cycles
        if cycle['ledger'] == record.resolve() or record.resolve() in cycle['ledger'].parents
    ]
    if not own:
        return []
    episodes = []
    seen = set()
    for directory in root.glob('session_*'):
        for row in _episodes_in(directory):
            key = (row['episode'], row['start'])
            if key in seen:
                continue
            seen.add(key)
            episodes.append(row)
    return _match_force(own, episodes)


def analyze_record(root: Path, relative: str) -> dict:
    """解析一次测试目录里的账本、几何和图片."""
    record = _under(root, relative)
    if not record.is_dir():
        raise ValueError('记录目录不存在')
    relative = record.relative_to(root.resolve()).as_posix()
    roots = _related_roots(record)
    outcomes = _outcome_rows(roots)
    seen_outcomes = {
        (str(item.get('batch') or ''), str(item.get('target_id') or ''))
        for item in outcomes
    }
    for item in _cycle_outcome_rows(record):
        key = (str(item.get('batch') or ''), str(item.get('target_id') or ''))
        if key not in seen_outcomes:
            outcomes.append(item)
            seen_outcomes.add(key)
    if not outcomes:
        ledger = _read_json(record / 'ledger.json') or {}
        raw = ledger.get('outcomes') if isinstance(ledger, dict) else None
        outcomes = [item for item in raw if isinstance(item, dict)] if isinstance(raw, list) else []
    geometry = _all_geometry(roots) or _geometry_rows(record / 'geometry.jsonl')
    latest = {}
    for row in geometry:
        target = str(row.get('target_id') or '')
        if target:
            latest[target] = row
    diameters = [
        item for item in (_mm(row.get('d95_m')) for row in geometry)
        if item is not None]
    margins = [
        item for item in (_mm(row.get('radial_margin_m')) for row in geometry)
        if item is not None]
    reasons = {}
    allowed = {'true': 0, 'false': 0}
    for row in geometry:
        reason = str(row.get('reason') or '未写原因')
        reasons[reason] = reasons.get(reason, 0) + 1
        allowed['true' if row.get('allowed') else 'false'] += 1
    accepted, skipped, skip_codes = _frame_counts(roots)
    frames = _frame_index(root.resolve(), record, relative, roots)
    report = _read_json(record / 'report.json')
    rounds = []
    if isinstance(report, dict):
        for item in report.get('rounds') or []:
            if not isinstance(item, dict):
                continue
            rounds.append({
                'request_id': item.get('request_id'),
                'target_id': item.get('target_id'),
                'success': item.get('success'),
                'elapsed_s': item.get('elapsed_s'),
                'error': item.get('error'),
            })
    targets = []
    for target, row in latest.items():
        targets.append({
            'target_id': target,
            'allowed': row.get('allowed'),
            'reason': row.get('reason'),
            'ended': 'complete' if row.get('allowed') else 'recorded',
            'd95_mm': _mm(row.get('d95_m')),
            'radial_margin_mm': _mm(row.get('radial_margin_m')),
            'cut_to_fruit_m': row.get('cut_to_fruit_m'),
            'cut_distance_source': row.get('cut_distance_source'),
            'view_count': row.get('view_count'),
            'points': _points(row),
        })
    return {
        'path': relative,
        'outcomes': [
            {
                'batch': item.get('batch'),
                'target_id': item.get('target_id'),
                'outcome': item.get('outcome'),
                'reason': item.get('reason'),
                'elapsed_s': item.get('elapsed_s'),
                'completion_level': item.get('completion_level'),
                'failure_code': item.get('failure_code'),
            }
            for item in outcomes if isinstance(item, dict)
        ],
        'rounds': rounds,
        'rates': _rates(outcomes, accepted, skipped),
        'skip_codes': skip_codes,
        'fruits': _fruits(outcomes, frames, geometry),
        'force_cycles': _force_cycles(root.resolve(), record),
        'centers': _centers(geometry, outcomes),
        'geometry': {
            'fused_count': len(geometry),
            'allowed': allowed,
            'reasons': reasons,
            'd95_mm': _stats(diameters),
            'radial_margin_mm': _stats(margins),
            'targets': targets,
        },
        'notes': [
            '运动周期完成是套入后的撤离确认（完成度 6）。采摘确认是完成度 7，要切断证据和撤离证据都在。',
            '径向余量和果距只作记录，不代表当时的放行条件。',
            '彩色原图只留在记录目录，分析页只显示掩膜和检测调试图。',
            '每一行是一次周期。掩膜和检测调试图排在数字前面。',
        ],
    }


def read_record_image(root: Path, relative: str, name: str) -> bytes | None:
    """只读取这次分析列出的图片."""
    listed = _listed_names(analyze_record(root, relative))
    text = str(name or '').replace('\\', '/')
    if text not in listed or '..' in text.split('/'):
        return None
    candidates = []
    for candidate in (str(Path(relative) / text), text):
        try:
            candidates.append(_under(root, candidate))
        except ValueError:
            continue
    data = None
    for path in candidates:
        try:
            data = path.read_bytes()
        except OSError:
            continue
        if data:
            break
    if not data:
        return None
    if len(data) > _MAX_IMAGE_BYTES:
        return None
    return data
