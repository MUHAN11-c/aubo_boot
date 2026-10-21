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
            if (directory.name.startswith('session_')
                    and (directory / 'startup.json').is_file()):
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
    found.sort(key=lambda item: item['mtime'], reverse=True)
    return found[:limit]


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


def _related_roots(record: Path) -> list[Path]:
    """会话带上同一次启动的批次；runner 目录带上 report 里点名的 round."""
    roots = [record]
    roots.extend(_report_round_roots(record))
    if not record.name.startswith('session_') or record.parent is None:
        return roots
    start = _clock(record.name)
    if start is None:
        return roots
    end = start + timedelta(hours=6)
    try:
        siblings = list(record.parent.iterdir())
    except OSError:
        return roots
    for child in siblings:
        if not child.is_dir() or child.name.startswith('session_'):
            continue
        clock = _clock(child.name)
        if clock is None:
            continue
        if clock.year == 1900:
            clock = start.replace(
                hour=clock.hour, minute=clock.minute, second=clock.second,
                microsecond=0)
        if start <= clock < end:
            roots.append(child)
            roots.extend(_report_round_roots(child))
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


def analyze_record(root: Path, relative: str) -> dict:
    """解析一次测试目录里的账本、几何和图片."""
    record = _under(root, relative)
    if not record.is_dir():
        raise ValueError('记录目录不存在')
    relative = record.relative_to(root.resolve()).as_posix()
    roots = _related_roots(record)
    outcomes = _outcome_rows(roots)
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
