"""离线读取 runs 下的测试记录。只读，不改监控状态."""
from __future__ import annotations

import json
from pathlib import Path
from urllib.parse import quote

_MARKERS = ('ledger.json', 'geometry.jsonl', 'report.json')
_IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.webp'}
_SKIP_DIRS = {'bag', 'ros_home', 'ros_logs'}
_MAX_LINES = 4000
_MAX_IMAGES = 12
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
    candidates = [root]
    candidates.extend(path for path in root.iterdir() if path.is_dir())
    for directory in list(candidates):
        if directory == root:
            continue
        try:
            candidates.extend(
                child for child in directory.iterdir()
                if child.is_dir() and child.name not in _SKIP_DIRS)
        except OSError:
            continue
    seen = set()
    for directory in candidates:
        if directory in seen or directory == root:
            continue
        seen.add(directory)
        try:
            markers = [name for name in _MARKERS if (directory / name).is_file()]
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


def _images(record: Path, relative: str) -> list[dict]:
    found = []
    pending = [record]
    while pending and len(found) < _MAX_IMAGES:
        current = pending.pop()
        try:
            children = list(current.iterdir())
        except OSError:
            continue
        for path in children:
            if path.is_dir():
                if (path.name not in _SKIP_DIRS
                        and len(path.relative_to(record).parts) <= 3):
                    pending.append(path)
                continue
            if len(found) >= _MAX_IMAGES:
                break
            if path.suffix.lower() not in _IMAGE_SUFFIXES:
                continue
            try:
                size = path.stat().st_size
            except OSError:
                continue
            if size <= 0 or size > _MAX_IMAGE_BYTES:
                continue
            name = path.relative_to(record).as_posix()
            found.append({
                'name': name,
                'url': (
                    '/api/analysis/image?path=' + quote(relative)
                    + '&name=' + quote(name)),
            })
    return found


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


def analyze_record(root: Path, relative: str) -> dict:
    """解析一次测试目录里的账本、几何和图片."""
    record = _under(root, relative)
    if not record.is_dir():
        raise ValueError('记录目录不存在')
    relative = record.relative_to(root.resolve()).as_posix()
    ledger = _read_json(record / 'ledger.json') or {}
    outcomes = ledger.get('outcomes') if isinstance(ledger, dict) else None
    if not isinstance(outcomes, list):
        outcomes = []
    geometry = _geometry_rows(record / 'geometry.jsonl')
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
        'geometry': {
            'fused_count': len(geometry),
            'allowed': allowed,
            'reasons': reasons,
            'd95_mm': _stats(diameters),
            'radial_margin_mm': _stats(margins),
            'targets': targets,
        },
        'images': _images(record, relative),
        'notes': [
            '径向余量和果距只作记录，不代表当时的放行条件。',
            '识别图若已保存在该目录，会显示在参考点旁边；图上的黄箭头是袋底到袋口，紫圈是剪切参考。',
        ],
    }


def read_record_image(root: Path, relative: str, name: str) -> bytes | None:
    """只读取这次分析列出的图片."""
    listed = {
        item['name'] for item in analyze_record(root, relative)['images']}
    text = str(name or '').replace('\\', '/')
    if text not in listed:
        return None
    path = _under(root, str(Path(relative) / text))
    try:
        data = path.read_bytes()
    except OSError:
        return None
    if len(data) > _MAX_IMAGE_BYTES:
        return None
    return data
