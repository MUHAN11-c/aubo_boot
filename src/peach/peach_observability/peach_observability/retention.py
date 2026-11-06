"""
bag 体积预算回收：纯核选择 + I/O 执行 + runs/retention_audit.jsonl 审计.

回收对象只有 bag 二进制目录（新会话 ``runs/session_*/bag`` 与旧 launch
录制的 ``runs/mcap_*``），按 bag 目录 mtime 从旧到新删至预算内；bag_report、
ledger.json 与一切文本记录永不触碰（AGENTS 红线的预算化例外，回收必须
逐条写审计）。预算 <=0 视为禁用回收。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import time
from uuid import uuid4

# 回收对象 glob（相对 runs 根）；kind 标记进审计
_BAG_GLOBS = (('session_*/bag', 'session_bag'), ('mcap_*', 'legacy_mcap'))
AUDIT_FILENAME = 'retention_audit.jsonl'


@dataclass(frozen=True)
class BagEntry:
    """一个候选 bag 目录：路径、字节数、目录 mtime、类别."""

    path: Path
    size_bytes: int
    mtime: float
    kind: str


def collect_bag_entries(runs_root) -> list[BagEntry]:
    """扫描 runs 根下的 bag 目录（缺失目录给空表）."""
    root = Path(runs_root).resolve()
    entries: list[BagEntry] = []
    if not root.is_dir():
        return entries
    for pattern, kind in _BAG_GLOBS:
        for path in sorted(root.glob(pattern)):
            if path.is_symlink() or path.parent.is_symlink() or not path.is_dir():
                continue
            try:
                mtime = path.stat().st_mtime
            except FileNotFoundError:
                continue  # Another retention pass has already removed this bag.
            entries.append(BagEntry(path, directory_size(path), mtime, kind))
    return entries


def directory_size(path) -> int:
    """目录递归字节数（不可读文件按 0 计，不抛）."""
    total = 0
    for item in Path(path).rglob('*'):
        try:
            if item.is_file():
                total += item.stat().st_size
        except OSError:
            continue
    return total


def select_bags_to_delete(entries: list[BagEntry], budget_bytes: int,
                          keep=()) -> list[BagEntry]:
    """
    纯核：超预算时从旧到新选 bag 目录直至总量回落预算内.

    预算 <=0 给空表；keep 中的目录计入总量但不参与删除。
    若受保护目录已超过预算，只能删尽其它候选，不能保证回落预算内。
    """
    if budget_bytes <= 0:
        return []
    kept = {Path(item).resolve() for item in keep}
    total = sum(entry.size_bytes for entry in entries)
    if total <= budget_bytes:
        return []
    candidates = sorted(
        (entry for entry in entries
         if entry.path.resolve() not in kept),
        key=lambda entry: (entry.mtime, str(entry.path)))
    chosen: list[BagEntry] = []
    for entry in candidates:
        if total <= budget_bytes:
            break
        chosen.append(entry)
        total -= entry.size_bytes
    return chosen


def _closed_bag(bag: Path) -> bool:
    """Only retire sealed sessions or legacy bags finalized by rosbag2."""
    if bag.name == 'bag':
        try:
            state = json.loads((bag.parent / 'session.json').read_text(encoding='utf-8'))
        except FileNotFoundError:
            pass  # Older recorders only have rosbag2 metadata, written on close.
        except (OSError, ValueError):
            return False
        else:
            return isinstance(state, dict) and state.get('state') == 'sealed'
    return (bag / 'metadata.yaml').is_file()


def _append_audit(audit: Path, row: dict, log_warning) -> bool:
    """追加并刷盘审计；删除意图写入失败时调用方必须保留 bag."""
    try:
        audit.parent.mkdir(parents=True, exist_ok=True)
        with audit.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        return True
    except OSError as error:
        log_warning(f'回收审计写入失败: {error}')
        return False


def _remove_bag(entry: BagEntry, audit: Path, row: dict, log_warning) -> bool:
    """先持久记录意图再删除；同一 operation_id 关联实际成功或失败."""
    intent = {**row, 'operation_id': uuid4().hex, 'phase': 'intent'}
    if not _append_audit(audit, intent, log_warning):
        log_warning(f'审计不可用，保留 bag：{entry.path}')
        return False
    try:
        shutil.rmtree(entry.path)
    except OSError as error:
        _append_audit(
            audit, {**intent, 'phase': 'delete_failed', 'error': str(error)}, log_warning)
        log_warning(f'回收失败（跳过）{entry.path}: {error}')
        return False
    _append_audit(audit, {**intent, 'phase': 'deleted'}, log_warning)
    return True


def sweep(runs_root, max_total_bag_gb: float, keep=(),
          log_warning=lambda msg: None, audit_path=None,
          now: float | None = None) -> list[dict]:
    """执行回收并追加审计；返回实际删除的条目（I/O 层，纯核在 select）."""
    budget_bytes = int(float(max_total_bag_gb) * (1024 ** 3))
    if budget_bytes <= 0:
        return []
    entries = collect_bag_entries(runs_root)
    protected = set(keep)
    protected.update(entry.path for entry in entries if not _closed_bag(entry.path))
    doomed = select_bags_to_delete(entries, budget_bytes, keep=protected)
    if not doomed:
        return []
    audit = Path(audit_path) if audit_path else Path(runs_root) / AUDIT_FILENAME
    rows = []
    for entry in doomed:
        if not _closed_bag(entry.path):
            continue
        row = {
            'recorded_at': round(time.time() if now is None else now, 3),
            'deleted': str(entry.path),
            'kind': entry.kind,
            'size_bytes': entry.size_bytes,
            'budget_bytes': budget_bytes,
            'reason': 'bag_budget',
        }
        if not _remove_bag(entry, audit, row, log_warning):
            continue
        rows.append(row)
        log_warning(
            f'bag 体积回收：删除 {entry.path.name}'
            f'（{entry.size_bytes / (1024 ** 3):.2f} GB）')
    return rows


def purge_analyzed_sessions(runs_root, sessions, log_warning=lambda msg: None,
                            audit_path=None, now: float | None = None):
    """
    分析完成后删除指定会话的 bag/ 二进制，保留 bag_report 与账本.

    sessions：session 目录或 bag 目录路径列表。只删名为 bag 的子目录；
    同级 bag_report.md/json、ledger 与其它文本不动。审计 reason=analyzed_purge。
    """
    deleted = []
    root = Path(runs_root).resolve()
    stamp = round(time.time() if now is None else now, 3)
    audit = Path(audit_path) if audit_path else root / AUDIT_FILENAME
    for item in sessions:
        path = Path(item)
        bag = path if path.name == 'bag' else path / 'bag'
        if not bag.is_dir():
            continue
        if bag.is_symlink() or root not in bag.resolve().parents or not _closed_bag(bag):
            log_warning(f'跳过未封口或记录根目录外的 bag：{bag}')
            continue
        size = directory_size(bag)
        row = {
            'recorded_at': stamp,
            'deleted': str(bag),
            'kind': 'session_bag',
            'size_bytes': size,
            'budget_bytes': 0,
            'reason': 'analyzed_purge',
        }
        entry = BagEntry(bag, size, stamp, 'session_bag')
        if not _remove_bag(entry, audit, row, log_warning):
            continue
        deleted.append(bag)
        log_warning(f'分析后清理 bag：{bag}')
    return deleted
