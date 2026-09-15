"""
bag 体积预算回收：纯核选择 + I/O 执行 + runs/retention_audit.jsonl 审计.

回收对象只有 bag 二进制目录（新会话 ``runs/session_*/bag`` 与旧 launch
录制的 ``runs/mcap_*``），按起始时间从旧到新删至预算内；bag_report、
ledger.json 与一切文本记录永不触碰（AGENTS 红线的预算化例外，回收必须
逐条写审计）。预算 <=0 视为禁用回收。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import time

# 回收对象 glob（相对 runs 根）；kind 标记进审计
_BAG_GLOBS = (('session_*/bag', 'session_bag'), ('mcap_*', 'legacy_mcap'))
AUDIT_FILENAME = 'retention_audit.jsonl'


@dataclass(frozen=True)
class BagEntry:
    """一个可回收 bag 目录：路径、字节数、起始时间、类别."""

    path: Path
    size_bytes: int
    mtime: float
    kind: str


def collect_bag_entries(runs_root) -> list[BagEntry]:
    """扫描 runs 根下的 bag 目录（缺失目录给空表）."""
    root = Path(runs_root)
    entries: list[BagEntry] = []
    if not root.is_dir():
        return entries
    for pattern, kind in _BAG_GLOBS:
        for path in sorted(root.glob(pattern)):
            if not path.is_dir():
                continue
            entries.append(BagEntry(
                path=path,
                size_bytes=directory_size(path),
                mtime=path.stat().st_mtime,
                kind=kind))
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

    keep（绝对路径集合）与预算 <=0 都给空表；keep 之外的目录仍参与
    总量统计——全保不下时只能删到只剩 keep。
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


def sweep(runs_root, max_total_bag_gb: float, keep=(),
          log_warning=lambda msg: None, audit_path=None,
          now: float | None = None) -> list[BagEntry]:
    """执行回收并追加审计；返回实际删除的条目（I/O 层，纯核在 select）."""
    budget_bytes = int(float(max_total_bag_gb) * (1024 ** 3))
    entries = collect_bag_entries(runs_root)
    doomed = select_bags_to_delete(entries, budget_bytes, keep=keep)
    if not doomed:
        return []
    audit = Path(audit_path) if audit_path else Path(runs_root) / AUDIT_FILENAME
    rows = []
    for entry in doomed:
        try:
            shutil.rmtree(entry.path)
        except OSError as error:
            log_warning(f'回收失败（跳过）{entry.path}: {error}')
            continue
        rows.append({
            'recorded_at': round(time.time() if now is None else now, 3),
            'deleted': str(entry.path),
            'kind': entry.kind,
            'size_bytes': entry.size_bytes,
            'budget_bytes': budget_bytes,
            'reason': 'bag_budget',
        })
        log_warning(
            f'bag 体积回收：删除 {entry.path.name}'
            f'（{entry.size_bytes / (1024 ** 3):.2f} GB）')
    if rows:
        try:
            audit.parent.mkdir(parents=True, exist_ok=True)
            with open(audit, 'a', encoding='utf-8') as stream:
                for row in rows:
                    stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        except OSError as error:
            log_warning(f'回收审计写入失败: {error}')
    return rows
