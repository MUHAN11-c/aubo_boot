"""
调试操作审计：每次 Web 调试操作（含被拒）追加一行 JSONL.

落盘路径 ``<runs 根>/debug_audit/<YYYYMMDD>.jsonl``；runs 根与账本、
session、监控记录同一目录约定。线程安全（HTTP 线程并发写）；IO 失败
降级为日志告警，不影响操作本身的结果。
"""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import threading


class DebugAudit:
    """审计写入器（线程安全；enabled=false 或写失败时静默丢弃）."""

    def __init__(self, runs_root: str, enabled: bool, log_warning):
        """建审计器；runs_root 已解析，enabled=false 时 record() 静默丢弃."""
        self._dir = Path(runs_root) / 'debug_audit'
        self._enabled = bool(enabled)
        self._log_warning = log_warning
        self._lock = threading.Lock()
        self._warned = False

    def record(self, entry: dict) -> None:
        """
        追加一条审计记录（含被拒请求），自动补时间戳.

        Args:
            entry: 至少含 action/accepted/status.

        Returns
        -------
            无返回值（None）；IO 失败首次告警后静默.

        """
        if not self._enabled:
            return
        row = {'ts': datetime.now().isoformat(timespec='milliseconds'), **entry}
        line = json.dumps(row, ensure_ascii=False) + '\n'
        try:
            with self._lock:
                self._dir.mkdir(parents=True, exist_ok=True)
                path = self._dir / f'{datetime.now():%Y%m%d}.jsonl'
                with path.open('a', encoding='utf-8') as stream:
                    stream.write(line)
        except OSError as exc:
            if not self._warned:
                self._warned = True
                self._log_warning(f'调试审计写入失败（后续静默）: {exc}')
