"""记录写者的薄文件契约：显式会话身份与状态，禁止按 bag mtime 判活."""
from __future__ import annotations

import json
import os
from pathlib import Path
import time


def _process_start(pid: int) -> str:
    """Linux 进程起始 tick，用于拒绝 PID 被复用后的旧声明."""
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return ''


def write_session_state(session: Path, state: str, **fields) -> None:
    """原子更新写者状态；已有身份/开始时刻保留，状态切换不改文件归属."""
    session = Path(session)
    session.mkdir(parents=True, exist_ok=True)
    path = session / 'session.json'
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        payload = {}
    if not isinstance(payload, dict):
        payload = {}
    payload.update(fields)
    payload.update(session_id=session.name, state=state,
                   owner_pid=os.getpid(), owner_start=_process_start(os.getpid()),
                   updated_at_ns=time.time_ns())
    temporary = path.with_suffix('.json.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def active_session(root: Path) -> Path | None:
    """选择唯一仍由存活写者持有的 active 会话；多写者歧义时拒绝猜测."""
    candidates = []
    for path in Path(root).glob('session_*/session.json'):
        try:
            payload = json.loads(path.read_text(encoding='utf-8'))
            if payload.get('state') != 'active':
                continue
            pid = int(payload['owner_pid'])
            if pid <= 0:
                continue
            os.kill(pid, 0)
            if payload.get('owner_start') != _process_start(pid):
                continue
        except (OSError, ValueError, TypeError, KeyError, AttributeError):
            continue
        candidates.append(path.parent)
    return candidates[0] if len(candidates) == 1 else None
