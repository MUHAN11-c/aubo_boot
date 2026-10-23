#!/usr/bin/env python3
"""
力传感器实时调试 Web.

订 /force/sample 原子完整帧（N/mm/AD/status、源时间和序号），SSE 推给浏览器；
并提供 20 s 记录按钮。接触段自动记录：力离开空载才开始写，回到空载并稳住后封口。
两种 CSV 都写进正在录的采摘会话 ``runs/session_*/force/``，不另开目录。

用法（先 source Jazzy 再 source 工作区）：
  python3 src/serial_force/scripts/force_web.py [--port 8099]

串口由 serial_force_node 持有，本服务只订话题、不碰 /dev/force。
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlparse
import uuid

from peach_common.paths import runs_root
from peach_interfaces.msg import ForceSample
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from serial_force.force_episode import (
    ClosedEpisode, ForceEpisodeTracker, live_peach_session,
    next_episode_index, peak_level)

RUNS = runs_root()
HTML_PATH = Path(__file__).with_name('force_web.html')
FACE_PATH = Path(__file__).with_name('face_fruit.png')
FACE_FULL_PATH = Path(__file__).with_name('face_full.png')
ASSETS = {'/face.png': FACE_PATH, '/face_full.png': FACE_FULL_PATH}

CH = (1, 2, 3, 5, 7)
# 五点 tcp 系坐标 (x, y) mm，来源 doc/force_pad_positions.md
PADS_MM = {1: (15.75, 4.30), 2: (9.45, -13.30), 3: (-9.65, -13.15),
           5: (0.0, 16.45), 7: (-15.70, 4.55)}
_lock = threading.RLock()
_flush_lock = threading.Lock()
_stop = threading.Event()
_stamps: deque[float] = deque(maxlen=300)
_state = {'values': None, 'travel': None, 'ad': None, 'status': None,
          'frames': 0, 'last_mono': None, 'rate': 0.0,
          'stamp_ns': 0, 'receive_unix_ns': 0,
          'source_uuid': '', 'sequence': 0, 'sequence_gaps': 0}
_rec = {'active': False, 'start': 0.0, 'seconds': 20, 'rows': [],
        'frames': 0, 'file': None, 'count': 0}
_episode = ForceEpisodeTracker()
_episode_session: Path | None = None
_episode_partial: Path | None = None
_pending: list = []
_manual_pending: list = []
_ep = {'count': 0, 'last_contact_s': None, 'last_duration_s': None,
       'last_path': None, 'last_reason': None, 'session': None}
_last_fsync = 0.0
STREAM_TIMEOUT_S = 0.5


class _ForceListener(Node):
    """完整帧是唯一数值来源；不同话题的到达顺序不用于拼帧."""

    def __init__(self) -> None:
        super().__init__('force_web')
        self.create_subscription(ForceSample, '/force/sample', self._cb_sample,
                                 QoSProfile(depth=100,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.1, self._watchdog)

    def _cb_sample(self, msg: ForceSample) -> None:
        if [int(value) for value in msg.channel_ids] != list(CH):
            self.get_logger().warning('忽略通道顺序不匹配的力帧')
            return
        now = time.monotonic()
        with _lock:
            previous_source = _state['source_uuid']
            previous_seq = _state['sequence']
            if previous_source and previous_source != msg.source_uuid:
                _finish_episode('source_changed')
                _finish_record_locked('source_changed')
            elif previous_source == msg.source_uuid and previous_seq:
                if msg.sequence <= previous_seq:
                    return
                _state['sequence_gaps'] += max(0, msg.sequence - previous_seq - 1)
            _state.update(values=[float(v) for v in msg.force_n],
                          travel=[float(v) for v in msg.travel_mm],
                          ad=[int(v) for v in msg.ad],
                          status=[int(v) for v in msg.status],
                          stamp_ns=(msg.header.stamp.sec * 1_000_000_000
                                    + msg.header.stamp.nanosec),
                          receive_unix_ns=time.time_ns(),
                          source_uuid=str(msg.source_uuid), sequence=int(msg.sequence))
            _state['frames'] += 1
            _state['last_mono'] = now
            _stamps.append(now)
            while _stamps and _stamps[0] < now - 1.0:
                _stamps.popleft()
            _state['rate'] = float(len(_stamps))
        try:
            self._record_maybe()
            self._episode_maybe()
        except OSError as error:
            self.get_logger().error(f'力记录写入失败，保留partial: {error}')

    def _watchdog(self) -> None:
        """没有下一帧时也封口，且已经绑定的段绝不挪到新会话."""
        with _lock:
            last = _state['last_mono']
            reason = None
            if last is not None and time.monotonic() - last >= STREAM_TIMEOUT_S:
                reason = 'stream_lost'
            session = live_peach_session(RUNS)
            if _episode_session is not None and session != _episode_session:
                reason = reason or 'session_closed'
            if reason:
                _finish_episode(reason)
            if _rec['active'] and (reason or session != _rec.get('session')):
                _finish_record_locked(reason or 'session_closed')
        _flush_pending()

    def _record_maybe(self) -> None:
        """每个完整帧写一行；起始时已经绑定会话."""
        with _lock:
            if not _rec['active']:
                return
            if (_state['values'] is None or _state['travel'] is None
                    or _state['ad'] is None or _state['status'] is None):
                return
            now = time.monotonic()
            elapsed = now - _rec['start']
            row = _sample_row(round(elapsed, 4))
            _rec['rows'].append(row)
            _journal_row(_rec['partial'], row)
            _rec['count'] = len(_rec['rows'])
            if elapsed >= _rec['seconds']:
                _finish_record_locked()
        _flush_pending()

    def _episode_maybe(self) -> None:
        """力离开空载才记，归零稳住后写成一段 CSV."""
        global _episode_session, _episode_partial
        with _lock:
            if (_state['values'] is None or _state['travel'] is None
                    or _state['ad'] is None or _state['status'] is None):
                return
            now = time.monotonic()
            # A failed channel is not evidence of returning to zero. Preserve
            # raw values in the CSV, close with an explicit data-quality reason.
            if any(st not in (0, 1) for st in _state['status']):
                _finish_episode('invalid_data')
                return
            level = peak_level(_state['values'])
            if _episode.phase == 'idle':
                if level < _episode.onset_n:
                    return
                session = live_peach_session(RUNS)
                if session is None:
                    return
                _episode_session = session
                _episode_partial = _new_journal(session, 'episode')
            row = _sample_row(now)
            _journal_row(_episode_partial, row)
            closed = _episode.update(now, level, row)
            if closed is not None:
                _queue_episode(closed)
        _flush_pending()


def _sample_row(t: float) -> list:
    return ([t] + list(_state['values']) + list(_state['travel'])
            + list(_state['ad']) + list(_state['status'])
            + [_state['frames'], round(_state['rate'], 1),
               _state['stamp_ns'], _state['receive_unix_ns'],
               _state['source_uuid'], _state['sequence']])


def _new_journal(session: Path, kind: str) -> Path:
    directory = session / 'force'
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f'.{kind}_{uuid.uuid4().hex}.partial.jsonl'
    _journal_row(path, {'kind': kind, 'session_id': session.name})
    return path


def _journal_row(path: Path, row) -> None:
    """逐帧append且退出文件缓冲；每0.5s落设备，崩溃留下可恢复partial."""
    global _last_fsync
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        now = time.monotonic()
        if now - _last_fsync >= 0.5:
            stream.flush()
            os.fsync(stream.fileno())
            _last_fsync = now


def _queue_episode(closed) -> None:
    global _episode_session, _episode_partial
    if _episode_session is not None:
        _pending.append((_episode_session, closed, time.time(),
                         time.monotonic(), _episode_partial))
    _episode_session = _episode_partial = None


def _finish_episode(reason: str) -> None:
    with _lock:
        closed = _episode.finish(reason)
        if closed is not None:
            _queue_episode(closed)


def _csv_header() -> list:
    return (['t_s'] + [f'f{c}' for c in CH] + [f'tr{c}' for c in CH]
            + [f'ad{c}' for c in CH] + [f'st{c}' for c in CH]
            + ['frames', 'hz', 'stamp_ns', 'receive_unix_ns',
               'source_uuid', 'sequence'])


def _flush_pending() -> None:
    """只写起始时绑定的目录，成功后再销账；失败保留partial及待写任务."""
    with _flush_lock:
        try:
            while _pending:
                session, closed, wall_end, mono_end, partial = _pending[0]
                _write_episode(session, closed, wall_end, mono_end,
                               partial.name if partial else '')
                if partial is not None:
                    partial.unlink(missing_ok=True)
                _pending.pop(0)
            while _manual_pending:
                job = _manual_pending[0]
                _write_manual(job)
                job['partial'].unlink(missing_ok=True)
                _manual_pending.pop(0)
        except OSError as error:
            print(f'force 记录写入失败，保留partial供恢复: {error}', flush=True)


def _format_wall(epoch: float) -> str:
    """本地时间保留毫秒，避免显示起点偏到整秒."""
    whole = int(math.floor(epoch))
    millis = int(round((epoch - whole) * 1000))
    if millis >= 1000:
        whole += 1
        millis -= 1000
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(whole)) + f'.{millis:03d}'


def _write_episode(session: Path, closed, wall_end: float,
                   mono_end: float, journal_id: str = '') -> None:
    """一段接触写入 ``<session>/force/``."""
    wall_start = wall_end - (mono_end - closed.t0)
    if closed.rows and len(closed.rows[0]) >= 27:
        wall_start = int(closed.rows[0][-3]) / 1e9
        wall_end = int(closed.rows[-1][-3]) / 1e9
    force_dir = session / 'force'
    force_dir.mkdir(parents=True, exist_ok=True)
    if journal_id:
        for existing in force_dir.glob('episode_*.json'):
            try:
                previous = json.loads(existing.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                continue
            if (previous.get('journal_id') == journal_id
                    and existing.with_suffix('.csv').is_file()):
                return
    index = next_episode_index(force_dir)
    rel_rows = []
    peaks = [0.0] * 5
    for row in closed.rows:
        rel = round(row[0] - closed.t0, 4)
        rel_rows.append([rel] + row[1:])
        for i, value in enumerate(row[1:6]):
            if isinstance(value, (int, float)) and math.isfinite(value):
                peaks[i] = max(peaks[i], abs(float(value)))
    name = f'episode_{index:03d}'
    csv_path = force_dir / f'{name}.csv'
    temporary = csv_path.with_suffix('.csv.tmp')
    with open(temporary, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(_csv_header())
        writer.writerows(rel_rows)
        fh.flush()
        os.fsync(fh.fileno())
    temporary.replace(csv_path)
    meta = {
        'episode': index,
        'reason': closed.reason,
        'session': str(session),
        'session_id': session.name,
        'journal_id': journal_id,
        'time_basis': 'host_receive_clock',
        'start_unix_ns': int(closed.rows[0][-3]),
        'end_unix_ns': int(closed.rows[-1][-3]),
        'start_ros_ns': int(closed.rows[0][-4]),
        'end_ros_ns': int(closed.rows[-1][-4]),
        'source_uuid': str(closed.rows[0][-2]),
        'first_sequence': int(closed.rows[0][-1]),
        'last_sequence': int(closed.rows[-1][-1]),
        'wall_start': _format_wall(wall_start),
        'wall_end': _format_wall(wall_end),
        'duration_s': round(closed.duration_s, 3),
        'contact_s': round(closed.contact_s, 3),
        'samples': len(rel_rows),
        'peak_n': [round(v, 4) for v in peaks],
        'csv': str(csv_path),
    }
    meta_path = force_dir / f'{name}.json'
    meta_tmp = meta_path.with_suffix('.json.tmp')
    meta_tmp.write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    meta_tmp.replace(meta_path)
    with open(force_dir / 'index.jsonl', 'a', encoding='utf-8') as fh:
        fh.write(json.dumps(meta, ensure_ascii=False) + '\n')
        fh.flush()
        os.fsync(fh.fileno())
    with _lock:
        _ep.update(count=index, last_contact_s=meta['contact_s'],
                   last_duration_s=meta['duration_s'],
                   last_path=str(csv_path), last_reason=closed.reason,
                   session=session.name)
    print(
        f'force episode {index} contact={meta["contact_s"]:.3f}s '
        f'duration={meta["duration_s"]:.3f}s reason={closed.reason} '
        f'{csv_path}',
        flush=True)


def _finish_record_locked(reason: str = 'duration') -> None:
    """收口手动段；目录在开始时绑定，退出/断流同样保留已收帧."""
    if not _rec['active']:
        return
    _rec['active'] = False
    if _rec['rows']:
        _manual_pending.append({
            'session': _rec['session'], 'rows': list(_rec['rows']),
            'reason': reason, 'partial': _rec['partial']})
    else:
        _rec['partial'].unlink(missing_ok=True)


def _write_manual(job: dict) -> None:
    """手动 20 s CSV 同样放进当前会话的 force/."""
    rows = job['rows']
    session = job['session']
    force_dir = session / 'force'
    force_dir.mkdir(parents=True, exist_ok=True)
    stamp = int(rows[0][-3])
    identity = job['partial'].name.removeprefix('.manual_').removesuffix('.partial.jsonl')
    path = force_dir / f'force_20s_{stamp}_{identity}.csv'
    temporary = path.with_suffix('.csv.tmp')
    with open(temporary, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(_csv_header())
        writer.writerows(rows)
        fh.flush()
        os.fsync(fh.fileno())
    temporary.replace(path)
    meta = {'session_id': session.name, 'reason': job['reason'],
            'time_basis': 'host_receive_clock',
            'start_unix_ns': int(rows[0][-3]), 'end_unix_ns': int(rows[-1][-3]),
            'start_ros_ns': int(rows[0][-4]), 'end_ros_ns': int(rows[-1][-4]),
            'source_uuid': str(rows[0][-2]), 'samples': len(rows)}
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(meta, ensure_ascii=False) + '\n', encoding='utf-8')
    temporary.replace(path.with_suffix('.json'))
    with _lock:
        _rec['file'] = path
        _ep['session'] = session.name
    print(f'record saved {path} rows={len(rows)}', flush=True)


def _start_record(seconds: int) -> dict:
    with _lock:
        if _rec['active']:
            return {'ok': False, 'error': 'already recording'}
        session = live_peach_session(RUNS)
        if session is None:
            return {'ok': False, 'error': 'no unique active recording session'}
        if _manual_pending:
            return {'ok': False, 'error': 'previous recording persistence pending'}
        last = _state['last_mono']
        if last is None or time.monotonic() - last >= STREAM_TIMEOUT_S:
            return {'ok': False, 'error': 'force stream unavailable'}
        partial = _new_journal(session, 'manual')
        _rec.update(active=True, start=time.monotonic(),
                    seconds=max(1, min(120, int(seconds))), rows=[],
                    frames=0, file=None, count=0, session=session,
                    partial=partial)
        return {'ok': True, 'seconds': _rec['seconds']}


def _recover_partials() -> None:
    """进程异常退出留下的日志只恢复进其原目录，不重绑定下一次会话."""
    for path in RUNS.glob('session_*/force/.*.partial.jsonl'):
        try:
            lines = path.read_text(encoding='utf-8').splitlines()
            header = json.loads(lines[0])
            rows = []
            for line in lines[1:]:
                try:
                    row = json.loads(line)
                except ValueError:
                    break  # a killed append may leave only its last row partial
                if isinstance(row, list) and len(row) == len(_csv_header()):
                    rows.append(row)
            session = path.parent.parent
            if header.get('session_id') != session.name:
                continue
            if not rows:
                path.unlink(missing_ok=True)
                continue
            if header.get('kind') == 'manual':
                _manual_pending.append({
                    'session': session, 'rows': rows,
                    'reason': 'recovered_partial', 'partial': path})
            else:
                hot = [row[0] for row in rows
                       if peak_level(row[1:6]) >= _episode.release_n]
                closed = ClosedEpisode(
                    'recovered_partial', rows[0][0], rows[-1][0],
                    hot[-1] if hot else rows[0][0], rows)
                _pending.append((session, closed, int(rows[-1][-3]) / 1e9,
                                 rows[-1][0], path))
        except (OSError, ValueError, IndexError, TypeError) as error:
            print(f'force partial保留，无法恢复 {path}: {error}', flush=True)
    _flush_pending()


def _record_status() -> dict:
    with _lock:
        elapsed = (time.monotonic() - _rec['start']) if _rec['active'] else 0.0
        return {
            'active': _rec['active'],
            'seconds': _rec['seconds'],
            'elapsed': round(min(elapsed, _rec['seconds']), 1),
            'count': _rec['count'],
            'file': '/record/download' if _rec['file'] else None,
            'path': str(_rec['file']) if _rec['file'] else None,
        }


def _cop(vals, statuses=None):
    """五点力到 CoP（tcp 系 mm）与总力（N）；无效通道不参与."""
    if not vals:
        return None, 0.0
    tot = sx = sy = 0.0
    for index, (ch, v) in enumerate(zip(CH, vals)):
        if statuses is not None and statuses[index] not in (0, 1):
            continue
        if v is None or not math.isfinite(v):
            continue
        f = max(0.0, float(v))
        x, y = PADS_MM[ch]
        tot += f
        sx += f * x
        sy += f * y
    if tot < 0.05:  # 空载/1–4 g 噪声：CoP 无意义
        return None, tot
    return [sx / tot, sy / tot], tot


def _clean(vals):
    if vals is None:
        return None
    return [v if math.isfinite(v) else None for v in vals]


def _snapshot() -> dict:
    with _lock:
        last = _state['last_mono']
        cop, total = _cop(_state['values'], _state['status'])
        return {
            'v': _clean(_state['values']),
            'cop': None if cop is None else [round(cop[0], 3), round(cop[1], 3)],
            'cop_n': None if cop is None else round(
                math.hypot(cop[0], cop[1]), 3),
            'total': round(total, 4),
            't': _clean(_state['travel']),
            'a': _clean(_state['ad']),
            's': _state['status'],
            'frames': _state['frames'],
            'source_uuid': _state['source_uuid'],
            'sequence': _state['sequence'],
            'sequence_gaps': _state['sequence_gaps'],
            'hz': round(_state['rate'], 1),
            'age_ms': None if last is None else round(
                (time.monotonic() - last) * 1000),
            'episode': _episode_view_locked(time.monotonic()),
        }


def _episode_file(session: str, name: str) -> Path | None:
    """只允许读 runs/session_*/force/episode_NNN.csv."""
    if not session.startswith('session_') or '/' in session or '..' in session:
        return None
    if not name.startswith('episode_') or not name[8:].isdigit():
        return None
    path = (RUNS / session / 'force' / f'{name}.csv').resolve()
    try:
        path.relative_to(RUNS.resolve())
    except ValueError:
        return None
    return path if path.is_file() else None


def _episode_catalog() -> list:
    """最近的接触段，新的在前."""
    if not RUNS.is_dir():
        return []
    files = sorted(
        RUNS.glob('session_*/force/episode_*.csv'),
        key=lambda item: item.stat().st_mtime,
        reverse=True)
    rows = []
    for csv_path in files[:40]:
        meta = {}
        side = csv_path.with_suffix('.json')
        if side.is_file():
            try:
                meta = json.loads(side.read_text(encoding='utf-8'))
            except (OSError, json.JSONDecodeError):
                meta = {}
        rows.append({
            'session': csv_path.parent.parent.name,
            'name': csv_path.stem,
            'contact_s': meta.get('contact_s'),
            'duration_s': meta.get('duration_s'),
            'samples': meta.get('samples'),
            'wall_start': meta.get('wall_start'),
            'reason': meta.get('reason'),
        })
    return rows


def _episode_samples(session: str, name: str) -> dict | None:
    """读一段 CSV，抽到最多 800 点给回放."""
    path = _episode_file(session, name)
    if path is None:
        return None
    samples = []
    with path.open(newline='', encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            try:
                forces = [float(row[f'f{c}']) for c in CH]
                statuses = [int(row.get(f'st{c}', 0)) for c in CH]
                stamp = float(row['t_s'])
            except (KeyError, TypeError, ValueError):
                continue
            cop, _total = _cop(forces, statuses)
            samples.append({
                't': stamp,
                'v': forces,
                's': statuses,
                'cop': None if cop is None else [
                    round(cop[0], 3), round(cop[1], 3)],
            })
    if len(samples) > 800:
        step = len(samples) / 800
        samples = [samples[min(len(samples) - 1, int(i * step))]
                   for i in range(800)]
    duration = samples[-1]['t'] - samples[0]['t'] if len(samples) > 1 else 0.0
    return {
        'session': session,
        'name': name,
        'duration_s': round(max(duration, 0.0), 3),
        'samples': samples,
    }


def _episode_view_locked(now: float) -> dict:
    """调用者已持 _lock。给网页的接触段状态."""
    elapsed = 0.0
    if _episode.phase == 'recording' and _episode.t0 is not None:
        elapsed = now - _episode.t0
    return {
        'phase': _episode.phase,
        'elapsed_s': round(elapsed, 2),
        'count': _ep['count'],
        'last_contact_s': _ep['last_contact_s'],
        'last_duration_s': _ep['last_duration_s'],
        'last_reason': _ep['last_reason'],
        'session': _ep['session'],
    }


class _Handler(BaseHTTPRequestHandler):
    def _json(self, obj: dict, code: int = 200) -> None:
        body = json.dumps(obj, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _csv(self) -> None:
        with _lock:
            path = _rec['file']
        if not path or not path.exists():
            self.send_error(404)
            return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', 'text/csv; charset=utf-8')
        self.send_header(
            'Content-Disposition',
            'attachment; filename="force_20s.csv"')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == '/record/start':
            q = parse_qs(parsed.query)
            try:
                seconds = int(q.get('seconds', ['20'])[0])
            except ValueError:
                seconds = 20
            self._json(_start_record(seconds))
        elif parsed.path == '/record/status':
            self._json(_record_status())
        elif parsed.path == '/record/download':
            self._csv()
        elif parsed.path == '/episodes':
            self._json({'episodes': _episode_catalog()})
        elif parsed.path == '/episodes/data':
            q = parse_qs(parsed.query)
            session = q.get('session', [''])[0]
            name = q.get('name', [''])[0]
            data = _episode_samples(session, name)
            if data is None:
                self._json({'ok': False, 'error': '没有这段记录'}, 404)
            else:
                self._json(data)
        elif parsed.path == '/':
            body = HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        elif parsed.path in ASSETS:
            try:
                body = ASSETS[parsed.path].read_bytes()
            except OSError:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        elif parsed.path.startswith('/stream'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                while not _stop.is_set():
                    payload = json.dumps(
                        _snapshot(), ensure_ascii=False, allow_nan=False)
                    self.wfile.write(f'data: {payload}\n\n'.encode())
                    self.wfile.flush()
                    _stop.wait(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *args) -> None:
        pass


class _Server(ThreadingHTTPServer):
    """SSE线程受停止事件控制，客户端长连接不阻止进程收尾."""

    daemon_threads = True


def main() -> None:
    """订力话题并起本机调试页."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8099)
    ap.add_argument('--runs-root', default='',
                    help='过程数据根；空=peach_common.paths.runs_root统一解析')
    args = ap.parse_args()
    global RUNS
    RUNS = runs_root(args.runs_root).resolve()
    RUNS.mkdir(parents=True, exist_ok=True)
    # One evidence writer per root. Two web processes must never reserve the
    # same episode number, recover each other's journal, or overwrite a CSV.
    with (RUNS / '.force_web.lock').open('a') as owner:
        try:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(f'force_web already owns {RUNS}') from error
        _recover_partials()
        server = _Server(('127.0.0.1', args.port), _Handler)
        node = executor = spin_thread = None
        try:
            rclpy.init()
            node = _ForceListener()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            def spin():
                try:
                    executor.spin()
                except ExternalShutdownException:
                    pass

            spin_thread = threading.Thread(target=spin, daemon=True)
            spin_thread.start()
            print(f'force web: http://127.0.0.1:{args.port} '
                  f'(sub /force/sample, runs={RUNS})', flush=True)
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            _stop.set()
            if executor is not None:
                executor.shutdown(timeout_sec=2.0)
            if spin_thread is not None:
                spin_thread.join(timeout=2.0)
            with _lock:
                _finish_episode('shutdown')
                _finish_record_locked('shutdown')
            _flush_pending()
            server.server_close()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
