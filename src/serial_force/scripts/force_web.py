#!/usr/bin/env python3
"""
力传感器实时调试 Web.

订 /force/points（N）、/force/travel（mm）、/force/ad（原始 AD）、
/force/status（每通道 OK/SAT/OPEN/INVALID），SSE 推给浏览器；
并提供 20 s 记录按钮。接触段自动记录：力离开空载才开始写，回到空载并稳住后封口。
两种 CSV 都写进正在录的采摘会话 ``runs/session_*/force/``，不另开目录。

用法（先 source Jazzy 再 source 工作区）：
  python3 src/serial_force/scripts/force_web.py [--port 8099]

串口由 serial_force_node 持有，本服务只订话题、不碰 /dev/force。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node

from serial_force.force_episode import (
    ForceEpisodeTracker, live_peach_session, next_episode_index, peak_level)

from std_msgs.msg import Float64MultiArray, UInt8MultiArray

ROOT = Path(__file__).resolve().parents[3]
RUNS = ROOT / 'runs'
HTML_PATH = Path(__file__).with_name('force_web.html')
FACE_PATH = Path(__file__).with_name('face_fruit.png')
FACE_FULL_PATH = Path(__file__).with_name('face_full.png')
ASSETS = {'/face.png': FACE_PATH, '/face_full.png': FACE_FULL_PATH}

CH = (1, 2, 3, 5, 7)
# 五点 tcp 系坐标 (x, y) mm，来源 doc/force_pad_positions.md
PADS_MM = {1: (15.75, 4.30), 2: (9.45, -13.30), 3: (-9.65, -13.15),
           5: (0.0, 16.45), 7: (-15.70, 4.55)}
_lock = threading.Lock()
_stamps: deque[float] = deque(maxlen=300)
_state = {'values': None, 'travel': None, 'ad': None, 'status': None,
          'frames': 0, 'last_mono': None, 'rate': 0.0}
_rec = {'active': False, 'start': 0.0, 'seconds': 20, 'rows': [],
        'frames': 0, 'file': None, 'count': 0}
_episode = ForceEpisodeTracker()
_pending: list = []
_ep = {'count': 0, 'last_contact_s': None, 'last_duration_s': None,
       'last_path': None, 'last_reason': None, 'session': None}
_last_flush = 0.0


class _ForceListener(Node):
    """订阅 4 个力话题；status 回调里顺带记录 20 s CSV."""

    def __init__(self) -> None:
        super().__init__('force_web')
        self.create_subscription(
            Float64MultiArray, '/force/points', self._cb_points, 10)
        self.create_subscription(
            Float64MultiArray, '/force/travel', self._cb_travel, 10)
        self.create_subscription(
            Float64MultiArray, '/force/ad', self._cb_ad, 10)
        self.create_subscription(
            UInt8MultiArray, '/force/status', self._cb_status, 10)

    def _cb_travel(self, msg: Float64MultiArray) -> None:
        with _lock:
            if len(msg.data) == 5:
                _state['travel'] = list(msg.data)

    def _cb_ad(self, msg: Float64MultiArray) -> None:
        with _lock:
            if len(msg.data) == 5:
                _state['ad'] = list(msg.data)

    def _cb_points(self, msg: Float64MultiArray) -> None:
        now = time.monotonic()
        with _lock:
            if len(msg.data) == 5:
                _state['values'] = list(msg.data)
            _state['frames'] += 1
            _state['last_mono'] = now
            _stamps.append(now)
            while _stamps and _stamps[0] < now - 1.0:
                _stamps.popleft()
            _state['rate'] = float(len(_stamps))

    def _cb_status(self, msg: UInt8MultiArray) -> None:
        with _lock:
            if len(msg.data) == 5:
                _state['status'] = [int(v) for v in msg.data]
        self._record_maybe()
        self._episode_maybe()

    def _record_maybe(self) -> None:
        """Status 是节点最后一个话题；在这里落一行，保证 f/t/a/s 同帧."""
        with _lock:
            if not _rec['active']:
                return
            if (_state['values'] is None or _state['travel'] is None
                    or _state['ad'] is None or _state['status'] is None):
                return
            now = time.monotonic()
            elapsed = now - _rec['start']
            row = ([round(elapsed, 3)]
                   + list(_state['values']) + list(_state['travel'])
                   + list(_state['ad']) + list(_state['status'])
                   + [_state['frames'], round(_state['rate'], 1)])
            _rec['rows'].append(row)
            _rec['count'] = len(_rec['rows'])
            if elapsed >= _rec['seconds']:
                _finish_record_locked()
        _flush_pending()

    def _episode_maybe(self) -> None:
        """力离开空载才记，归零稳住后写成一段 CSV."""
        with _lock:
            if (_state['values'] is None or _state['travel'] is None
                    or _state['ad'] is None or _state['status'] is None):
                return
            now = time.monotonic()
            row = ([now] + list(_state['values']) + list(_state['travel'])
                   + list(_state['ad']) + list(_state['status'])
                   + [_state['frames'], round(_state['rate'], 1)])
            closed = _episode.update(now, peak_level(_state['values']), row)
        if closed is not None:
            _pending.append((closed, time.time(), time.monotonic()))
        _flush_pending()


def _csv_header() -> list:
    return (['t_s'] + [f'f{c}' for c in CH] + [f'tr{c}' for c in CH]
            + [f'ad{c}' for c in CH] + [f'st{c}' for c in CH]
            + ['frames', 'hz'])


def _flush_pending() -> None:
    """有正在写的采摘会话才落盘；没有就留在内存里等会话出现."""
    global _last_flush
    with _lock:
        holding_manual = bool(_rec.get('hold'))
        known = _ep['session'] is not None
    if not _pending and not holding_manual:
        return
    now = time.monotonic()
    if not known and (now - _last_flush) < 0.5:
        return
    _last_flush = now
    session = live_peach_session(RUNS)
    if session is None:
        return
    batch = list(_pending)
    _pending.clear()
    for closed, wall_end, mono_end in batch:
        _write_episode(session, closed, wall_end, mono_end)
    _write_manual(session)


def _format_wall(epoch: float) -> str:
    """本地时间，保留毫秒。只写到整秒会让分析把接触段放偏最多 1 秒。"""
    whole = int(math.floor(epoch))
    millis = int(round((epoch - whole) * 1000))
    if millis >= 1000:
        whole += 1
        millis -= 1000
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(whole)) + f'.{millis:03d}'


def _write_episode(session: Path, closed, wall_end: float,
                   mono_end: float) -> None:
    """一段接触写入 ``<session>/force/``."""
    wall_start = wall_end - (mono_end - closed.t0)
    force_dir = session / 'force'
    force_dir.mkdir(parents=True, exist_ok=True)
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
    with open(csv_path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(_csv_header())
        writer.writerows(rel_rows)
    meta = {
        'episode': index,
        'reason': closed.reason,
        'session': str(session),
        'wall_start': _format_wall(wall_start),
        'wall_end': _format_wall(wall_end),
        'duration_s': round(closed.duration_s, 3),
        'contact_s': round(closed.contact_s, 3),
        'samples': len(rel_rows),
        'peak_n': [round(v, 4) for v in peaks],
        'csv': str(csv_path),
    }
    (force_dir / f'{name}.json').write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    with open(force_dir / 'index.jsonl', 'a', encoding='utf-8') as fh:
        fh.write(json.dumps(meta, ensure_ascii=False) + '\n')
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


def _finish_record_locked() -> None:
    """20 s 按钮结束；调用者已持 _lock。文件等采摘会话目录再写."""
    if not _rec['active']:
        return
    _rec['active'] = False
    _rec['hold'] = list(_rec['rows'])
    _rec['file'] = None


def _write_manual(session: Path) -> None:
    """手动 20 s CSV 同样放进当前会话的 force/."""
    with _lock:
        rows = _rec.get('hold')
        if not rows:
            return
        _rec['hold'] = None
    force_dir = session / 'force'
    force_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    path = force_dir / f'force_20s_{stamp}.csv'
    with open(path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(_csv_header())
        writer.writerows(rows)
    with _lock:
        _rec['file'] = path
        _ep['session'] = session.name
    print(f'record saved {path} rows={len(rows)}', flush=True)


def _start_record(seconds: int) -> dict:
    with _lock:
        if _rec['active']:
            return {'ok': False, 'error': 'already recording'}
        _rec.update(active=True, start=time.monotonic(),
                    seconds=max(1, min(120, int(seconds))), rows=[],
                    frames=0, file=None, count=0)
        return {'ok': True, 'seconds': _rec['seconds']}


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


def _cop(vals):
    """五点力到 CoP（tcp 系 mm）与总力（N）；无效通道不参与."""
    if not vals:
        return None, 0.0
    tot = sx = sy = 0.0
    for ch, v in zip(CH, vals):
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
        cop, total = _cop(_state['values'])
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
                stamp = float(row['t_s'])
            except (KeyError, TypeError, ValueError):
                continue
            cop, _total = _cop(forces)
            samples.append({
                't': stamp,
                'v': forces,
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
                while True:
                    payload = json.dumps(
                        _snapshot(), ensure_ascii=False, allow_nan=False)
                    self.wfile.write(f'data: {payload}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *args) -> None:
        pass


def main() -> None:
    """订力话题并起本机调试页."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8099)
    args = ap.parse_args()
    rclpy.init()
    node = _ForceListener()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), _Handler)
    print(f'force web: http://127.0.0.1:{args.port}  '
          f'(sub points/travel/ad/status, /record/start)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
