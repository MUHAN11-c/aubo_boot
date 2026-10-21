#!/usr/bin/env python3
"""
力传感器实时调试 Web.

订 /force/points（N）、/force/travel（mm）、/force/ad（原始 AD）、
/force/status（每通道 OK/SAT/OPEN/INVALID），SSE 推给浏览器；
并提供 20 s 记录按钮（保存 CSV 到 runs/force_20s_*/force.csv）。

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
from std_msgs.msg import Float64MultiArray, UInt8MultiArray

ROOT = Path(__file__).resolve().parents[3]
RUNS = ROOT / 'runs'
HTML_PATH = Path(__file__).with_name('force_web.html')
FACE_PATH = Path(__file__).with_name('face_fruit.png')
FACE_FULL_PATH = Path(__file__).with_name('face_full.png')
ASSETS = {'/face.png': FACE_PATH, '/face_full.png': FACE_FULL_PATH}

CH = (1, 2, 3, 5, 7)
_lock = threading.Lock()
_stamps: deque[float] = deque(maxlen=300)
_state = {'values': None, 'travel': None, 'ad': None, 'status': None,
          'frames': 0, 'last_mono': None, 'rate': 0.0}
_rec = {'active': False, 'start': 0.0, 'seconds': 20, 'rows': [],
        'frames': 0, 'file': None, 'count': 0}


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

    def _record_maybe(self) -> None:
        """status 是节点最后一个话题；在这里落一行，保证 f/t/a/s 同帧."""
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


def _finish_record_locked() -> None:
    """把 _rec['rows'] 写成 CSV；调用者已持 _lock."""
    if not _rec['active']:
        return
    _rec['active'] = False
    ts = time.strftime('%Y%m%d_%H%M%S')
    run = RUNS / f'force_20s_{ts}'
    run.mkdir(parents=True, exist_ok=True)
    path = run / 'force.csv'
    header = (['t_s'] + [f'f{c}' for c in CH] + [f'tr{c}' for c in CH]
              + [f'ad{c}' for c in CH] + [f'st{c}' for c in CH]
              + ['frames', 'hz'])
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(_rec['rows'])
    _rec['file'] = path
    print(f'record saved {path} rows={_rec["count"]}', flush=True)


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


def _clean(vals):
    if vals is None:
        return None
    return [v if math.isfinite(v) else None for v in vals]


def _snapshot() -> dict:
    with _lock:
        last = _state['last_mono']
        return {
            'v': _clean(_state['values']),
            't': _clean(_state['travel']),
            'a': _clean(_state['ad']),
            's': _state['status'],
            'frames': _state['frames'],
            'hz': round(_state['rate'], 1),
            'age_ms': None if last is None else round(
                (time.monotonic() - last) * 1000),
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
