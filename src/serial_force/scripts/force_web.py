#!/usr/bin/env python3
"""
力传感器实时调试 Web.

订 /force/points（牛顿），SSE 推给浏览器按 CAD 实测五点显示。

用法（先 source Jazzy 再 source 工作区）：
  python3 src/serial_force/scripts/force_web.py [--port 8099]

串口由 serial_force_node 持有，本服务只订话题、不碰 /dev/force，
可与驱动并行。浏览器开 http://127.0.0.1:<port>。
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

HTML_PATH = Path(__file__).with_name('force_web.html')
FACE_PATH = Path(__file__).with_name('face_fruit.png')
FACE_FULL_PATH = Path(__file__).with_name('face_full.png')
ASSETS = {'/face.png': FACE_PATH, '/face_full.png': FACE_FULL_PATH}

_lock = threading.Lock()
_stamps: deque[float] = deque(maxlen=300)
_state = {'values': None, 'frames': 0, 'last_mono': None, 'rate': 0.0}


class _ForceListener(Node):
    """只订 /force/points；README 契约：Reliable+Volatile，5 值牛顿，通道 1/2/3/5/7."""

    def __init__(self) -> None:
        super().__init__('force_web')
        self.create_subscription(Float64MultiArray, '/force/points', self._cb, 10)

    def _cb(self, msg: Float64MultiArray) -> None:
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


def _snapshot() -> dict:
    with _lock:
        last = _state['last_mono']
        return {
            'v': _state['values'],
            'frames': _state['frames'],
            'hz': round(_state['rate'], 1),
            'age_ms': None if last is None else round((time.monotonic() - last) * 1000),
        }


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == '/':
            body = HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        elif self.path in ASSETS:
            try:
                body = ASSETS[self.path].read_bytes()
            except OSError:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith('/stream'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(_snapshot(), ensure_ascii=False)
                    self.wfile.write(f'data: {payload}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *args) -> None:  # 调试页不刷访问日志
        pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8099)
    args = ap.parse_args()
    rclpy.init()
    node = _ForceListener()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), _Handler)
    print(f'force web: http://127.0.0.1:{args.port}  (sub /force/points)', flush=True)
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
