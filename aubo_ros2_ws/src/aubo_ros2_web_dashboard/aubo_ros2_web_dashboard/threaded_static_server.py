"""
多线程静态 HTTP 服务（RobotWebTools 场景下并发拉取 JS/CSS/图片）。

Python 3.7+ 的 ``python -m http.server`` 已使用 ThreadingHTTPServer；本模块显式固定该行为，
并保证与 launch 中 ``port / --bind / --directory`` 参数形式一致，便于与 colcon 安装路径配合。
"""
from __future__ import annotations

import argparse
import os
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ThreadingHTTPServer static files (IVG web dashboard).")
    p.add_argument("port", type=int, nargs="?", default=8000, help="listen port")
    p.add_argument("--bind", "-b", default="", metavar="ADDRESS", help="bind address; empty = all interfaces")
    p.add_argument("--directory", "-d", default=".", metavar="DIR", help="document root")
    return p


def main(argv: list[str] | None = None) -> None:
    args = _build_parser().parse_args(argv)
    root = os.path.abspath(os.path.realpath(args.directory))
    if not os.path.isdir(root):
        raise SystemExit(f"not a directory: {root}")

    handler_cls = partial(SimpleHTTPRequestHandler, directory=root)
    bind = args.bind or ""
    httpd = ThreadingHTTPServer((bind, args.port), handler_cls)
    httpd.daemon_threads = True
    host_display = bind or "0.0.0.0"
    print(f"Serving {root} at http://{host_display}:{args.port}/ (ThreadingHTTPServer)", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, exiting.", flush=True)
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
