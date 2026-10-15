"""ros_web_link CLI 入口 — argparse → cfg.init → create_app → uvicorn."""
from __future__ import annotations

import argparse
import os
import socket

import uvicorn

from . import config as cfg


def _check_port_free(bind: str, port: int) -> None:
    """启动前端口占用预检.

    launch 里 respawn=true 时 bind 失败会静默 respawn 死循环
    （端口被占/残留进程）；预检给出可诊断的错误信息后快速失败。
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((bind, port))
    except OSError as e:
        raise SystemExit(
            f"[ros_web_link] 端口 {bind}:{port} 绑定失败: {e}——"
            f"疑似被占用（pgrep -af 'ros_web_link|uvicorn' 查残留；"
            f"8090 归 peach_observability，桥默认 8765 常被 preview_server 占用）") from e


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="ros_web_link_gateway",
        description="ROS Web Link 网关（纯传输层；UI 层通过 create_app 组合）",
    )
    p.add_argument("port", type=int, nargs="?", default=cfg.gateway_port(),
                   help=f"监听端口（默认 {cfg.gateway_port()}；8090 归 peach_observability）")
    p.add_argument("--bind", "-b", default=cfg.gateway_bind(),
                   help=f"监听地址（默认 {cfg.gateway_bind()}）")
    p.add_argument("--directory", "-d", required=False, default=None,
                   help="UI 层 HTML 静态根（纯传输模式省略）")
    p.add_argument("--js-directory", "-j", action="append", default=[],
                   help="UI 层 JS 静态根（可多次指定；通信层自动包含）")

    f = p.add_argument_group("foxglove_bridge 上游")
    f.add_argument("--foxglove-bridge-host", default=cfg.foxglove_bridge_host())
    f.add_argument("--foxglove-bridge-port", type=int, default=cfg.foxglove_bridge_port())
    f.add_argument("--foxglove-bridge-ws-path", default=cfg.foxglove_bridge_ws_path(),
                   help=f"上游 WS 路径（默认 {cfg.foxglove_bridge_ws_path()!r}，vendored 桥服务于根路径）")
    f.add_argument("--foxglove-bridge-compression", action="store_true",
                   default=cfg.foxglove_bridge_compression(),
                   help="上游腿开启 permessage-deflate（默认关：CDR 二进制压缩收益差）")
    f.add_argument("--foxglove-bridge-max-msg-bytes", type=int,
                   default=cfg.foxglove_bridge_max_message_bytes())
    f.add_argument("--foxglove-bridge-ping-interval", type=int,
                   default=cfg.foxglove_bridge_ping_interval())
    f.add_argument("--foxglove-bridge-ping-timeout", type=int,
                   default=cfg.foxglove_bridge_ping_timeout())
    f.add_argument("--foxglove-bridge-close-timeout", type=int,
                   default=cfg.foxglove_bridge_close_timeout())

    g = p.add_argument_group("网关 WebSocket（客户端腿）")
    g.add_argument("--ws-ping-interval", type=float, default=cfg.gateway_ws_ping_interval(),
                   help="客户端腿心跳间隔秒（默认 20；传 -1 关闭——不建议，半开连接不可检测）")
    g.add_argument("--ws-ping-timeout", type=float, default=cfg.gateway_ws_ping_timeout(),
                   help="客户端腿心跳超时秒（默认 20）")
    g.add_argument("--ws-permessage-deflate", action="store_true",
                   default=cfg.gateway_ws_per_message_deflate(),
                   help="客户端腿开启 permessage-deflate（默认关）")
    return p


def main(args: list[str] | None = None) -> None:
    p = build_parser()
    ns = p.parse_args(args)

    ws_ping_interval = None if ns.ws_ping_interval is None or ns.ws_ping_interval < 0 \
        else ns.ws_ping_interval
    ws_ping_timeout = None if ws_ping_interval is None or ns.ws_ping_timeout is None \
        else ns.ws_ping_timeout

    cfg.init({
        "port": ns.port,
        "bind": ns.bind,
        "foxglove_bridge_host": ns.foxglove_bridge_host,
        "foxglove_bridge_port": ns.foxglove_bridge_port,
        "foxglove_bridge_ws_path": ns.foxglove_bridge_ws_path,
        "foxglove_bridge_compression": ns.foxglove_bridge_compression,
        "foxglove_bridge_max_msg_bytes": ns.foxglove_bridge_max_msg_bytes,
        "foxglove_bridge_ping_interval": ns.foxglove_bridge_ping_interval,
        "foxglove_bridge_ping_timeout": ns.foxglove_bridge_ping_timeout,
        "foxglove_bridge_close_timeout": ns.foxglove_bridge_close_timeout,
        "gateway_ws_ping_interval": ws_ping_interval,
        "gateway_ws_ping_timeout": ws_ping_timeout,
        "gateway_ws_per_message_deflate": ns.ws_permessage_deflate,
    })

    _check_port_free(ns.bind, ns.port)

    from .app import create_app
    app = create_app(
        js_roots=ns.js_directory or None,
        html_root=ns.directory,
    )

    uvicorn.run(
        app,
        host=ns.bind,
        port=ns.port,
        log_level="info",
        ws_ping_interval=ws_ping_interval,
        ws_ping_timeout=ws_ping_timeout,
        ws_per_message_deflate=ns.ws_permessage_deflate,
        ws_max_size=cfg.foxglove_bridge_max_message_bytes(),
    )


if __name__ == "__main__":
    main()
