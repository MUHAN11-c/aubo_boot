"""CLI 入口 — 解析命令行参数 → 初始化配置 → 创建应用 → 启动 uvicorn。

链路: launch 文件传递 ROS 覆盖值 → argparse 解析（默认值来自 YAML）
     → cfg.init() 合并覆盖 → create_app() → uvicorn.run()
"""
from __future__ import annotations

import argparse

import uvicorn

from aubo_ros2_web_dashboard import config as cfg
from aubo_ros2_web_dashboard.gateway.app import create_app


def _add_args(p: argparse.ArgumentParser) -> None:
    """定义所有 CLI 参数，默认值从 YAML 配置读取。"""

    # 网关自身
    p.add_argument("port", type=int, nargs="?", default=cfg.gateway_port(),
                   help="监听端口")
    p.add_argument("--bind", "-b", default=cfg.gateway_bind(),
                   help="监听地址")
    p.add_argument("--directory", "-d", required=True,
                   help="静态文件根目录 (share/.../web/public)")

    # rosbridge 上游连接参数
    g = p.add_argument_group("rosbridge 上游")
    g.add_argument("--rosbridge-host", default=cfg.rosbridge_host())
    g.add_argument("--rosbridge-port", type=int, default=cfg.rosbridge_port())
    g.add_argument("--rosbridge-max-msg-bytes", type=int,
                   default=cfg.rosbridge_max_message_bytes())
    g.add_argument("--rosbridge-ping-interval", type=int,
                   default=cfg.rosbridge_ping_interval())
    g.add_argument("--rosbridge-ping-timeout", type=int,
                   default=cfg.rosbridge_ping_timeout())
    g.add_argument("--rosbridge-close-timeout", type=int,
                   default=cfg.rosbridge_close_timeout())

    # web_video 上游连接参数
    w = p.add_argument_group("web_video 上游")
    w.add_argument("--web-video-host", default=cfg.web_video_host())
    w.add_argument("--web-video-port", type=int, default=cfg.web_video_port())

    # 代理调优
    x = p.add_argument_group("代理调优")
    x.add_argument("--proxy-video-connect-timeout", type=float,
                   default=cfg.proxy_video_connect_timeout())
    x.add_argument("--proxy-video-pool-timeout", type=float,
                   default=cfg.proxy_video_pool_timeout())
    x.add_argument("--proxy-video-chunk-bytes", type=int,
                   default=cfg.proxy_video_chunk_bytes())

    # RobotWebTools
    r = p.add_argument_group("RobotWebTools")
    r.add_argument("--rwt-assets-dir", default=None,
                   help="覆盖 robotwebtools 资产目录")


def main(argv: list[str] | None = None) -> None:
    """入口：解析参数 → 合并配置 → 创建应用 → 启动 uvicorn。"""
    p = argparse.ArgumentParser(description="IVG FastAPI 静态网关")
    _add_args(p)
    args = p.parse_args(argv)

    # 步骤 1: 将 CLI 参数合并到全局配置（YAML 已在 import 时加载）
    cli = {k: v for k, v in vars(args).items()
           if v is not None and k != "directory"}
    cfg.init(cli)

    # 步骤 2: 创建 FastAPI 应用
    app = create_app(args.directory, rwt_override=args.rwt_assets_dir)

    # 步骤 3: 启动 uvicorn
    uvicorn.run(app, host=args.bind, port=args.port, log_level="info")
