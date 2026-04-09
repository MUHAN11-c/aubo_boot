"""
命令行入口：``main()`` → ``create_app`` → ``uvicorn.run``。

与 ``fastapi_static_gateway`` 模块、``setup.py`` 的 ``ivg_fastapi_static_gateway`` 控制台脚本
共用此入口；ROS launch 使用 ``python3 -m ... fastapi_static_gateway`` 也会走到 ``main()``。
"""
from __future__ import annotations

import argparse

import uvicorn

from aubo_ros2_web_dashboard.gateway.app import create_app


def _build_parser() -> argparse.ArgumentParser:
	# 监听端口与静态根目录由命令行决定；rosbridge/web_video 上游仅读环境变量（见 settings / launch additional_env）
	p = argparse.ArgumentParser(description="FastAPI static gateway (IVG web dashboard).")
	p.add_argument("port", type=int, nargs="?", default=8090, help="listen port")
	p.add_argument(
		"--bind",
		"-b",
		default="0.0.0.0",
		metavar="ADDRESS",
		help="bind address (default 0.0.0.0)",
	)
	p.add_argument(
		"--directory",
		"-d",
		required=True,
		metavar="DIR",
		help="document root (installed share/.../web/public)",
	)
	return p


def main(argv: list[str] | None = None) -> None:
	args = _build_parser().parse_args(argv)
	app = create_app(args.directory)
	host = args.bind if args.bind else "0.0.0.0"
	# Uvicorn 为 ASGI 宿主；传入的 app 即 create_app 返回的 FastAPI 实例
	uvicorn.run(app, host=host, port=args.port, log_level="info")
