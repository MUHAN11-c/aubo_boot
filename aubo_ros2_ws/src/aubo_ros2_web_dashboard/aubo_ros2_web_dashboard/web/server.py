"""
`ros2 run aubo_ros2_web_dashboard web_dashboard` 的入口。

解析命令行与部分环境变量后，以工厂模式加载 `create_app`，便于 uvicorn 多 worker 时各自构造应用实例。
"""

from __future__ import annotations

import argparse
import os

import uvicorn


def main() -> None:
    parser = argparse.ArgumentParser(description="启动 aubo_ros2_web_dashboard 网关（FastAPI + uvicorn）")
    parser.add_argument("--host", default=os.environ.get("AUBO_WEB_HOST", "0.0.0.0"), help="Bind host")
    parser.add_argument("--port", type=int, default=int(os.environ.get("AUBO_WEB_PORT", "8090")), help="Bind port")
    parser.add_argument("--reload", action="store_true", help="Dev auto-reload")
    args, _rest = parser.parse_known_args()
    os.environ["AUBO_WEB_HOST"] = args.host
    os.environ["AUBO_WEB_PORT"] = str(args.port)

    log_level = (os.environ.get("AUBO_WEB_LOG_LEVEL") or "info").lower()
    if log_level not in ("critical", "error", "warning", "info", "debug", "trace"):
        log_level = "info"

    uvicorn.run(
        "aubo_ros2_web_dashboard.web.app:create_app",
        factory=True,
        host=args.host,
        port=args.port,
        reload=args.reload,
        log_level=log_level,
    )


if __name__ == "__main__":
    main()
