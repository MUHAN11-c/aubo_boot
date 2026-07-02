"""命令行入口：解析 host/port，以 Uvicorn + create_app 工厂启动 ASGI 服务。"""

from __future__ import annotations

import argparse
import os

import uvicorn


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the FastAPI web service")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host")
    parser.add_argument("--port", type=int, default=8088, help="Bind port")
    parser.add_argument("--reload", action="store_true", help="Enable auto reload")
    args, _unknown_args = parser.parse_known_args()
    os.environ["VPE_WEB_HOST"] = args.host
    os.environ["VPE_WEB_PORT"] = str(args.port)

    # reload 子进程需要可导入路径；factory 与 create_app 一致
    uvicorn.run(
        "visual_pose_estimation_python.web.app:create_app",
        factory=True,
        host=args.host,
        port=args.port,
        reload=args.reload,
        reload_excludes=[
            "build/**",
            "install/**",
            "log/**",
            "rosbags/**",
            "**/__pycache__/*",
            "*.pyc",
        ] if args.reload else None,
        reload_includes=["*.py", "*.yaml", "*.yml", "*.json"] if args.reload else None,
    )


if __name__ == "__main__":
    main()
