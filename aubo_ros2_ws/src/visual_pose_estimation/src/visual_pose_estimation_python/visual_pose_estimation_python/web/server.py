from __future__ import annotations

import argparse
import os

import uvicorn

from .app import create_app


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the FastAPI web service")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host")
    parser.add_argument("--port", type=int, default=8088, help="Bind port")
    parser.add_argument("--reload", action="store_true", help="Enable auto reload")
    args, _unknown_args = parser.parse_known_args()
    os.environ["VPE_WEB_HOST"] = args.host
    os.environ["VPE_WEB_PORT"] = str(args.port)

    uvicorn.run(
        create_app(),
        host=args.host,
        port=args.port,
        reload=args.reload,
    )


if __name__ == "__main__":
    main()
