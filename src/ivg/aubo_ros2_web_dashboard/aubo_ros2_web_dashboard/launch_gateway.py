"""aubo_ros2_web_dashboard 组合网关入口 — 组合 ros_web_link 通信层 + UI BFF/静态.

用法: python3 -m aubo_ros2_web_dashboard.launch_gateway <port> --bind ... --directory ... --js-directory ...
"""
from __future__ import annotations

import argparse
import sys


def main(args: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(
        prog="aubo_ros2_web_dashboard.launch_gateway",
        description="组合网关：ros_web_link 传输 + UI BFF + 静态文件",
    )
    p.add_argument("port", type=int, nargs="?", default=8095)
    p.add_argument("--bind", "-b", default="0.0.0.0")
    p.add_argument("--directory", "-d", required=True, help="UI 层 HTML 静态根")
    p.add_argument("--js-directory", "-j", action="append", default=[], help="UI 层 JS 静态根（可多次）")
    p.add_argument("--foxglove-bridge-host", default="127.0.0.1")
    p.add_argument("--foxglove-bridge-port", type=int, default=8765)
    ns = p.parse_args(args)

    # 1) 初始化通信层配置
    from ros_web_link import config as transport_cfg
    transport_cfg.init({
        "port": ns.port,
        "bind": ns.bind,
        "foxglove_bridge_host": ns.foxglove_bridge_host,
        "foxglove_bridge_port": ns.foxglove_bridge_port,
    })

    # 2) UI 层 BFF 子路由
    from aubo_ros2_web_dashboard.gateway_routes import get_bff_router
    bff = get_bff_router()

    # 3) 合并静态根：通信层 JS 逐文件 symlink 到 UI 层 js/ 目录（启动时合成）
    #    2026-10-13 单源化重构：传输层（transport/core/view3d/vendor/robotwebtools/
    #    ivg_*）唯一源在 ros_web_link；UI 层仅保留面板与 UI 专有模块，
    #    core/ 等目录两边混居 → 必须逐文件合并（目录级 symlink 不再适用）。
    from pathlib import Path as _Path
    ui_js = _Path(ns.directory) / "js"
    if ui_js.is_dir():
        try:
            from ament_index_python.packages import get_package_share_directory
            transport_js = _Path(get_package_share_directory("ros_web_link")) / "web" / "public" / "js"
            merged = skipped = 0
            if transport_js.is_dir():
                for src in sorted(transport_js.rglob("*")):
                    if src.is_dir() or "__tests__" in src.parts:
                        continue
                    rel = src.relative_to(transport_js)
                    dst = ui_js / rel
                    if dst.exists() or dst.is_symlink():
                        skipped += 1
                        continue
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    dst.symlink_to(src)
                    merged += 1
            print(f"[launch_gateway] 通信层 JS 逐文件合并: {merged} 新建 / {skipped} 已在")
        except Exception as e:
            print(f"[launch_gateway] JS 合并警告: {e}")

    # 4) 组合应用（单根：UI 层 directory，已含通信层 symlink）
    from ros_web_link.app import create_app
    app = create_app(
        html_root=ns.directory,
        sub_routers=[bff],
    )

    # 4) 启动
    import uvicorn
    uvicorn.run(
        app,
        host=ns.bind,
        port=ns.port,
        log_level="info",
        ws_ping_interval=None,
        ws_max_size=transport_cfg.foxglove_bridge_max_message_bytes(),
    )


if __name__ == "__main__":
    main()
