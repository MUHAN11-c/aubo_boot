"""入口模块 — python -m aubo_ros2_web_dashboard.fastapi_static_gateway。

由 ROS 2 launch 文件通过 ExecuteProcess 调用，CLI 参数全部由 launch 传递。
"""
from __future__ import annotations

from aubo_ros2_web_dashboard.gateway.cli import main

if __name__ == "__main__":
    main()
