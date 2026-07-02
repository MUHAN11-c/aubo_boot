"""加载 node_runtime、启动/停止 ROS2Node 与 spin 线程，对外暴露 node 引用。"""

from __future__ import annotations

import importlib
import logging
import threading
import time
from pathlib import Path
from types import ModuleType
from typing import Optional

from ..resources import WebPaths
from ..runtime_support import get_camera_pose_fixed_orientation, get_paths_app_config, get_pose_list_dir, get_templates_dir


LOGGER = logging.getLogger(__name__)


class RosBridgeManager:
    """Manage the ROS2 bridge node inside FastAPI lifespan."""

    def __init__(self, paths: WebPaths) -> None:
        self._paths = paths
        self._module: Optional[ModuleType] = None
        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._startup_error: Optional[str] = None
        self._lock = threading.Lock()

    @property
    def paths(self) -> WebPaths:
        return self._paths

    @property
    def bridge_module(self) -> Optional[ModuleType]:
        return self._module

    @property
    def node(self):
        return self._node

    @property
    def startup_error(self) -> Optional[str]:
        return self._startup_error

    @property
    def app_config(self) -> dict:
        return get_paths_app_config(self._paths)

    @property
    def camera_pose_fixed_orientation(self) -> dict:
        return get_camera_pose_fixed_orientation(self._paths)

    @property
    def templates_dir(self) -> Path:
        return get_templates_dir(self._paths)

    @property
    def pose_list_dir(self) -> Path:
        return get_pose_list_dir(self._paths)

    @property
    def is_ready(self) -> bool:
        return self._module is not None and self._node is not None and self._startup_error is None

    def status(self) -> dict:
        return {
            "ready": self.is_ready,
            "startup_error": self._startup_error,
            "bridge_module": "visual_pose_estimation_python.web.ros_bridge.node_runtime",
        }

    def start(self) -> None:
        with self._lock:
            if self._node is not None or self._startup_error is not None:
                return

            try:
                module = self._load_bridge_module()
                if not module.rclpy.ok():
                    module.rclpy.init()

                node = module.ROS2Node(self._paths)

                self._module = module
                self._node = node
                self._stop_event.clear()
                self._spin_thread = threading.Thread(
                    target=self._spin_loop,
                    name="fastapi-ros2-spin",
                    daemon=True,
                )
                self._spin_thread.start()
                LOGGER.info("FastAPI ROS2 bridge started")
            except Exception as exc:
                self._startup_error = str(exc)
                LOGGER.exception("Failed to start FastAPI ROS2 bridge")

    def stop(self) -> None:
        with self._lock:
            module = self._module
            node = self._node

            self._stop_event.set()
            if self._spin_thread and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)

            try:
                if node is not None:
                    node.destroy_node()
            except Exception:
                LOGGER.exception("Failed to destroy ROS2 bridge node")

            try:
                if module is not None and module.rclpy.ok():
                    module.rclpy.shutdown()
            except Exception:
                LOGGER.exception("Failed to shutdown ROS2 runtime")

            self._module = None
            self._node = None
            self._spin_thread = None

    def _spin_loop(self) -> None:
        """后台 ROS2 spin 线程：持续 spin_once 处理回调和服务响应。

        FastAPI 线程通过 _spin_future() 的 threading.Event 等待结果，
        不再需要 executor_running / executor_lock 协调。
        """
        assert self._module is not None
        assert self._node is not None

        while not self._stop_event.is_set():
            try:
                if not self._module.rclpy.ok():
                    break
                self._module.rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                LOGGER.exception("ROS2 spin loop error")
                # context 可能已被 shutdown（如 uvicorn reload），检查后退出
                try:
                    if self._module is None or not self._module.rclpy.ok():
                        break
                except Exception:
                    break
                time.sleep(0.2)

    def _load_bridge_module(self) -> ModuleType:
        if self._module is not None:
            return self._module

        module_name = "visual_pose_estimation_python.web.ros_bridge.node_runtime"
        return importlib.import_module(module_name)
