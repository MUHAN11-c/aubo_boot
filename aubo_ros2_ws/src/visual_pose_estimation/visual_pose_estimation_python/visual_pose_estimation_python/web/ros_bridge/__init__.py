"""ROS2 桥：在独立线程中 spin 节点，供 FastAPI 同步路由调用。"""

from .manager import RosBridgeManager

__all__ = ["RosBridgeManager"]
