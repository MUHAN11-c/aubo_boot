"""
共享 TF 查询工具 — 消除 trajectory_pipeline / visualize / debug_panel 三处重复代码喵~

提供:
  - get_current_ee_pose()   同步阻塞查询 base_link → tool_tcp (含重试，适合 ROS 节点)
  - get_ee_pose_from_tf()   独立查询 (创建临时节点，适合非 ROS 脚本)
  - TfQueryNode              持久 ROS 节点 + 后台线程 spin (适合 GUI 应用)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# ═══════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════

DEFAULT_BASE_FRAME = "base_link"
DEFAULT_EE_LINK = "tool_tcp"
TF_RETRY_COUNT = 20
TF_RETRY_INTERVAL = 0.03


def tf_to_pose(tfs) -> Pose:
    """TransformStamped → geometry_msgs/Pose 喵~"""
    t = tfs.transform.translation
    r = tfs.transform.rotation
    return Pose(
        position=Point(x=t.x, y=t.y, z=t.z),
        orientation=Quaternion(x=r.x, y=r.y, z=r.z, w=r.w),
    )


def get_current_ee_pose(node: Node, base_frame: str = DEFAULT_BASE_FRAME,
                        ee_link: str = DEFAULT_EE_LINK,
                        retry_count: int = TF_RETRY_COUNT,
                        retry_interval: float = TF_RETRY_INTERVAL) -> Pose | None:
    """在已有 ROS 节点中同步查询 TF，带重试喵~

    适用于 trajectory_pipeline.py 的 LatteImitationNode 内部喵~

    Args:
        node:       已有 ROS 2 节点 (需要已创建 tf_buffer + tf_listener 喵~)
        base_frame: TF 基准坐标系 (默认 base_link) 喵~
        ee_link:    TF 目标坐标系 (默认 tool_tcp) 喵~
        retry_count:   重试次数 喵~
        retry_interval: 重试间隔 (秒) 喵~

    Returns:
        geometry_msgs/Pose 或 None 喵~
    """
    # 从 node 获取或创建 tf buffer/listener 喵~
    buf = getattr(node, '_tf_buffer', None)
    if buf is None:
        buf = Buffer(node=node)
        tf_listener = TransformListener(buf, node)
        node._tf_buffer = buf
        node._tf_listener = tf_listener

    for i in range(retry_count):
        try:
            tfs = buf.lookup_transform(
                base_frame, ee_link, rclpy.time.Time(),
            )
            return tf_to_pose(tfs)
        except (LookupException, ConnectivityException, ExtrapolationException):
            if i < retry_count - 1:
                rclpy.spin_once(node, timeout_sec=retry_interval)
    return None


def get_ee_pose_from_tf(base_frame: str = DEFAULT_BASE_FRAME,
                        ee_link: str = DEFAULT_EE_LINK,
                        timeout: float = 2.0) -> Pose | None:
    """独立查询 TF — 创建临时节点，查完即销毁喵~

    适用于 visualize_latte_trajectory.py 等非 ROS 持久运行的脚本喵~

    Args:
        base_frame: TF 基准坐标系 喵~
        ee_link:    TF 目标坐标系 喵~
        timeout:    查询超时 (秒) 喵~

    Returns:
        geometry_msgs/Pose 或 None 喵~
    """
    if not rclpy.ok():
        rclpy.init(args=[])
    node = rclpy.create_node("_latte_tf_query_temp")
    try:
        buf = Buffer(node=node)
        _listener = TransformListener(buf, node)
        # 给 TF buffer 短暂填充时间
        rclpy.spin_once(node, timeout_sec=0.5)
        tfs = buf.lookup_transform(
            base_frame, ee_link, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=timeout),
        )
        return tf_to_pose(tfs)
    except Exception:
        return None
    finally:
        node.destroy_node()


class TfQueryNode:
    """持久 TF 查询节点 — 后台线程 spin，适合 GUI 应用 (latte_debug_panel.py) 喵~

    Usage:
        tf_node = TfQueryNode()
        pose = tf_node.get_pose(timeout=1.5)
        # ... GUI 运行中可反复调用 ...
        tf_node.shutdown()
    """

    def __init__(self, base_frame: str = DEFAULT_BASE_FRAME,
                 ee_link: str = DEFAULT_EE_LINK):
        import threading
        if not rclpy.ok():
            rclpy.init(args=[])
        self._base = base_frame
        self._ee = ee_link
        self._node = rclpy.create_node("_latte_tf_persistent")
        self._buffer = Buffer(node=self._node)
        self._listener = TransformListener(self._buffer, self._node)
        self._thread = threading.Thread(
            target=lambda: rclpy.spin(self._node), daemon=True,
        )
        self._thread.start()

    def get_pose(self, timeout: float = 1.5) -> Pose | None:
        """查询当前末端位姿 (非阻塞，指定超时) 喵~"""
        try:
            tfs = self._buffer.lookup_transform(
                self._base, self._ee, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
            return tf_to_pose(tfs)
        except Exception:
            return None

    def shutdown(self):
        """销毁节点，停止后台线程 喵~"""
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
