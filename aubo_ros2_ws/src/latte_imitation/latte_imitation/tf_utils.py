"""
TF 查询工具 — 三种模式覆盖不同使用场景喵~

模式 1: get_current_ee_pose(node) — 在已有 ROS 节点中同步查询 (带重试)
  适用: trajectory_pipeline.py 的 LatteImitationNode 内部

模式 2: get_ee_pose_from_tf() — 独立脚本模式 (创建临时节点, 查完销毁)
  适用: visualize_latte_trajectory.py 等非持久运行的脚本

模式 3: TfQueryNode — GUI 持久模式 (后台线程 spin)
  适用: 需要持续查询 TF 的 GUI 应用
  注意: 标记为 @deprecated, 新代码优先使用 test_latte_pour.py + RViz2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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
    """在已有 ROS 节点中同步查询 TF，带重试喵~"""
    buf = getattr(node, '_tf_buffer', None)
    if buf is None:
        buf = Buffer(node=node)
        tf_listener = TransformListener(buf, node)
        node._tf_buffer = buf
        node._tf_listener = tf_listener

    for i in range(retry_count):
        try:
            tfs = buf.lookup_transform(base_frame, ee_link, rclpy.time.Time())
            return tf_to_pose(tfs)
        except (LookupException, ConnectivityException, ExtrapolationException):
            if i < retry_count - 1:
                rclpy.spin_once(node, timeout_sec=retry_interval)
    return None


def get_ee_pose_from_tf(base_frame: str = DEFAULT_BASE_FRAME,
                        ee_link: str = DEFAULT_EE_LINK,
                        timeout: float = 2.0) -> Pose | None:
    """独立查询 TF — 创建临时节点，查完即销毁喵~"""
    if not rclpy.ok():
        rclpy.init(args=[])
    node = rclpy.create_node("_latte_tf_query_temp")
    try:
        buf = Buffer(node=node)
        _listener = TransformListener(buf, node)
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
    """持久 TF 查询节点 — 后台线程 spin，适合 GUI 应用 喵~

    @deprecated: 新代码优先使用 test_latte_pour.py + RViz2 preview 模式
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
        try:
            tfs = self._buffer.lookup_transform(
                self._base, self._ee, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
            return tf_to_pose(tfs)
        except Exception:
            return None

    def shutdown(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
