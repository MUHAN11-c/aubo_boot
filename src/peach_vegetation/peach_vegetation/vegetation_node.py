"""
枝/叶分割 Lifecycle 节点（图名 peach_vegetation）.

订阅相对名 ``image``（launch remap 到相机彩色图），Active 后才推理。
发布 ``/peach/vegetation/{leaf_mask,branch_mask,overlay,status}``。
不写 PlanningScene、不发运动、不进 harvest_system。
"""

from __future__ import annotations

import json
import threading

from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
import numpy as np
from peach_common.lifecycle import ensure_lifecycle_active
from peach_common.qos import sensor
from peach_vegetation.params import peach_vegetation
from peach_vegetation.split import config_from_params, FrangiExgSplitter
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String

LEAF_MASK_TOPIC = '/peach/vegetation/leaf_mask'
BRANCH_MASK_TOPIC = '/peach/vegetation/branch_mask'
OVERLAY_TOPIC = '/peach/vegetation/overlay'
STATUS_TOPIC = '/peach/vegetation/status'

_STREAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class VegetationNode(LifecycleNode):
    """Active 后才分割彩色帧；忙则丢帧."""

    def __init__(self):
        """建节点：资源在 configure 里加载."""
        super().__init__('peach_vegetation')
        self._active = False
        self._busy = threading.Lock()
        self.bridge = CvBridge()
        self._params = None
        self._splitter = None
        self._leaf_pub = None
        self._branch_pub = None
        self._overlay_pub = None
        self._status_pub = None
        self._image_sub = None
        self._updater = None
        self._dropped = 0
        self._last_infer_ms = -1.0
        self._last_leaf_frac = 0.0
        self._last_branch_frac = 0.0
        self._last_device = ''
        # GPU 推理专用互斥组：不与诊断定时器/生命周期服务共用默认组，
        # 否则一帧 Frangi（数百 ms）期间 diagnostics 全部饿死
        self._infer_group = MutuallyExclusiveCallbackGroup()

    def on_configure(self, state):
        """装参数、预热 GPU、建 publisher / diagnostics."""
        del state
        try:
            self._params = peach_vegetation.attach(self)
            self._splitter = FrangiExgSplitter(config_from_params(self._params))
            self._splitter.warmup()
        except Exception as exc:  # noqa: BLE001 启动失败停在 Unconfigured
            self.get_logger().error(f'configure 失败: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._leaf_pub = self.create_lifecycle_publisher(
            Image, LEAF_MASK_TOPIC, _STREAM_QOS)
        self._branch_pub = self.create_lifecycle_publisher(
            Image, BRANCH_MASK_TOPIC, _STREAM_QOS)
        self._overlay_pub = self.create_lifecycle_publisher(
            Image, OVERLAY_TOPIC, _STREAM_QOS)
        self._status_pub = self.create_lifecycle_publisher(
            String, STATUS_TOPIC, _STREAM_QOS)
        self._image_sub = self.create_subscription(
            Image, 'image', self._on_image, sensor(depth=10),
            callback_group=self._infer_group)
        self._updater = Updater(self)
        self._updater.setHardwareID(self._splitter.device)
        self._updater.add('vegetation_split', self._diag)
        self._last_device = self._splitter.device
        self.get_logger().info(
            f'vegetation device={self._splitter.device} '
            f'(requested={self._params.device})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """打开输出；此后才处理图像回调."""
        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state):
        """停推理，保留 GPU 资源待再激活."""
        self._active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        """释放订阅、发布者与分割器."""
        del state
        self._active = False
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        """进程退出路径."""
        del state
        self._active = False
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def ensure_active(self) -> None:
        """
        进入 Active.

        独立 launch 的 EmitEvent 有时匹配不到本节点；spin 前自行转换
        （与 observability 共用 peach_common helper）。
        """
        ensure_lifecycle_active(self)

    def _release(self) -> None:
        """Destroy ROS handles and drop the splitter (frees CUDA)."""
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
        for attr in ('_leaf_pub', '_branch_pub', '_overlay_pub', '_status_pub'):
            pub = getattr(self, attr)
            if pub is not None:
                self.destroy_lifecycle_publisher(pub)
                setattr(self, attr, None)
        self._splitter = None
        self._updater = None

    def _diag(self, stat):
        """diagnostic_updater 回调：延迟 / 丢帧 / 设备."""
        if not self._active:
            stat.summary(DiagnosticStatus.STALE, 'inactive')
        elif self._last_infer_ms < 0.0:
            stat.summary(DiagnosticStatus.WARN, 'no frames')
        else:
            stat.summary(
                DiagnosticStatus.OK,
                f'{self._last_infer_ms:.1f} ms {self._last_device}')
        stat.add('device', self._last_device)
        stat.add('dropped', str(self._dropped))
        stat.add('leaf_frac', f'{self._last_leaf_frac:.4f}')
        stat.add('branch_frac', f'{self._last_branch_frac:.4f}')
        return stat

    def _on_image(self, msg: Image) -> None:
        """Active 且空闲才分割；否则丢这一帧."""
        if not self._active or self._splitter is None:
            return
        if not self._busy.acquire(blocking=False):
            self._dropped += 1
            return
        try:
            self._process(msg)
        except Exception as exc:  # noqa: BLE001 单帧失败不杀 executor
            self.get_logger().error(f'分割失败: {exc}')
        finally:
            self._busy.release()
            if self._updater is not None:
                self._updater.force_update()

    def _process(self, msg: Image) -> None:
        """Decode BGR, split, publish masks and status JSON."""
        bgr = self._to_bgr(msg)
        result = self._splitter.split(bgr)
        pixels = float(result.leaf.size) if result.leaf.size else 1.0
        self._last_infer_ms = result.infer_ms
        self._last_device = result.device
        self._last_leaf_frac = float(np.count_nonzero(result.leaf)) / pixels
        self._last_branch_frac = float(
            np.count_nonzero(result.branch)) / pixels
        header = msg.header
        self._leaf_pub.publish(self._mask_msg(header, result.leaf))
        self._branch_pub.publish(self._mask_msg(header, result.branch))
        if self._params.publish_overlay:
            overlay_msg = self.bridge.cv2_to_imgmsg(result.overlay, encoding='bgr8')
            overlay_msg.header = header
            self._overlay_pub.publish(overlay_msg)
        status = String()
        status.data = json.dumps({
            'infer_ms': round(result.infer_ms, 3),
            'device': result.device,
            'leaf_frac': round(self._last_leaf_frac, 5),
            'branch_frac': round(self._last_branch_frac, 5),
            'dropped': self._dropped,
            'stamp': {
                'sec': int(header.stamp.sec),
                'nanosec': int(header.stamp.nanosec),
            },
            'frame_id': header.frame_id,
        }, separators=(',', ':'))
        self._status_pub.publish(status)

    def _to_bgr(self, msg: Image) -> np.ndarray:
        """Decode sensor_msgs/Image to HxWx3 uint8 BGR."""
        encoding = (msg.encoding or 'bgr8').lower()
        if encoding in ('bgr8', '8uc3'):
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if encoding == 'rgb8':
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            return rgb[:, :, ::-1]
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _mask_msg(self, header, mask: np.ndarray) -> Image:
        """Pack a bool mask as mono8 0/255, same stamp/frame as the color."""
        packed = np.where(mask, 255, 0).astype(np.uint8)
        msg = self.bridge.cv2_to_imgmsg(packed, encoding='mono8')
        msg.header = header
        return msg


def main(args=None):
    """
    Run the vegetation node; self-activate if launch events miss.

    双线程执行器：GPU 推理（专用互斥组）与 diagnostics 定时器并行，
    单线程会把诊断饿死在长帧推理上。
    """
    rclpy.init(args=args)
    node = VegetationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        node.ensure_active()
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
