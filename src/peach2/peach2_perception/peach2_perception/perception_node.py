"""
peach2_perception lifecycle node: RGB-D frames -> TargetObservationArray in base_link.

Never commands motion or IO. Callbacks only buffer: colour/depth/camera_info are synchronized
with message_filters and handed to one inference thread through a capacity-1 drop-oldest slot.
TF is looked up only at the image stamp (no latest-TF fallback); a miss drops the frame.
Models load in on_configure (a missing GPU or weights file fails the transition).
"""
from __future__ import annotations

from collections import Counter, deque
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
import diagnostic_msgs.msg
import diagnostic_updater
import message_filters
import numpy as np
from peach2_interfaces.msg import TargetObservationArray
from peach2_interfaces.srv import BeginScene
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header
import tf2_ros

from .debug_view import draw_debug
from .detector import Detector, UltralyticsYoloBackend
from .frame_pipeline import builder_params, FrameInput, FramePipeline
from .lock_policy import LockPolicy, LockState, LockStatus
from .msg_conversion import observation_array_msg
from .observation_builder import ObservationBuilder, transform_matrix
from .params import load_params, PerceptionParams
from .segmenter import MobileSamBackend, RefineParams, Segmenter
from .stationary import JointMotionBuffer, Motion
from .worker import LatestSlot

COLOR_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/depth/image_raw'
CAMERA_INFO_TOPIC = '/camera/color/camera_info'
CONFIDENCE_TOPIC = '/camera/depth/confidence'
JOINT_STATES_TOPIC = '/joint_states'
OBSERVATIONS_TOPIC = '/peach/perception/observations'
DEBUG_IMAGE_TOPIC = '~/debug_image'
BEGIN_SCENE_SERVICE = '/peach/perception/begin_scene'

_OBSERVATIONS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
_DEBUG_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
_JOINT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
_DEPTH_ENCODINGS = ('16UC1', 'mono16', '32FC1')
_CONFIDENCE_CACHE = 8
_FPS_WINDOW = 30
_STALE_FRAME_S = 2.0
_WORKER_JOIN_S = 10.0


def _default_config() -> str:
    return os.path.join(get_package_share_directory('peach2_perception'), 'config',
                        'perception.yaml')


def _stamp_s(stamp: TimeMsg) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


class PerceptionNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__('peach2_perception')
        self.declare_parameter('config_file', _default_config())
        self._params: PerceptionParams | None = None
        self._bridge = CvBridge()
        self._state_lock = threading.Lock()
        self._joint_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._active = False
        self._slot: LatestSlot = LatestSlot()
        self._worker: threading.Thread | None = None
        self._pipeline: FramePipeline | None = None
        self._builder: ObservationBuilder | None = None
        self._lock_policy: LockPolicy | None = None
        self._joints: JointMotionBuffer | None = None
        self._confidence: deque = deque(maxlen=_CONFIDENCE_CACHE)
        self._tf_buffer: tf2_ros.Buffer | None = None
        self._tf_listener: tf2_ros.TransformListener | None = None
        self._obs_pub = None
        self._debug_pub = None
        self._mf_subs: list = []
        self._sync = None
        self._conf_sub = None
        self._joint_sub = None
        self._begin_srv = None
        self._bond = None
        self._device = ''
        self._reset_stats()
        self._diag = diagnostic_updater.Updater(self, period=1.0)
        self._diag.setHardwareID('peach2_perception')
        self._diag.add('perception', self._diagnose)

    def _reset_stats(self) -> None:
        with self._stats_lock:
            self._synced = 0
            self._processed = 0
            self._drops: Counter = Counter()
            self._flags_seen: Counter = Counter()
            self._tf_failures = 0
            self._proc_times: deque = deque(maxlen=_FPS_WINDOW)
            self._timing_ema: dict[str, float] = {}
            self._last_status: LockStatus | None = None
            self._last_motion = Motion.UNKNOWN
            self._confidence_source = ''
            self._confirmed = 0
            self._pending = 0
            self._sam_truncated = 0
            self._distorted_info = False

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        path = self.get_parameter('config_file').get_parameter_value().string_value
        try:
            params = load_params(path, get_package_share_directory)
        except (OSError, ValueError, LookupError) as exc:
            self.get_logger().error(f'configure rejected ({path}): {exc}')
            return TransitionCallbackReturn.FAILURE
        try:
            pipeline, builder = self._build_models(params)
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            self.get_logger().error(f'model load failed: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._params = params
        self._reset_stats()
        with self._state_lock:
            self._pipeline = pipeline
            self._builder = builder
            self._lock_policy = LockPolicy(params.lock.min_stationary_frames,
                                           params.lock.stable_s, params.lock.max_collect_s)
        with self._joint_lock:
            self._joints = JointMotionBuffer(params.stationary.buffer_s)
        self._confidence.clear()
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._obs_pub = self.create_lifecycle_publisher(TargetObservationArray,
                                                        OBSERVATIONS_TOPIC, _OBSERVATIONS_QOS)
        if params.debug.publish_image:
            self._debug_pub = self.create_lifecycle_publisher(Image, DEBUG_IMAGE_TOPIC,
                                                              _DEBUG_QOS)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5, durability=DurabilityPolicy.VOLATILE,
            reliability=(ReliabilityPolicy.RELIABLE if params.sensor_qos_reliable
                         else ReliabilityPolicy.BEST_EFFORT))
        inputs = MutuallyExclusiveCallbackGroup()
        self._mf_subs = [
            message_filters.Subscriber(self, Image, COLOR_TOPIC, qos_profile=sensor_qos,
                                       callback_group=inputs),
            message_filters.Subscriber(self, Image, DEPTH_TOPIC, qos_profile=sensor_qos,
                                       callback_group=inputs),
            message_filters.Subscriber(self, CameraInfo, CAMERA_INFO_TOPIC,
                                       qos_profile=sensor_qos, callback_group=inputs),
        ]
        self._sync = message_filters.ApproximateTimeSynchronizer(
            self._mf_subs, queue_size=params.sync_queue_size, slop=params.sync_slop_s)
        self._sync.registerCallback(self._on_frame)
        self._conf_sub = self.create_subscription(Image, CONFIDENCE_TOPIC, self._on_confidence,
                                                  sensor_qos, callback_group=inputs)
        self._joint_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, _JOINT_QOS,
            callback_group=MutuallyExclusiveCallbackGroup())
        self._begin_srv = self.create_service(BeginScene, BEGIN_SCENE_SERVICE,
                                              self._on_begin_scene,
                                              callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f'configured from {path} (device {self._device})')
        return TransitionCallbackReturn.SUCCESS

    def _build_models(self, p: PerceptionParams) -> tuple[FramePipeline, ObservationBuilder]:
        d, s = p.detector, p.segmenter
        detector = Detector(
            UltralyticsYoloBackend(d.model_path, d.engine_path, d.device, d.allow_cpu, d.imgsz,
                                   d.half),
            d.conf, d.iou, d.class_names, d.dedup_ios, d.dedup_frag_ios, d.dedup_area_ratio)
        detector.check_class_names()
        segmenter = Segmenter(
            MobileSamBackend(s.model_path, d.device, d.allow_cpu, s.imgsz), s.max_boxes,
            s.box_expand_frac, s.neg_point_offset_px,
            RefineParams(s.depth_jump_rel, s.depth_jump_abs_m, s.seed_frac, s.morph_kernel_px,
                         s.min_area_px))
        self._device = f'yolo={detector.device} sam={segmenter.device}'
        builder = ObservationBuilder(builder_params(p))
        return FramePipeline(p, detector, segmenter, builder), builder

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        ret = super().on_activate(state)
        self._slot.reopen()
        with self._state_lock:
            self._active = True
        self._worker = threading.Thread(target=self._worker_loop, name='peach2_perception',
                                        daemon=True)
        self._worker.start()
        self._bond = self._create_bond()
        return ret

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._stop_worker()
        self._destroy_bond()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._stop_worker()
        self._destroy_bond()
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def _stop_worker(self) -> None:
        with self._state_lock:
            self._active = False
        self._slot.close()
        if self._worker is not None:
            self._worker.join(timeout=_WORKER_JOIN_S)
            if self._worker.is_alive():
                self.get_logger().warning('inference thread did not stop in time')
            self._worker = None

    def _teardown(self) -> None:
        self._sync = None
        for sub in self._mf_subs:
            self.destroy_subscription(sub.sub)
        self._mf_subs = []
        for attr in ('_conf_sub', '_joint_sub'):
            sub = getattr(self, attr)
            if sub is not None:
                self.destroy_subscription(sub)
                setattr(self, attr, None)
        if self._begin_srv is not None:
            self.destroy_service(self._begin_srv)
            self._begin_srv = None
        for attr in ('_obs_pub', '_debug_pub'):
            pub = getattr(self, attr)
            if pub is not None:
                self.destroy_lifecycle_publisher(pub)
                setattr(self, attr, None)
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        self._tf_buffer = None
        with self._state_lock:
            self._pipeline = None
            self._builder = None
            self._lock_policy = None
        with self._joint_lock:
            self._joints = None
        self._confidence.clear()

    def _create_bond(self):
        try:
            from bondpy.bondpy import Bond
        except ImportError:
            self.get_logger().warning('bondpy not available: lifecycle_manager bond disabled')
            return None
        bond = Bond(self, '/bond', self.get_name())
        bond.start()
        return bond

    def _destroy_bond(self) -> None:
        if self._bond is not None:
            self._bond.shutdown()
            self._bond = None

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def _on_frame(self, color: Image, depth: Image, info: CameraInfo) -> None:
        with self._state_lock:
            if not self._active:
                return
        with self._stats_lock:
            self._synced += 1
        self._slot.put((color, depth, info))

    def _on_confidence(self, msg: Image) -> None:
        self._confidence.append((_stamp_s(msg.header.stamp), msg))

    def _on_joint_states(self, msg: JointState) -> None:
        with self._joint_lock:
            if self._joints is not None:
                self._joints.add(_stamp_s(msg.header.stamp), msg.name, msg.position,
                                 msg.velocity)

    def _on_begin_scene(self, request: BeginScene.Request,
                        response: BeginScene.Response) -> BeginScene.Response:
        with self._state_lock:
            if not self._active or self._lock_policy is None or self._builder is None:
                response.accepted = False
                response.scene_epoch = (self._lock_policy.scene_epoch
                                        if self._lock_policy is not None else 0)
                response.message = 'perception not active'
                return response
            self._builder.reset()
            epoch = self._lock_policy.begin_scene(self._now_s())
        self._slot.clear()
        response.accepted = True
        response.scene_epoch = epoch
        response.message = 'tracks cleared, collecting target set'
        self.get_logger().info(f'begin_scene request_id={request.request_id!r}: '
                               f'scene_epoch={epoch}')
        return response

    def _drop(self, reason: str) -> None:
        with self._stats_lock:
            self._drops[reason] += 1

    # ------------------------------------------------------------------ worker
    def _worker_loop(self) -> None:
        while True:
            item = self._slot.get(timeout=0.5)
            if item is None:
                with self._state_lock:
                    if not self._active:
                        return
                continue
            try:
                self._process(*item)
            except Exception as exc:  # noqa: BLE001 - keep the thread alive, count the frame
                self._drop('process_error')
                self.get_logger().error(f'frame processing failed: {exc!r}',
                                        throttle_duration_sec=5.0)

    def _confidence_for(self, stamp_s: float, tol_s: float) -> np.ndarray | None:
        best = None
        for t, msg in list(self._confidence):
            if abs(t - stamp_s) <= tol_s and (best is None or abs(t - stamp_s) < best[0]):
                best = (abs(t - stamp_s), msg)
        if best is None:
            return None
        return self._bridge.imgmsg_to_cv2(best[1], desired_encoding='passthrough')

    def _process(self, color: Image, depth: Image, info: CameraInfo) -> None:
        p = self._params
        stamp_s = _stamp_s(depth.header.stamp)
        with self._state_lock:
            pipeline, builder, lock = self._pipeline, self._builder, self._lock_policy
            if pipeline is None or builder is None or lock is None:
                return
            epoch = lock.scene_epoch
            if not lock.accepts(stamp_s):
                self._drop('before_scene_start')
                return
        if depth.encoding not in _DEPTH_ENCODINGS:
            self._drop('depth_encoding')
            return
        if (info.width, info.height) != (color.width, color.height) or \
                (depth.width, depth.height) != (color.width, color.height):
            self._drop('size_mismatch')
            return
        K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            self._drop('invalid_intrinsics')
            return
        distorted = any(abs(c) > 1e-6 for c in info.d)
        camera_frame = p.camera_frame or depth.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                p.frame_id, camera_frame, Time.from_msg(depth.header.stamp),
                timeout=Duration(seconds=p.tf_timeout_s))
        except tf2_ros.TransformException:
            with self._stats_lock:
                self._tf_failures += 1
            self._drop('tf_lookup')
            return
        tr, q = tf.transform.translation, tf.transform.rotation
        T = transform_matrix((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        bgr = self._bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
        depth_raw = self._bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        conf = self._confidence_for(stamp_s, p.confidence_match_tol_s)
        with self._joint_lock:
            st = p.stationary
            motion = (self._joints.classify(stamp_s, st.joint_speed_threshold_rad_s,
                                            st.window_s, st.max_gap_s)
                      if self._joints is not None else Motion.UNKNOWN)

        result = pipeline.run(FrameInput(stamp_s, bgr, depth_raw, K, T, conf))
        if distorted:
            for m in result.measurements:
                m.flags.append('distorted_intrinsics')

        with self._state_lock:
            if lock is not self._lock_policy or lock.scene_epoch != epoch:
                self._drop('scene_changed_mid_frame')
                return
            records = builder.associate(stamp_s, result.measurements, motion)
            confirmed, pending = builder.track_summary()
            if lock.state is LockState.COLLECTING:
                status = lock.update(stamp_s, motion is Motion.STATIONARY, confirmed, pending)
            else:
                status = lock.status(stamp_s)

        header = Header(stamp=depth.header.stamp, frame_id=p.frame_id)
        out = observation_array_msg(header, T, status, records)
        if self._obs_pub is not None:
            self._obs_pub.publish(out)
        if self._debug_pub is not None:
            tracked = {id(r.measurement) for r in records}
            untracked = [m for m in result.measurements if id(m) not in tracked]
            caption = (f'epoch {status.scene_epoch} {status.state.value} '
                       f'{motion.value} {result.timings_ms["total"]:.0f} ms')
            img = draw_debug(bgr, records, untracked, p.debug.downscale, caption)
            msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header = color.header
            self._debug_pub.publish(msg)
        self._record_stats(result, status, motion, confirmed, pending, distorted, records)

    def _record_stats(self, result, status: LockStatus, motion: Motion, confirmed, pending: int,
                      distorted: bool, records) -> None:
        with self._stats_lock:
            self._processed += 1
            self._proc_times.append(time.monotonic())
            for k, v in result.timings_ms.items():
                prev = self._timing_ema.get(k)
                self._timing_ema[k] = v if prev is None else 0.9 * prev + 0.1 * v
            if status.state is LockState.LOCKED and (
                    self._last_status is None or not self._last_status.locked):
                self.get_logger().info(
                    f'target set locked ({status.reason}): epoch {status.scene_epoch}, '
                    f'{len(status.locked_ids)} targets')
            self._last_status = status
            self._last_motion = motion
            self._confidence_source = result.confidence_source
            self._confirmed = len(confirmed)
            self._pending = pending
            self._sam_truncated += result.sam_truncated
            self._distorted_info = distorted
            for r in records:
                self._flags_seen.update(r.measurement.flags)

    # ------------------------------------------------------------- diagnostics
    def _diagnose(self, stat):
        levels = diagnostic_msgs.msg.DiagnosticStatus
        with self._state_lock:
            active = self._active
        with self._stats_lock:
            times = list(self._proc_times)
            fps = ((len(times) - 1) / (times[-1] - times[0])
                   if len(times) >= 2 and times[-1] > times[0] else 0.0)
            age = time.monotonic() - times[-1] if times else math.inf
            status = self._last_status
            snapshot = {
                'synced_frames': self._synced, 'processed_frames': self._processed,
                'worker_dropped': self._slot.dropped, 'tf_failures': self._tf_failures,
                'confirmed_tracks': self._confirmed, 'tentative_tracks': self._pending,
                'sam_truncated_total': self._sam_truncated,
                'confidence_source': self._confidence_source,
                'arm_motion': self._last_motion.value, 'device': self._device,
            }
            drops = dict(self._drops)
            timings = dict(self._timing_ema)
            distorted = self._distorted_info
        if not active:
            stat.summary(levels.OK, 'inactive')
        elif age > _STALE_FRAME_S:
            stat.summary(levels.WARN, 'no frames processed recently')
        elif distorted:
            stat.summary(levels.WARN, 'camera_info has distortion; images must be rectified')
        else:
            stat.summary(levels.OK, f'{fps:.1f} fps')
        stat.add('fps', f'{fps:.2f}')
        stat.add('last_frame_age_s', f'{age:.2f}')
        for k, v in snapshot.items():
            stat.add(k, str(v))
        for k, v in sorted(timings.items()):
            stat.add(f'ms.{k}', f'{v:.1f}')
        for k, v in sorted(drops.items()):
            stat.add(f'dropped.{k}', str(v))
        if status is not None:
            stat.add('lock_state', status.state.value)
            stat.add('lock_reason', status.reason)
            stat.add('scene_epoch', str(status.scene_epoch))
            stat.add('locked_targets', str(len(status.locked_ids)))
            stat.add('stationary_frames', str(status.stationary_frames))
        return stat


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
