"""
peach2_scene lifecycle node: on-demand layered scene snapshot -> PlanningScene world objects.

/peach/scene/build_snapshot grabs the next depth frame stamped after the request, aligns TF
(base<-camera and base<-every collision link) and /joint_states at the image stamp, runs
scene_core and replaces all `BuildSceneSnapshot.HARD_OBJECT_PREFIX*` objects through
/apply_planning_scene in one diff. Never writes the ACM (peach2_manipulation owns it), never
commands motion or IO.

Callback groups: service (mutually exclusive), images, joint_states, target models, robot
description (each mutually exclusive), ApplyPlanningScene client (reentrant). Snapshot build and
scene writes are serialised by `_scene_lock`; frame waiting happens outside it.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
import diagnostic_msgs.msg
import diagnostic_updater
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
from peach2_interfaces.msg import TargetModelArray
from peach2_interfaces.srv import BuildSceneSnapshot
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import tf2_ros

from . import robot_model
from .conversions import (DEPTH_FLOAT_ENCODINGS, depth_to_metres, DEPTH_UINT16_ENCODINGS,
                          pose_matrix, target_capsule)
from .params import load_params, NodeParams, QOS_RELIABLE
from .primitives import object_primitives
from .scene_core import (build_scene, FrameInput, FramePoints, prepare_frame, SceneResult,
                         SelfGeometry, TargetCapsule)
from .timesync import JointStateBuffer

DEPTH_TOPIC = '/camera/depth/image_raw'
CAMERA_INFO_TOPIC = '/camera/color/camera_info'
CONFIDENCE_TOPIC = '/camera/depth/confidence'
JOINT_STATES_TOPIC = '/joint_states'
ROBOT_DESCRIPTION_TOPIC = '/robot_description'
MODELS_TOPIC = '/peach/target_model/models'
SNAPSHOT_SERVICE = '/peach/scene/build_snapshot'
APPLY_SERVICE = '/apply_planning_scene'
OBJECT_PREFIX = BuildSceneSnapshot.Request.HARD_OBJECT_PREFIX

_LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
_JOINT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
_CONFIDENCE_MATCH_S = 0.005
_POLL_S = 0.01
_CLEANUP_APPLY_TIMEOUT_S = 1.0


def _default_config() -> str:
    return os.path.join(get_package_share_directory('peach2_scene'), 'config', 'scene.yaml')


def _stamp_s(stamp: TimeMsg) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


def _resolve_mesh(uri: str) -> str | None:
    return robot_model.resolve_mesh_uri(uri, get_package_share_directory)


def _tf_matrix(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    return pose_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


@dataclass
class _Acquired:
    stamp: TimeMsg
    stamp_s: float
    frame: FrameInput
    tcp: np.ndarray | None


class _Rejected(Exception):
    """Frame unusable (reason in the message); the service tries the next frame."""


class SceneNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__('peach2_scene')
        self.declare_parameter('config_file', _default_config())
        self._params: NodeParams | None = None
        self._bridge = CvBridge()
        self._active = False
        self._state_lock = threading.Lock()
        # frame handoff: depth callback -> service
        self._frame_cond = threading.Condition()
        self._pending_after_s: float | None = None
        self._frames: deque = deque(maxlen=3)
        self._camera_info: CameraInfo | None = None
        self._confidence: deque = deque(maxlen=5)
        self._depth_rx = 0
        self._last_depth_s = math.nan
        self._joint_lock = threading.Lock()
        self._joints: JointStateBuffer | None = None
        self._robot_lock = threading.Lock()
        self._link_samples: dict[str, np.ndarray] = {}
        self._robot_problems: list[str] = []
        self._robot_ready = False
        self._targets_lock = threading.Lock()
        self._targets: list[TargetCapsule] = []
        self._targets_key: tuple = ()
        self._targets_skipped = 0
        # batch state, guarded by _scene_lock
        self._scene_lock = threading.Lock()
        self._batch: list[FramePoints] = []
        self._batch_tcp: np.ndarray | None = None
        self._batch_stamp: TimeMsg | None = None
        self._applied_key: tuple = ()
        self._written_ids: list[str] = []
        # diagnostics, guarded by _state_lock
        self._last_ok: bool | None = None
        self._last_error = ''
        self._last_result: SceneResult | None = None
        self._last_build_s = math.nan
        self._last_build_ms = math.nan
        self._apply_failures = 0
        self._tf_buffer: tf2_ros.Buffer | None = None
        self._tf_listener: tf2_ros.TransformListener | None = None
        self._subs: list = []
        self._srv = None
        self._apply_client = None
        self._bond = None
        self._diag = diagnostic_updater.Updater(self, period=2.0)
        self._diag.setHardwareID('peach2_scene')
        self._diag.add('scene', self._diagnose)

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        path = self.get_parameter('config_file').get_parameter_value().string_value
        try:
            params = load_params(path)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'configure rejected ({path}): {exc}')
            return TransitionCallbackReturn.FAILURE
        self._params = params
        with self._joint_lock:
            self._joints = JointStateBuffer(params.joint_buffer_s)
        with self._scene_lock:
            self._batch, self._batch_tcp, self._batch_stamp = [], None, None
            self._applied_key = ()
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=2, durability=DurabilityPolicy.VOLATILE,
            reliability=(ReliabilityPolicy.RELIABLE if params.image_qos_reliability == QOS_RELIABLE
                         else ReliabilityPolicy.BEST_EFFORT))
        images = MutuallyExclusiveCallbackGroup()
        self._subs = [
            self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, image_qos,
                                     callback_group=images),
            self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info,
                                     image_qos, callback_group=images),
            self.create_subscription(Image, CONFIDENCE_TOPIC, self._on_confidence, image_qos,
                                     callback_group=images),
            self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_states,
                                     _JOINT_QOS, callback_group=MutuallyExclusiveCallbackGroup()),
            self.create_subscription(String, ROBOT_DESCRIPTION_TOPIC, self._on_robot_description,
                                     _LATCHED, callback_group=MutuallyExclusiveCallbackGroup()),
            self.create_subscription(TargetModelArray, MODELS_TOPIC, self._on_models, _LATCHED,
                                     callback_group=MutuallyExclusiveCallbackGroup()),
        ]
        self._apply_client = self.create_client(ApplyPlanningScene, APPLY_SERVICE,
                                                callback_group=ReentrantCallbackGroup())
        self._srv = self.create_service(BuildSceneSnapshot, SNAPSHOT_SERVICE, self._on_snapshot,
                                        callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f'configured from {path}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        ret = super().on_activate(state)
        with self._state_lock:
            self._active = True
        self._bond = self._create_bond()
        return ret

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        with self._state_lock:
            self._active = False
        self._cancel_pending()
        self._destroy_bond()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._remove_written(_CLEANUP_APPLY_TIMEOUT_S)
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        with self._state_lock:
            self._active = False
        self._cancel_pending()
        self._destroy_bond()
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def _teardown(self) -> None:
        if self._srv is not None:
            self.destroy_service(self._srv)
            self._srv = None
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        if self._apply_client is not None:
            self.destroy_client(self._apply_client)
            self._apply_client = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        self._tf_buffer = None
        with self._joint_lock:
            self._joints = None
        with self._scene_lock:
            self._batch, self._batch_tcp, self._batch_stamp = [], None, None
            self._written_ids = []
            self._applied_key = ()
        with self._frame_cond:
            self._frames.clear()
            self._camera_info = None
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

    def _is_active(self) -> bool:
        with self._state_lock:
            return self._active

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def _on_depth(self, msg: Image) -> None:
        stamp_s = _stamp_s(msg.header.stamp)
        with self._frame_cond:
            self._depth_rx += 1
            self._last_depth_s = stamp_s
            if self._pending_after_s is not None and stamp_s > self._pending_after_s:
                self._frames.append(msg)
                self._frame_cond.notify_all()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._frame_cond:
            self._camera_info = msg

    def _on_confidence(self, msg: Image) -> None:
        with self._frame_cond:
            self._confidence.append((_stamp_s(msg.header.stamp), msg))

    def _on_joint_states(self, msg: JointState) -> None:
        with self._joint_lock:
            if self._joints is not None and len(msg.position) == len(msg.name):
                self._joints.add(_stamp_s(msg.header.stamp), list(msg.name), list(msg.position))

    def _on_robot_description(self, msg: String) -> None:
        p = self._params
        if p is None:
            return
        t0 = time.monotonic()
        try:
            samples, problems = robot_model.link_samples(msg.data, p.self_sample_spacing_m,
                                                         _resolve_mesh)
        except ValueError as exc:
            samples, problems = {}, [f'robot_description rejected: {exc}']
        with self._robot_lock:
            self._link_samples = samples
            self._robot_problems = problems
            self._robot_ready = bool(samples)
        n = sum(v.shape[0] for v in samples.values())
        log = self.get_logger().warning if problems or not samples else self.get_logger().info
        log(f'robot model: {len(samples)} links, {n} surface samples '
            f'({time.monotonic() - t0:.1f} s); problems: {problems or "none"}')

    def _on_models(self, msg: TargetModelArray) -> None:
        p = self._params
        if p is None:
            return
        capsules: list[TargetCapsule] = []
        skipped = 0
        for m in msg.models:
            frame = m.header.frame_id or msg.header.frame_id
            b, n = m.bottom.position, m.neck.position
            cap = target_capsule(m.target_id, m.bottom.valid, (b.x, b.y, b.z), m.neck.valid,
                                 (n.x, n.y, n.z), m.d95_m)
            if cap is None or frame != p.base_frame:
                skipped += 1
                continue
            capsules.append(cap)
        key = tuple(sorted((m.target_id, int(m.model_revision)) for m in msg.models
                           if any(c.target_id == m.target_id for c in capsules)))
        with self._targets_lock:
            self._targets = capsules
            self._targets_key = key
            self._targets_skipped = skipped
        if self._is_active():
            self._rebuild_from_batch()

    # ---------------------------------------------------------------- snapshot
    def _on_snapshot(self, req: BuildSceneSnapshot.Request,
                     resp: BuildSceneSnapshot.Response) -> BuildSceneSnapshot.Response:
        resp.success = False
        p = self._params
        if not self._is_active() or p is None:
            resp.message = 'peach2_scene not active'
            return resp
        with self._robot_lock:
            robot_ready, problems = self._robot_ready, list(self._robot_problems)
        if not robot_ready:
            resp.message = 'robot model not ready (/robot_description missing or unusable)'
            return self._fail(resp, req)
        if problems:
            resp.message = f'robot model incomplete: {problems}'
            return self._fail(resp, req)
        try:
            acquired = self._acquire(p)
        except _Rejected as exc:
            resp.message = str(exc)
            return self._fail(resp, req)
        resp.frame_stamp = acquired.stamp
        t0 = time.monotonic()
        try:
            prepared = prepare_frame(acquired.frame, p.scene)
        except ValueError as exc:
            resp.message = f'frame rejected: {exc}'
            return self._fail(resp, req)
        with self._scene_lock:
            batch = [] if req.clear_previous else list(self._batch)
            if len(batch) >= p.max_frames_per_batch:
                resp.message = (f'batch already holds {len(batch)} frames '
                                f'(max_frames_per_batch); call with clear_previous=true')
                return self._fail(resp, req)
            batch.append(prepared)
            targets, key = self._target_snapshot()
            result = build_scene(batch, targets, acquired.tcp, p.scene)
            ok, why = self._apply(result, acquired.stamp, p.apply_timeout_s)
            if ok:
                self._batch = batch
                self._batch_tcp = acquired.tcp
                self._batch_stamp = acquired.stamp
                self._applied_key = key
        self._record(ok, why, result, t0)
        resp.frame_age_s = max(self._now_s() - acquired.stamp_s, 0.0)
        if not ok:
            resp.message = why
            return self._fail(resp, req)
        resp.success = True
        resp.n_hard_objects = len(result.hard_objects)
        resp.n_soft_voxels = result.n_soft_voxels
        resp.truncated = bool(result.truncated)
        resp.n_frames = len(batch)
        resp.message = self._summary(result)
        self.get_logger().info(f'[{req.request_id}] snapshot: {resp.message} '
                               f'(frame age {resp.frame_age_s:.2f} s)')
        return resp

    def _fail(self, resp: BuildSceneSnapshot.Response,
              req: BuildSceneSnapshot.Request) -> BuildSceneSnapshot.Response:
        with self._state_lock:
            self._last_ok = False
            self._last_error = resp.message
        self.get_logger().warning(f'[{req.request_id}] snapshot failed: {resp.message}')
        return resp

    def _cancel_pending(self) -> None:
        with self._frame_cond:
            self._pending_after_s = None
            self._frames.clear()
            self._frame_cond.notify_all()

    def _acquire(self, p: NodeParams) -> _Acquired:
        """Wait for a usable frame stamped after now; raise _Rejected with the last reason."""
        deadline = time.monotonic() + p.frame_timeout_s
        with self._frame_cond:
            self._frames.clear()
            self._pending_after_s = self._now_s()
        last = 'no depth frame received'
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise _Rejected(f'no usable frame within {p.frame_timeout_s:.1f} s: {last}')
                with self._frame_cond:
                    if not self._frames:
                        self._frame_cond.wait(remaining)
                    if self._pending_after_s is None:
                        raise _Rejected('deactivated while waiting for a frame')
                    if not self._frames:
                        continue
                    msg = self._frames.popleft()
                    info = self._camera_info
                try:
                    return self._align(msg, info, p, deadline)
                except _Rejected as exc:
                    last = str(exc)
        finally:
            with self._frame_cond:
                self._pending_after_s = None
                self._frames.clear()

    def _align(self, msg: Image, info: CameraInfo | None, p: NodeParams,
               deadline: float) -> _Acquired:
        if info is None:
            raise _Rejected('no camera_info')
        if (info.width, info.height) != (msg.width, msg.height):
            raise _Rejected(f'camera_info {info.width}x{info.height} != depth '
                            f'{msg.width}x{msg.height} (depth must be registered)')
        K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            raise _Rejected('invalid intrinsics')
        if msg.encoding not in DEPTH_UINT16_ENCODINGS + DEPTH_FLOAT_ENCODINGS:
            raise _Rejected(f'unsupported depth encoding {msg.encoding!r}')
        camera_frame = p.camera_frame or msg.header.frame_id
        if not camera_frame:
            raise _Rejected('depth frame_id empty and camera_frame not set')
        stamp_s = _stamp_s(msg.header.stamp)
        joint = self._joint_at(stamp_s, p, deadline)
        if joint.max_speed > p.max_joint_speed_rad_s:
            raise _Rejected(f'arm moving at image stamp ({joint.max_speed:.3f} rad/s)')
        stamp = Time.from_msg(msg.header.stamp)
        T_cam = self._lookup(p.base_frame, camera_frame, stamp, p, deadline)
        robot = self._self_geometry(stamp, p, deadline)
        try:
            tcp = self._lookup(p.base_frame, p.tcp_frame, stamp, p, deadline)[:3, 3]
        except _Rejected:
            tcp = None
        depth = depth_to_metres(self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'),
                                msg.encoding, p.depth_unit_m)
        frame = FrameInput(depth, K, T_cam, robot, confidence=self._confidence_at(stamp_s, depth))
        return _Acquired(msg.header.stamp, stamp_s, frame, tcp)

    def _joint_at(self, stamp_s: float, p: NodeParams, deadline: float):
        while True:
            with self._joint_lock:
                joints = self._joints
                latest = joints.latest_stamp() if joints is not None else None
                sample = (joints.sample(stamp_s, p.joint_max_gap_s)
                          if latest is not None and latest >= stamp_s else None)
            if sample is not None:
                return sample
            if latest is not None and latest >= stamp_s:
                raise _Rejected('joint_states do not bracket the image stamp')
            if time.monotonic() >= deadline:
                raise _Rejected('no joint_states' if latest is None
                                else 'joint_states not yet past the image stamp')
            time.sleep(_POLL_S)

    def _lookup(self, target: str, source: str, stamp: Time, p: NodeParams,
                deadline: float) -> np.ndarray:
        timeout = max(min(p.tf_timeout_s, deadline - time.monotonic()), 0.0)
        try:
            tf = self._tf_buffer.lookup_transform(target, source, stamp,
                                                  timeout=Duration(seconds=timeout))
        except tf2_ros.TransformException as exc:
            raise _Rejected(f'TF {target}<-{source} at image stamp: {exc}') from exc
        return _tf_matrix(tf)

    def _self_geometry(self, stamp: Time, p: NodeParams, deadline: float) -> SelfGeometry:
        with self._robot_lock:
            samples = dict(self._link_samples)
        chunks = []
        for link, pts in samples.items():
            T = self._lookup(p.base_frame, link, stamp, p, deadline)
            chunks.append(pts @ T[:3, :3].T + T[:3, 3])
        return SelfGeometry(points=np.vstack(chunks) if chunks else np.zeros((0, 3)))

    def _confidence_at(self, stamp_s: float, depth: np.ndarray) -> np.ndarray | None:
        with self._frame_cond:
            cached = list(self._confidence)
        for t, msg in cached:
            if abs(t - stamp_s) <= _CONFIDENCE_MATCH_S:
                conf = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                return conf if conf.shape[:2] == depth.shape[:2] else None
        return None

    def _target_snapshot(self) -> tuple[list[TargetCapsule], tuple]:
        with self._targets_lock:
            return list(self._targets), self._targets_key

    def _rebuild_from_batch(self) -> None:
        """Target models changed: rebuild from the cached batch frames (never a new frame)."""
        p = self._params
        with self._scene_lock:
            targets, key = self._target_snapshot()
            if not self._batch or key == self._applied_key or p is None:
                return
            t0 = time.monotonic()
            result = build_scene(self._batch, targets, self._batch_tcp, p.scene)
            ok, why = self._apply(result, self._batch_stamp, p.apply_timeout_s)
            if ok:
                self._applied_key = key
        self._record(ok, why, result, t0)
        if ok:
            self.get_logger().info(f'rebuilt for target models: {self._summary(result)}')
        else:
            self.get_logger().warning(f'rebuild for target models failed: {why}')

    # ------------------------------------------------------------ planning scene
    def _collision_objects(self, result: SceneResult, stamp: TimeMsg | None,
                           remove: list[str]) -> tuple[list[CollisionObject], list[str]]:
        base = self._params.base_frame
        objects: list[CollisionObject] = []
        for oid in remove:
            co = CollisionObject()
            co.header.frame_id = base
            co.id = oid
            co.operation = CollisionObject.REMOVE
            objects.append(co)
        ids = []
        for i, obj in enumerate(result.hard_objects):
            co = CollisionObject()
            co.header.frame_id = base
            if stamp is not None:
                co.header.stamp = stamp
            co.id = f'{OBJECT_PREFIX}{i:04d}'
            co.pose.orientation.w = 1.0
            for prim in object_primitives(obj):
                sp = SolidPrimitive()
                sp.type = prim.shape_type
                sp.dimensions = [float(d) for d in prim.dimensions]
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (
                    float(v) for v in prim.position)
                q = prim.quat_xyzw
                pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
                pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])
                co.primitives.append(sp)
                co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD
            objects.append(co)
            ids.append(co.id)
        return objects, ids

    def _call_apply(self, objects: list[CollisionObject], timeout_s: float) -> tuple[bool, str]:
        client = self._apply_client
        if client is None:
            return False, 'not configured'
        if not client.wait_for_service(timeout_sec=min(timeout_s, 1.0)):
            return False, f'{APPLY_SERVICE} unavailable (move_group not running?)'
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.world.collision_objects = objects
        done = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            client.remove_pending_request(future)
            return False, f'{APPLY_SERVICE} timed out after {timeout_s:.1f} s'
        res = future.result()
        if res is None or not res.success:
            return False, f'{APPLY_SERVICE} returned success=false'
        return True, ''

    def _apply(self, result: SceneResult, stamp: TimeMsg | None,
               timeout_s: float) -> tuple[bool, str]:
        """REMOVE every previously written object and ADD the new ones in one diff."""
        old = list(self._written_ids)
        objects, ids = self._collision_objects(result, stamp, old)
        ok, why = self._call_apply(objects, timeout_s)
        if not ok and old and 'success=false' in why:
            # MoveIt reports false when a REMOVE names an object it no longer has (move_group
            # restarted); ADD replaces same-id objects, so resend without the removals.
            objects, ids = self._collision_objects(result, stamp, [])
            ok, why = self._call_apply(objects, timeout_s)
            if ok:
                self.get_logger().warning('planning scene lost our objects; re-added without '
                                          'REMOVE (stale ids beyond the new count may remain)')
        if ok:
            self._written_ids = ids
        else:
            with self._state_lock:
                self._apply_failures += 1
        return ok, why

    def _remove_written(self, timeout_s: float) -> None:
        with self._scene_lock:
            if not self._written_ids:
                return
            objects, _ = self._collision_objects(SceneResult([], np.zeros((0, 3)), False, {}),
                                                 None, list(self._written_ids))
            ok, why = self._call_apply(objects, timeout_s)
            if ok:
                self._written_ids = []
            else:
                self.get_logger().warning(f'cleanup could not remove scene objects: {why}')

    # ------------------------------------------------------------- diagnostics
    @staticmethod
    def _summary(result: SceneResult) -> str:
        s = result.stats
        return (f'{len(result.hard_objects)} hard objects'
                f'{" (truncated)" if result.truncated else ""}, {result.n_soft_voxels} soft '
                f'voxels, {s.get("frames", 0)} frames, {s.get("voxels_hard", 0)} hard voxels, '
                f'target/self/sparse voxels removed {s.get("voxels_target_removed", 0)}/'
                f'{s.get("voxels_self_removed", 0)}/{s.get("voxels_sparse_removed", 0)}')

    def _record(self, ok: bool, why: str, result: SceneResult, t0: float) -> None:
        with self._state_lock:
            self._last_ok = ok
            self._last_error = '' if ok else why
            self._last_result = result
            self._last_build_s = self._now_s()
            self._last_build_ms = 1e3 * (time.monotonic() - t0)

    def _diagnose(self, stat):
        lvl = diagnostic_msgs.msg.DiagnosticStatus
        with self._state_lock:
            active, last_ok, err = self._active, self._last_ok, self._last_error
            result, build_s, build_ms = self._last_result, self._last_build_s, self._last_build_ms
            apply_failures = self._apply_failures
        with self._robot_lock:
            robot_ready, problems = self._robot_ready, list(self._robot_problems)
            n_links = len(self._link_samples)
        with self._frame_cond:
            depth_rx, last_depth_s = self._depth_rx, self._last_depth_s
        with self._targets_lock:
            n_targets, skipped = len(self._targets), self._targets_skipped
        # The scene lock is held across ApplyPlanningScene waits; never block the default group.
        if self._scene_lock.acquire(blocking=False):
            try:
                n_frames, n_written = str(len(self._batch)), str(len(self._written_ids))
            finally:
                self._scene_lock.release()
        else:
            n_frames = n_written = 'busy'
        if not active:
            stat.summary(lvl.OK, 'inactive')
        elif not robot_ready or problems:
            stat.summary(lvl.ERROR, 'robot model not ready: self filter unavailable')
        elif last_ok is False:
            stat.summary(lvl.WARN, f'last snapshot failed: {err}')
        else:
            stat.summary(lvl.OK, 'idle' if last_ok is None else f'{n_written} objects written')
        now = self._now_s()
        stat.add('robot_links', str(n_links))
        stat.add('robot_problems', '; '.join(problems) or 'none')
        stat.add('depth_rx', str(depth_rx))
        stat.add('depth_age_s', f'{now - last_depth_s:.2f}' if math.isfinite(last_depth_s)
                 else 'never')
        stat.add('targets_hollowed', str(n_targets))
        stat.add('targets_skipped', str(skipped))
        stat.add('batch_frames', n_frames)
        stat.add('objects_written', n_written)
        stat.add('apply_failures', str(apply_failures))
        stat.add('last_error', err or 'none')
        stat.add('last_build_age_s', f'{now - build_s:.2f}' if math.isfinite(build_s) else 'never')
        stat.add('last_build_ms', f'{build_ms:.1f}')
        if result is not None:
            stat.add('truncated', str(result.truncated))
            stat.add('soft_voxels', str(result.n_soft_voxels))
            for k, v in sorted(result.stats.items()):
                stat.add(f'stats.{k}', str(v))
        return stat


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneNode()
    executor = MultiThreadedExecutor(num_threads=6)
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
