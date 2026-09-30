"""
peach2_target_model lifecycle node: per-target multi-view fusion, GraspDecision, ObserveTarget.

Never commands motion or IO. Callback groups: observations (mutually exclusive), get_decision
(mutually exclusive), observe action (reentrant, goals block on a condition variable). Fusion
runs outside the store lock so get_decision only waits for short snapshot/commit sections.
"""
from __future__ import annotations

from collections import Counter
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
import diagnostic_msgs.msg
import diagnostic_updater
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import numpy as np
from peach2_core import codes
from peach2_core.tool import load_tool_calibration, load_tool_geometry
from peach2_interfaces.action import ObserveTarget
from peach2_interfaces.msg import (BagLandmark, GraspDecision, TargetModel, TargetModelArray,
                                   TargetObservation, TargetObservationArray)
from peach2_interfaces.srv import GetDecision
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from .decision import build_decision, Remeasure, remeasure_deviation
from .params import load_params, TargetModelParams
from .store import build_frame, fuse_job, ModelStore

MODELS_TOPIC = '/peach/target_model/models'
OBSERVATIONS_TOPIC = '/peach/perception/observations'
DECISION_SERVICE = '/peach/target_model/get_decision'
OBSERVE_ACTION = '/peach/target_model/observe'

_LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
_OBSERVATIONS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
_WAIT_SLICE_S = 0.2


def _default_config() -> str:
    return os.path.join(get_package_share_directory('peach2_target_model'), 'config',
                        'target_model.yaml')


def _stamp_s(stamp: TimeMsg) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


def _time_msg(t_s: float) -> TimeMsg:
    return Time(nanoseconds=int(round(t_s * 1e9))).to_msg()


def _landmark_tuple(lm: BagLandmark) -> tuple:
    p = lm.position
    return (bool(lm.valid), (p.x, p.y, p.z), tuple(lm.covariance), float(lm.confidence))


def _point(v) -> Point:
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _bag_landmark(position, cov, confidence: float) -> BagLandmark:
    out = BagLandmark()
    if position is None or cov is None or not np.all(np.isfinite(position)):
        return out
    out.valid = True
    out.source = BagLandmark.SOURCE_GEOMETRY
    out.position = _point(position)
    out.covariance = [float(x) for x in np.asarray(cov, dtype=np.float64).reshape(9)]
    out.confidence = float(confidence)
    return out


class TargetModelNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__('peach2_target_model')
        self.declare_parameter('config_file', _default_config())
        self._params: TargetModelParams | None = None
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._store: ModelStore | None = None
        self._active = False
        self._tools: dict[str, tuple] = {}
        self._tools_lock = threading.Lock()
        self._drops: Counter = Counter()
        self._decision_flags: Counter = Counter()
        self._last_fusion_s = math.nan
        self._last_fusion_ms = math.nan
        self._models_pub = None
        self._obs_sub = None
        self._decision_srv = None
        self._observe_server = None
        self._bond = None
        self._diag = diagnostic_updater.Updater(self, period=2.0)
        self._diag.setHardwareID('peach2_target_model')
        self._diag.add('target_model', self._diagnose)

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        path = self.get_parameter('config_file').get_parameter_value().string_value
        try:
            params = load_params(path, get_package_share_directory)
            for d in (params.description_config_dir, params.calibration_dir):
                if not os.path.isdir(d):
                    raise ValueError(f'directory not found: {d}')
        except (OSError, ValueError, LookupError) as exc:
            self.get_logger().error(f'configure rejected ({path}): {exc}')
            return TransitionCallbackReturn.FAILURE
        self._params = params
        with self._tools_lock:
            self._tools.clear()
        with self._lock:
            self._store = ModelStore(params)
            self._drops.clear()
            self._decision_flags.clear()
        self._models_pub = self.create_lifecycle_publisher(TargetModelArray, MODELS_TOPIC,
                                                           _LATCHED)
        self._obs_sub = self.create_subscription(
            TargetObservationArray, OBSERVATIONS_TOPIC, self._on_observations,
            _OBSERVATIONS_QOS, callback_group=MutuallyExclusiveCallbackGroup())
        self._decision_srv = self.create_service(
            GetDecision, DECISION_SERVICE, self._on_get_decision,
            callback_group=MutuallyExclusiveCallbackGroup())
        self._observe_server = ActionServer(
            self, ObserveTarget, OBSERVE_ACTION, execute_callback=self._execute_observe,
            goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'configured from {path}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        ret = super().on_activate(state)
        with self._lock:
            self._active = True
        self._publish_models()
        self._bond = self._create_bond()
        return ret

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        with self._cond:
            self._active = False
            self._cond.notify_all()
        self._destroy_bond()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        with self._cond:
            self._active = False
            self._cond.notify_all()
        self._destroy_bond()
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def _teardown(self) -> None:
        if self._observe_server is not None:
            self._observe_server.destroy()
            self._observe_server = None
        if self._decision_srv is not None:
            self.destroy_service(self._decision_srv)
            self._decision_srv = None
        if self._obs_sub is not None:
            self.destroy_subscription(self._obs_sub)
            self._obs_sub = None
        if self._models_pub is not None:
            self.destroy_lifecycle_publisher(self._models_pub)
            self._models_pub = None
        with self._lock:
            self._store = None

    def _create_bond(self):
        try:
            from bondpy import Bond
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

    # ------------------------------------------------------------ observations
    def _on_observations(self, msg: TargetObservationArray) -> None:
        with self._lock:
            if not self._active or self._store is None:
                return
            store, params = self._store, self._params
        frames = []
        drops: Counter = Counter()
        for obs in msg.observations:
            frame = self._frame_from(obs, msg.header.frame_id, params)
            if isinstance(frame, str):
                drops[frame] += 1
            else:
                frames.append((obs.target_id, frame))
        with self._cond:
            if store is not self._store:
                return
            self._drops.update(drops)
            cleared = store.set_epoch(int(msg.scene_epoch))
            for tid, frame in frames:
                store.add_frame(tid, frame)
            jobs = store.take_jobs()
        t0 = time.monotonic()
        outputs = [fuse_job(job, params) for job in jobs]
        with self._cond:
            if store is not self._store:
                return
            changed = store.commit(outputs)
            if jobs:
                self._last_fusion_s = self._now_s()
                self._last_fusion_ms = 1e3 * (time.monotonic() - t0)
            self._cond.notify_all()
        if changed or cleared:
            self._publish_models()

    def _frame_from(self, obs: TargetObservation, array_frame: str, params: TargetModelParams):
        if not obs.confirmed:
            return 'unconfirmed'
        if obs.category != TargetObservation.CATEGORY_BAG:
            return 'not_bag'
        if not obs.target_id:
            return 'no_target_id'
        frame_id = obs.header.frame_id or array_frame
        if frame_id != params.frame_id:
            return 'wrong_frame'
        a = obs.axis
        cp, cq = obs.camera_pose.position, obs.camera_pose.orientation
        return build_frame(_stamp_s(obs.header.stamp), _landmark_tuple(obs.bottom),
                           _landmark_tuple(obs.neck), _landmark_tuple(obs.tie),
                           (a.x, a.y, a.z), obs.diameter95_m, obs.camera_distance_m,
                           ((cp.x, cp.y, cp.z), (cq.x, cq.y, cq.z, cq.w)),
                           obs.swing_known, obs.swing_amplitude_m)

    # ---------------------------------------------------------------- messages
    def _model_msg(self, rec, stamp: TimeMsg) -> TargetModel:
        m = rec.fused
        out = TargetModel()
        out.header.stamp = _time_msg(rec.last_obs_s) if math.isfinite(rec.last_obs_s) else stamp
        out.header.frame_id = self._params.frame_id
        out.target_id = rec.target_id
        out.model_revision = rec.revision
        out.n_views = int(m.n_views)
        out.converged = bool(rec.converged)
        out.swing_amplitude_m = float(rec.swing_amplitude_m)
        out.fruit_top_offset_m = float(rec.fruit_top_offset_m)
        out.branch_direction_known = False
        if not m.ok:
            inf = float('inf')
            out.sigma_lateral95_m = out.sigma_axial95_m = out.theta95_deg = inf
            return out
        out.bottom = _bag_landmark(m.bottom, m.bottom_cov, m.inlier_ratio)
        out.neck = _bag_landmark(m.neck, m.neck_cov, m.inlier_ratio)
        out.tie = _bag_landmark(m.tie, m.neck_cov, m.inlier_ratio)
        out.axis = Vector3(x=float(m.axis[0]), y=float(m.axis[1]), z=float(m.axis[2]))
        out.d95_m = float(m.d95_m)
        out.length_m = float(m.length_m)
        out.sigma_lateral95_m = float(m.sigma_lateral95_m)
        out.sigma_axial95_m = float(m.sigma_axial95_m)
        out.theta95_deg = float(m.theta95_deg)
        out.fit_rmse_m = float(m.fit_rmse_m)
        out.inlier_ratio = float(m.inlier_ratio)
        return out

    def _publish_models(self) -> None:
        with self._lock:
            if self._store is None or self._models_pub is None:
                return
            records = self._store.records()
            pub = self._models_pub
        stamp = self.get_clock().now().to_msg()
        arr = TargetModelArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self._params.frame_id
        arr.models = [self._model_msg(r, stamp) for r in records]
        pub.publish(arr)

    # ------------------------------------------------------------ get_decision
    def _tool(self, tool_id: str):
        with self._tools_lock:
            cached = self._tools.get(tool_id)
        if cached is not None:
            return cached
        p = self._params
        geom = load_tool_geometry(tool_id, p.description_config_dir)
        calib = load_tool_calibration(tool_id, p.calibration_dir)
        with self._tools_lock:
            self._tools[tool_id] = (geom, calib)
        return geom, calib

    def _on_get_decision(self, req: GetDecision.Request,
                         resp: GetDecision.Response) -> GetDecision.Response:
        resp.found = False
        with self._lock:
            if not self._active or self._store is None:
                return resp
            rec = self._store.record(req.target_id)
            params = self._params
        if rec is None or not req.tool_id:
            return resp
        if req.min_model_revision and rec.revision < req.min_model_revision:
            return resp
        try:
            geom, calib = self._tool(req.tool_id)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'tool {req.tool_id!r} unavailable: {exc}',
                                    throttle_duration_sec=5.0)
            return resp
        now_s = self._now_s()
        d = build_decision(rec, geom, calib, params, now_s)
        with self._lock:
            self._decision_flags.update(d.flags)
        out = GraspDecision()
        out.header.stamp = _time_msg(now_s)
        out.header.frame_id = params.frame_id
        out.target_id = d.target_id
        out.model_revision = d.model_revision
        out.tool_id = d.tool_id
        out.valid_until = _time_msg(d.valid_until_s)
        out.approach_allowed = d.approach_allowed
        out.sleeve_allowed = d.sleeve_allowed
        out.cut_allowed = d.cut_allowed
        out.radial_margin_m = float(d.radial_margin_m)
        out.axial_margin_m = float(d.axial_margin_m)
        q = d.pregrasp_quat_xyzw
        out.pregrasp_tcp = Pose(position=_point(d.pregrasp_position),
                                orientation=Quaternion(x=float(q[0]), y=float(q[1]),
                                                       z=float(q[2]), w=float(q[3])))
        out.blade_target = _point(d.blade_target)
        out.insert_travel_m = float(d.insert_travel_m)
        out.failure_code = int(d.failure_code)
        out.reason = d.reason
        resp.found = True
        resp.decision = out
        return resp

    # ------------------------------------------------------------ observe action
    def _on_goal(self, goal: ObserveTarget.Goal) -> GoalResponse:
        with self._lock:
            ok = self._active and bool(goal.target_id)
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def _feedback(self, goal_handle, rec) -> None:
        fb = ObserveTarget.Feedback()
        fb.n_views = int(rec.fused.n_views)
        fb.sigma_lateral95_m = float(rec.fused.sigma_lateral95_m)
        fb.sigma_axial95_m = float(rec.fused.sigma_axial95_m)
        fb.theta95_deg = float(rec.fused.theta95_deg)
        goal_handle.publish_feedback(fb)

    def _finish(self, goal_handle, rec, code: int, succeeded: bool):
        result = ObserveTarget.Result()
        result.failure_code = int(code)
        if rec is not None:
            result.model = self._model_msg(rec, self.get_clock().now().to_msg())
            result.converged = bool(rec.converged)
        if code == codes.CANCELED and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif succeeded:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_observe(self, goal_handle):
        goal = goal_handle.request
        params = self._params
        t0 = self._now_s()
        deadline = t0 + params.observe_timeout_s
        if goal.neck_remeasure:
            return self._remeasure(goal_handle, goal.target_id, t0, deadline)
        max_views = int(goal.max_views) or params.default_max_views
        last_rev = None
        rec = None
        while True:
            with self._cond:
                if goal_handle.is_cancel_requested or not self._active or self._store is None:
                    return self._finish(goal_handle, rec, codes.CANCELED, False)
                rec = self._store.record(goal.target_id)
                if rec is None or rec.revision == last_rev:
                    self._cond.wait(_WAIT_SLICE_S)
            if rec is not None and rec.revision != last_rev:
                last_rev = rec.revision
                self._feedback(goal_handle, rec)
                if rec.converged:
                    return self._finish(goal_handle, rec, codes.NONE, True)
                if rec.fused.n_views >= max_views:
                    return self._finish(goal_handle, rec, codes.MODEL_NOT_CONVERGED, False)
            if self._now_s() >= deadline:
                code = codes.PERCEPTION_NO_TARGET if rec is None else codes.MODEL_NOT_CONVERGED
                return self._finish(goal_handle, rec, code, False)

    def _remeasure(self, goal_handle, target_id: str, t0: float, deadline: float):
        params = self._params
        with self._lock:
            rec = self._store.record(target_id) if self._store is not None else None
        if rec is None or not rec.fused.ok:
            code = codes.PERCEPTION_NO_TARGET if rec is None else codes.MODEL_NOT_CONVERGED
            return self._finish(goal_handle, rec, code, False)
        fused_neck = rec.fused.neck.copy()
        seen_any = False
        while True:
            with self._cond:
                if goal_handle.is_cancel_requested or not self._active or self._store is None:
                    return self._finish(goal_handle, rec, codes.CANCELED, False)
                frames = self._store.frames_since(target_id, t0)
                close = [f.landmarks.neck.position for f in frames
                         if f.camera_distance_m <= params.remeasure_max_camera_distance_m]
                if len(close) < params.remeasure_min_frames:
                    self._cond.wait(_WAIT_SLICE_S)
            seen_any = seen_any or bool(frames)
            if len(close) >= params.remeasure_min_frames:
                deviation = remeasure_deviation(fused_neck, close)
                passed = deviation <= params.remeasure_mismatch_m
                with self._lock:
                    if self._store is not None:
                        self._store.set_remeasure(
                            target_id, Remeasure.PASSED if passed else Remeasure.MISMATCH)
                        rec = self._store.record(target_id) or rec
                self.get_logger().info(
                    f'{target_id}: neck remeasure deviation {deviation * 1e3:.1f} mm '
                    f'({"pass" if passed else "MISMATCH"})')
                self._feedback(goal_handle, rec)
                if passed:
                    return self._finish(goal_handle, rec, codes.NONE, True)
                return self._finish(goal_handle, rec, codes.NECK_REMEASURE_MISMATCH, False)
            if self._now_s() >= deadline:
                code = (codes.PERCEPTION_LOW_QUALITY if seen_any
                        else codes.PERCEPTION_NO_TARGET)
                return self._finish(goal_handle, rec, code, False)

    # ------------------------------------------------------------- diagnostics
    def _diagnose(self, stat):
        level_ok = diagnostic_msgs.msg.DiagnosticStatus.OK
        with self._lock:
            active = self._active
            records = self._store.records() if self._store is not None else []
            epoch = self._store.epoch if self._store is not None else None
            drops = dict(self._drops)
            decision_flags = dict(self._decision_flags)
            last_s, last_ms = self._last_fusion_s, self._last_fusion_ms
        reasons = Counter(r for rec in records for r in rec.unconverged)
        n_conv = sum(1 for r in records if r.converged)
        if not active:
            stat.summary(level_ok, 'inactive')
        else:
            stat.summary(level_ok, f'{len(records)} targets, {n_conv} converged')
        stat.add('scene_epoch', str(epoch))
        stat.add('targets', str(len(records)))
        stat.add('converged', str(n_conv))
        age = self._now_s() - last_s if math.isfinite(last_s) else math.nan
        stat.add('last_fusion_age_s', f'{age:.2f}')
        stat.add('last_fusion_ms', f'{last_ms:.2f}')
        for reason, n in sorted(reasons.items()):
            stat.add(f'unconverged.{reason}', str(n))
        for reason, n in sorted(drops.items()):
            stat.add(f'dropped.{reason}', str(n))
        for flag, n in sorted(decision_flags.items()):
            stat.add(f'decision_flags.{flag}', str(n))
        stat.add('swing_unknown', str(sum(1 for r in records
                                          if not math.isfinite(r.swing_amplitude_m))))
        remeasure = Counter(r.remeasure.value for r in records)
        for state, n in sorted(remeasure.items()):
            stat.add(f'remeasure.{state}', str(n))
        return stat


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetModelNode()
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
