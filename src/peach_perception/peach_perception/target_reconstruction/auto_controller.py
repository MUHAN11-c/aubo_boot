"""自动状态机驱动方法集（宿主契约见 capture 模块 docstring）."""
from __future__ import annotations

from peach_perception.target_reconstruction.capture_gate import GATE_ALLOW
from peach_perception.target_reconstruction.frame_collector import (
    STATE_COLLECTING,
    STATE_IDLE,
)
from peach_perception.target_reconstruction.refine import candidate_axis_hint
from peach_perception.target_reconstruction.skip_codes import classify_skip_reason


class AutoControllerMixin:
    """自动状态机驱动方法集（宿主契约见模块 docstring；不自带 __init__）."""

    def _auto_drive(self):
        """
        自动模式驱动：每个新同步帧回调末尾调用一次.

        流程：IDLE 且有候选 → 自动开始；COLLECTING → 满 max_views 自动
        finalize，否则尝试自动采帧；READY/FAILED 停采，等 reset/start 进
        下一轮。所有"不行"都只对当前帧跳过/告警，不打断流程。
        并发收敛：本方法可由 worker 线程（_process_rgbd）与 executor 线程
        （_on_target_observations）并发进入；collector/TSDF/产物读写全程
        持 _state_lock（RLock 允许锁内嵌套调 _finalize_now）。唯一例外是
        采帧的阻塞式 TF 查询：锁内 begin 采集判据 → 锁外查询（最长
        tf_timeout，期间服务/心跳可取锁）→ 锁内 finish 按 stamp 复核收口
        （见 _gated_capture_finish 竞态说明）。
        """
        with self._state_lock:
            if self.collector.state == STATE_IDLE and \
                    self.collector.should_auto_start():
                self._auto_start()
            if self.collector.state != STATE_COLLECTING:
                return
            if self.collector.should_auto_finalize():
                ok, message = self._finalize_now()
                if ok:
                    self.get_logger().info(f'自动完成：{message}')
                return
            if len(self.collector.frames) >= self.params.capture.max_views:
                # 满栈后静默等待 finalize，避免每帧重复构云/ICP和刷屏
                return
            decision, tf_request = self._gated_capture_begin(automatic=True)
        if tf_request is None:
            # 前置门禁已定案（skip），无需 TF 查询。
            if decision.reason:
                self._record_auto_skip(
                    decision.reason,
                    count_reject=decision.count_reject,
                    count_tf_failure=decision.count_tf_failure)
            return
        # 锁外：阻塞式 TF 查询（不得持 _state_lock）。
        tf_result = self._gated_capture_query_tf(tf_request)
        with self._state_lock:
            decision, context = self._gated_capture_finish(
                automatic=True, tf_request=tf_request, tf_result=tf_result)
            self._auto_capture_commit(decision, context)

    def _auto_start(self):
        """自动开始：绑定当前最优候选进 COLLECTING；无候选则保持 IDLE 等待."""
        target_id, center = self._best_candidate()
        if not target_id:
            self.get_logger().debug(
                '自动开始：无 initial_pose 候选，保持 IDLE')
            return
        message = self.collector.start(target_id, center)
        self._target_kind_memory.bind(target_id)
        self._last_captured_stamp_sec = -1.0
        self._reset_products(create_volume=True)
        self._bound_axis_hint = candidate_axis_hint(
            self._latest_candidates, target_id)
        self.get_logger().info(f'自动开始：{message}')
        self._publish_all()

    def _auto_capture_commit(self, decision, context) -> None:
        """
        自动采帧落地段（须持 _state_lock）：门禁结果 → 间隔/视角决策 → 建云.

        decision 非 GATE_ALLOW 即按 skip 跳过（按需计 tf_failures）；
        context 为 ALLOW 时的帧上下文（见 _gated_capture_finish）。
        """
        if decision.action != GATE_ALLOW:
            if decision.reason:
                self._record_auto_skip(
                    decision.reason,
                    count_reject=decision.count_reject,
                    count_tf_failure=decision.count_tf_failure)
            elif decision.count_tf_failure:
                self.collector.tf_failures += 1
            return
        (rgb, depth_mm, K, stamp_sec,
         T_base_camera, tf_status, target_mask) = context
        if self._last_captured_stamp_sec > 0.0:
            since_last = stamp_sec - self._last_captured_stamp_sec
        else:
            since_last = float('inf')  # 首帧不受间隔门限制
        action, reason = self.collector.auto_capture_decision(
            T_base_camera, since_last)
        if action != 'capture':
            self._record_auto_skip(reason)
            return
        accepted, message = self._accept_frame(
            rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status,
            target_mask=target_mask)
        if not accepted:
            self.collector.rejected_views += 1
            self._record_auto_skip(message, count_reject=False)
            self.get_logger().warning(f'自动采帧未入库：{message}')

    def _record_auto_skip(
            self, reason: str, *, count_reject: bool = False,
            count_tf_failure: bool = False) -> None:
        """自动 skip 落账：原因码计数 + 节流 WARN + harvest_data 事件."""
        code = classify_skip_reason(reason)
        self.collector.note_skip(code, reason)
        if count_tf_failure:
            self.collector.tf_failures += 1
        if count_reject:
            self.collector.rejected_views += 1
        self.get_logger().warning(
            f'自动采帧跳过 [{code}]：{reason}',
            throttle_duration_sec=1.0)
        self._harvest_data.append_event({
            'source': 'reconstruction', 'event': 'frame_skipped',
            'target_id': self.collector.target_id,
            'code': code, 'reason': reason,
        })
