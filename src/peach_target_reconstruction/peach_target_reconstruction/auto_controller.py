"""
自动状态机驱动 mixin — 自动绑定/采帧/完成的节点内决策段.

职责边界：PeachReconstructionNode 自动模式（capture.auto_mode=true）的
驱动逻辑：IDLE 绑定最优候选进 COLLECTING（_auto_start）、逐帧采帧决策
落地（_auto_capture_commit）、满栈/满足条件自动 finalize（_auto_drive
编排）。手动 Trigger 服务流与采帧门禁三段式（_gated_capture_*）留在
reconstruction_node.py，本模块只在锁内消费其结果。

线程模型：_auto_drive 可由 worker 线程（_process_rgbd 帧处理末尾）与
executor 线程（_on_target_observations 回调）并发进入；collector/TSDF/
产物读写全程持节点 _state_lock（RLock，允许锁内嵌套 _finalize_now）。
唯一锁外段：采帧门禁的阻塞式精确时刻 TF 查询（_gated_capture_begin →
_query_tf → _finish 三段式，最长 tf_timeout），查询期间锁空闲，
Trigger 服务/观测回调/心跳不被堵。

宿主契约（mixin，PeachReconstructionNode 提供）：
  状态 collector/params/_state_lock/_target_kind_memory/
  _latest_candidates/_last_captured_stamp_sec/_bound_axis_hint；
  方法 _best_candidate()/_reset_products()/_finalize_now()/
  _gated_capture_begin()/_gated_capture_query_tf()/_gated_capture_finish()/
  _accept_frame()/_publish_all()。
"""
from __future__ import annotations

from peach_target_reconstruction.candidate_contract import candidate_axis_hint
from peach_target_reconstruction.capture_gate import GATE_ALLOW
from peach_target_reconstruction.frame_collector import (
    STATE_COLLECTING,
    STATE_IDLE,
)


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
                self.get_logger().debug(f'自动采帧跳过：{decision.reason}')
            return
        # 锁外：阻塞式 TF 查询（不得持 _state_lock）。
        tf_result = self._gated_capture_query_tf(tf_request)
        with self._state_lock:
            decision, context = self._gated_capture_finish(
                automatic=True, tf_request=tf_request, tf_result=tf_result)
            self._auto_capture_commit(decision, context)

    def _auto_start(self):
        """自动开始：绑定当前最优候选进 COLLECTING；无候选静默等待."""
        target_id, center = self._best_candidate()
        if not target_id:
            return  # 无候选：静默等待（initial_pose 到位后自然触发）
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
            if decision.count_tf_failure:
                self.collector.tf_failures += 1
            if decision.reason:
                self.get_logger().debug(f'自动采帧跳过：{decision.reason}')
            return
        (rgb, depth_mm, K, stamp_sec,
         T_base_camera, tf_status, target_mask) = context
        if self._last_captured_stamp_sec > 0.0:
            since_last = stamp_sec - self._last_captured_stamp_sec
        else:
            since_last = float('inf')  # 首帧不受间隔门限制
        action, reason = self.collector.auto_capture_decision(
            T_base_camera, since_last)
        if action == 'skip':
            self.get_logger().debug(f'自动采帧跳过：{reason}')
            return
        if action == 'warn_capture':
            self.get_logger().warning(f'自动采帧：{reason}')
        accepted, message = self._accept_frame(
            rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status,
            target_mask=target_mask)
        if not accepted:
            self.collector.rejected_views += 1
            self.get_logger().warning(f'自动采帧未入库：{message}')
