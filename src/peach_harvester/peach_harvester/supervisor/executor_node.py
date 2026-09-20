"""
采摘任务执行器：显式 RunHarvest，launch 从不自动开批.

Lifecycle Active 后才接受动作。选择与账本在本节点；感知不再提供
reset/complete/clear/reopen。批次态由 harvest_fsm.react 唯一推导。
"""
from __future__ import annotations

import json
import threading
import time
from typing import Optional

from action_msgs.msg import GoalStatus
import geometry_msgs.msg
from peach_common.lifecycle import break_bond, create_bond

from peach_interfaces.action import (
    BuildTargetModel, ExecuteTarget, MoveTo, RunHarvest, SurveyScene)
from peach_interfaces.msg import (
    CanonicalEvent,
    Enables,
    GraspDecision,
    HarvestState,
    PeachTargetObservationArray,
    SceneSnapshot,
    TargetOutcome,
)
from peach_interfaces.srv import (
    BeginScene,
    CheckReachability,
    ControlTask,
    FireStep,
    SetBatchPolicy,
    SetEnables,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros

from . import observe
from .batch import (
    apply_control,
    build_details,
    build_summary,
    classify_build_failure,
    default_ledger_root,
    full_failure_code,
    ledger_file,
    load_ledger,
    merge_cycle_extra,
    next_target,
    reach_queries,
    save_ledger,
    set_elapsed,
    stages_from_execute,
)
from .domain.reducer import BatchEvent, OrchestratorState, reduce_event
from .harvest_fsm import (
    apply_recovery_ack,
    BATCH_NAMES,
    canonical_code_for_outcome,
    Command,
    DISCOVERY,
    enter_recovery,
    Event,
    event_for_outcome,
    EventHold,
    MODE_AUTO,
    MODE_MAINTENANCE,
    MODE_PAUSED,
    permissions_for,
    react,
    Reaction,
    RUNNING,
    settle_terminal,
    WAITING_READY,
)
from .param_rules import check_enable_deps
from .params import SupervisorParams
from ..cycle_core.batch_policy import (
    BatchPolicy,
    ratio_reached,
    rework_kind,
    ReworkList,
    TargetDeadline,
)
from ..cycle_core.view_policy import ViewSignals


class TaskExecutorNode(LifecycleNode):
    """整栈调度：RunHarvest 串联导航/感知/臂；launch 从不自动开批."""

    _params: SupervisorParams
    """peach_supervisor.yaml 实时命名空间."""
    _batch_state: int
    """HarvestState.batch_state；只经 harvest_fsm / _apply 写入."""
    _target_phase: int
    """HarvestState.target_phase；只经 FSM 推导."""
    _paused: bool
    """操作台暂停请求已落地."""
    _cancel: bool
    """整批取消旗标."""
    _skip_target: bool
    """跳过当前目标（下一拍 FSM SKIP）."""
    _active: bool
    """Lifecycle Active."""
    _run_id: str
    """本批 request_id / HarvestState.run_id."""
    _current_target_id: str
    """调度当前目标；覆盖感知 selected."""
    _scene_epoch: int
    """BeginScene 世代."""
    _operation_mode: int
    """MODE_AUTO / PAUSED / MAINTENANCE."""
    _action_active: bool
    """RunHarvest action 正在执行."""
    _recovery_required: bool
    """接触故障，须 ACK."""
    _grasp_enabled: bool
    """套入使能（操作台覆盖或本地参数）."""
    _tool_enabled: bool
    """刀具使能."""
    _batch_policy: BatchPolicy
    """采收率/时限/视点档；goal 初值，SetBatchPolicy 可改."""
    _rework: Optional[ReworkList]
    """本批补采清单；批末落盘."""
    _target_deadline: Optional[TargetDeadline]
    """当前目标单果时限；None=不限."""
    _enables_override: dict
    """SetEnables 后的使能覆盖；空=跟随本地参数."""
    _observations: Optional[PeachTargetObservationArray]
    """最近一帧感知观测（选果数据源）."""
    _decision_cache: Optional[GraspDecision]
    """接触许可令牌缓存（装配 goal.clearance）."""
    _stack_ready: bool
    """managed_nodes_activated."""
    _harvest_busy: bool
    """RunHarvest execute 占用."""

    def __init__(self):
        """构造：状态与缓存初始化；ROS 实体在 on_configure 建."""
        super().__init__('peach_supervisor')
        self._lock = threading.Lock()
        self._state_seq = 0
        self._paused = False
        self._cancel = False
        self._skip_target = False
        self._active = False
        # W14：nav2_lm 进程死检心跳句柄（configure 建 / deactivate-cleanup 断）
        self._bond = None
        self._run_id = ''
        self._cycle_id = ''
        self._cycle_message = ''
        self._current_target_id = ''
        self._scene_epoch = 0
        self._outcomes = []
        self._outcome_details = []
        # 批次态初值声明（生命周期复位/开批另经 FSM 落地）；
        # 此后节点内不再直写 self._batch_state，唯一写入点在 _apply。
        self._batch_state: int = WAITING_READY
        self._target_phase = 0
        self._fsm_message = ''
        self._operation_mode = MODE_AUTO
        self._action_active = False
        self._recovery_required = False
        # ControlTask.reason 暂存：服务回调写入，随后第一个审计事件
        # （batch_paused / batch_resumed / recovery_acknowledged）消费并清空。
        self._control_reason = ''
        self._grasp_enabled = False
        self._tool_enabled = False
        self._progress = 0.0
        self._paused_batch = WAITING_READY
        self._event_hold = EventHold()
        self._action_generation = 0
        self._transaction_id = ''
        self._settled_transaction = ''
        self._cycle_plan_id = ''
        self._last_calibration_revision = ''
        self._last_config_revision = ''
        self._recovery_batch = WAITING_READY
        self._in_flight = []
        self._build_feedback = {'view_count': 0, 'status': '', 'started_s': 0.0}
        self._run_goal_handle = None
        self._harvest_busy = False
        self._run_started = 0.0
        self._discovered = 0
        self._ledger_loaded = False
        self._cycle_observe_extra = {}
        self._cycle_dispatch_t0 = 0.0
        # 批次命令循环内共享态（W6-A 表化：handler 间传递；单批串行，
        # _harvest_busy 门保证无并发批）
        self._claimed = set()
        self._empty_rounds = 0
        self._empty_limit = 1
        self._survey_goal = None
        self._survey_only = False
        self._run_enabled = False
        # state/feedback 广播节流（W6-A 限频）：上次发布时刻（monotonic）
        self._state_last_publish_s = 0.0
        self._observations: Optional[PeachTargetObservationArray] = None
        self._decision_cache: Optional[GraspDecision] = None
        # 批次策略（3c-2a：RunHarvest goal 初值；0=不限；fast 默认）
        self._batch_policy = BatchPolicy()
        self._rework: Optional[ReworkList] = None
        self._target_deadline: Optional[TargetDeadline] = None
        # 操作台使能覆盖（阶段 4）：None=跟随本地参数；SetEnables 后覆盖并
        # 广播 /peach/batch/enables（臂侧命令门为强制点；激活后 1Hz 心跳重发）
        self._enables_override: dict = {}
        self._enables_heartbeat_timer = None
        self._next_policy_default = None
        self._last_model_revision = ''
        self._stack_ready = False
        self._wake = threading.Event()
        self._cb = ReentrantCallbackGroup()
        from peach_harvester.supervisor.params import peach_supervisor as params_ns
        self._params = params_ns.attach(self)

    def on_configure(self, state):
        """配置转移：集中建 ROS 实体，失败回 ERROR 停在 Unconfigured."""
        try:  # 官方 LifecycleNode：configure 失败返回 ERROR，停在 Unconfigured.
            self._configure_ros()
        except Exception as exc:  # noqa: BLE001 接线失败（话题/参数非法）整包停走
            self.get_logger().error(f'configure 失败: {exc}')
            return TransitionCallbackReturn.ERROR
        # W14：nav2_lm 进程死检心跳（缺 ros-jazzy-bondpy 时守卫降级为 WARN）
        self._bond = create_bond(self, self.get_name())
        self.get_logger().info(
            'task executor configured; will not auto-start harvest')
        return super().on_configure(state)

    def _configure_ros(self) -> None:
        """集中创建 ROS 实体（发布器/订阅/客户端/动作/服务）."""
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_state = self.create_lifecycle_publisher(
            HarvestState, '~/state', latched)
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_event = self.create_lifecycle_publisher(
            CanonicalEvent, '~/events', event_qos)
        self._pub_snapshot = self.create_lifecycle_publisher(
            SceneSnapshot, '~/scene_snapshot', latched)
        self._sub_obs = self.create_subscription(
            PeachTargetObservationArray,
            '/peach/perception/target_observations',
            self._on_obs, 10, callback_group=self._cb)
        # 接触许可令牌缓存（3c-2a）：goal.clearance 装配来源——大脑订阅
        # 融合决策，臂侧仍按双路复检（令牌优先/快照回退），单一许可源不变。
        self._sub_decision = self.create_subscription(
            GraspDecision,
            '/peach/reconstruction/grasp_decision',
            self._on_decision, latched, callback_group=self._cb)
        self._begin = self.create_client(
            BeginScene, self._params.begin_scene_service,
            callback_group=self._cb)
        self._survey = ActionClient(
            self, SurveyScene,
            self._params.survey_scene_action,
            callback_group=self._cb)
        self._exec = ActionClient(
            self, ExecuteTarget,
            self._params.execute_target_action,
            callback_group=self._cb)
        self._build = ActionClient(
            self, BuildTargetModel,
            self._params.build_target_model_action,
            callback_group=self._cb)
        self._ack_recovery = self.create_client(
            Trigger,
            '/peach_arm/acknowledge_recovery',
            callback_group=self._cb)
        self._reach = self.create_client(
            CheckReachability, self._params.check_reachability_service,
            callback_group=self._cb)
        self._run_server = ActionServer(
            self, RunHarvest, '~/run_harvest',
            execute_callback=self._run_harvest,
            goal_callback=self._goal_if_active,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb)
        self._control_srv = self.create_service(
            ControlTask, '~/control', self._on_control,
            callback_group=self._cb)
        self._sub_stack = self.create_subscription(
            Bool, '/peach/lifecycle/managed_nodes_activated',
            self._on_stack_ready, latched, callback_group=self._cb)
        # fast 档观察（3c-2c）：TF 查相机位 + MoveTo 客户端直驱补视
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._move_to = ActionClient(
            self, MoveTo, '/peach_arm/move_to', callback_group=self._cb)
        # 操作台服务面（阶段 4）：使能开关（意图源=本节点，强制点=臂侧
        # 命令门）+ 批次策略运行期改 + 单步调试；使能变更经 latched
        # /peach/batch/enables 广播给臂侧覆盖本地参数。
        self._pub_enables = self.create_lifecycle_publisher(
            Enables, '/peach/batch/enables', latched)
        self._set_enables_srv = self.create_service(
            SetEnables, '~/set_enables', self._on_set_enables,
            callback_group=self._cb)
        # 使能心跳（缺心跳=故障）：override 非空且 Active 时 1Hz 重发，
        # 臂侧 enables_heartbeat_timeout_s 超时即回落本地参数权威。
        self._enables_heartbeat_timer = self.create_timer(
            1.0, self._heartbeat_enables, callback_group=self._cb)
        self._set_batch_policy_srv = self.create_service(
            SetBatchPolicy, '~/set_batch_policy', self._on_set_batch_policy,
            callback_group=self._cb)
        self._fire_step_srv = self.create_service(
            FireStep, '~/fire_step', self._on_fire_step,
            callback_group=self._cb)
        self._fire_step_seq = 0

    def _unconfigure_ros(self) -> None:
        """on_cleanup 释放全部 ROS 实体（与 _configure_ros 一一对应）."""
        try:
            for pub in (self._pub_state, self._pub_event, self._pub_snapshot):
                self.destroy_lifecycle_publisher(pub)
        except Exception:  # noqa: BLE001 已释放则忽略
            pass
        try:
            for client in (self._begin, self._ack_recovery):
                self.destroy_client(client)
            for client in (self._survey, self._exec, self._build):
                client.destroy()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.destroy_subscription(self._sub_obs)
            self.destroy_subscription(self._sub_decision)
            self.destroy_subscription(self._sub_stack)
        except Exception:  # noqa: BLE001
            pass
        try:
            self._run_server.destroy()
            self.destroy_service(self._control_srv)
        except Exception:  # noqa: BLE001
            pass
        try:
            self._move_to.destroy()
            self._tf_listener.unregister()
            self._tf_buffer.clear()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.destroy_lifecycle_publisher(self._pub_enables)
            if self._enables_heartbeat_timer is not None:
                self.destroy_timer(self._enables_heartbeat_timer)
                self._enables_heartbeat_timer = None
            self.destroy_service(self._set_enables_srv)
            self.destroy_service(self._set_batch_policy_srv)
            self.destroy_service(self._fire_step_srv)
        except Exception:  # noqa: BLE001
            pass

    def on_activate(self, state):
        """激活转移：置 Active 并复位到 FSM 初始批次态（立即发布）."""
        result = super().on_activate(state)
        self._active = True
        if self._bond is None:
            self._bond = create_bond(self, self.get_name())  # deactivate 后重臂
        # 生命周期复位到初始批次态（FSM 初值，非手写迁移）
        self._apply_state(WAITING_READY, force_publish=True)
        return result

    def on_deactivate(self, state):
        """去激活转移：清 Active 旗标（在途动作由取消/收口路径处理）."""
        self._active = False
        break_bond(self._bond)
        self._bond = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        """清理转移：清 Active 并释放全部 ROS 实体."""
        self._active = False
        break_bond(self._bond)
        self._bond = None
        self._unconfigure_ros()
        return super().on_cleanup(state)

    def _on_stack_ready(self, msg: Bool) -> None:
        """托管栈就绪订阅：缓存 managed_nodes_activated 供 goal 门判读."""
        self._stack_ready = bool(msg.data)

    def _on_decision(self, msg: GraspDecision) -> None:
        """接触许可令牌缓存（goal.clearance 装配源；心跳不续签语义不变）."""
        self._decision_cache = msg

    # ---- 操作台服务面（阶段 4）----

    def _execution_enabled_effective(self) -> bool:
        """读执行使能有效值：操作台 override 优先，未覆盖回落本地参数."""
        override = self._enables_override.get('execution')
        if override is not None:
            return bool(override)
        return bool(self._params.execution_enabled)

    def _publish_enables(self, reason: str) -> Enables:
        """按当前 override 组装并广播使能（latched；1Hz 心跳复用同一口）."""
        msg = Enables()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_name()
        msg.execution = bool(self._enables_override.get('execution', False))
        msg.grasp = bool(self._enables_override.get('grasp', False))
        msg.tool = bool(self._enables_override.get('tool', False))
        msg.reason = str(reason or '')
        self._pub_enables.publish(msg)
        return msg

    def _on_set_enables(self, request, response):
        """使能开关：意图源在此（广播覆盖臂侧本地参数），强制点在臂命令门."""
        # W2/S3：使能依赖链校验（execution→grasp→tool）——操作台不得越级
        # 开刀；violation 时拒绝且不落 override（旧版任何组合都广播）。
        violation = check_enable_deps(
            bool(request.execution), bool(request.grasp), bool(request.tool))
        if violation:
            response.accepted = False
            response.message = f'拒绝：{violation}'
            self.get_logger().warning(response.message)
            return response
        self._enables_override.update(
            execution=bool(request.execution),
            grasp=bool(request.grasp),
            tool=bool(request.tool),
            reason=str(request.reason or ''))
        msg = self._publish_enables(self._enables_override['reason'])
        self._emit(
            'enables_changed', '',
            details={'execution': msg.execution, 'grasp': msg.grasp,
                     'tool': msg.tool, 'reason': msg.reason})
        self.get_logger().info(
            f'操作台使能: execution={msg.execution} grasp={msg.grasp} '
            f'tool={msg.tool}（{msg.reason}）')
        self._publish_state()
        response.accepted = True
        response.message = '已广播 /peach/batch/enables'
        return response

    def _heartbeat_enables(self) -> None:
        """
        1Hz 重发操作台使能：臂侧缺心跳超时即回落本地参数权威.

        仅在操作台至少设过一次（override 非空）且节点 Active 时重发；
        从未设过使能的栈不发——臂侧本地参数保持唯一权威（旧栈兼容）。
        """
        if not self._active or not self._enables_override:
            return
        self._publish_enables(self._enables_override.get('reason', ''))

    def _on_set_batch_policy(self, request, response):
        """批次策略运行期改：当前批立即生效或作下批默认."""
        updates = {
            'target_harvest_ratio': max(
                0.0, float(request.target_harvest_ratio)),
            'per_target_timeout_s': max(
                0.0, float(request.per_target_timeout_s)),
            'sector_timeout_s': max(0.0, float(request.sector_timeout_s)),
            'view_policy': int(request.view_policy),
        }
        updated = self._batch_policy.with_updates(**updates)
        if bool(request.apply_to_current):
            self._batch_policy = updated
            response.message = '当前批已更新'
        else:
            self._next_policy_default = updated
            response.message = '将作为下批默认'
        self._emit('batch_policy_updated', '', details={
            **updates, 'apply_to_current': bool(request.apply_to_current)})
        response.accepted = True
        return response

    def _on_fire_step(self, request, response):
        """
        操作台单步（debug-only 占位面）：仅 PHOTO 已接线，走 MoveTo 拍照位.

        VIEWPOINT / BUILD / APPROACH / PREVIEW / TOOL_DEBUG 五个单步为
        占位——未接线，随后续轮补（当前请用 8090 调试面既有单步）；
        运动类最终都过臂侧命令门，不旁路。
        """
        self._fire_step_seq += 1
        step = int(request.step)
        if step == FireStep.Request.PHOTO:
            # W2/S3：运动类单步先过使能门（本地参数与操作台 override 的
            # 有效值都为关时拒绝——调试单步不得绕开 execution 使能）。
            if not self._execution_enabled_effective():
                self._emit(
                    'fire_step', '', details={
                        'step': 'photo', 'seq': self._fire_step_seq,
                        'accepted': False,
                        'rejected': 'execution_disabled'})
                response.accepted = False
                response.message = '拒绝：execution 未使能'
                response.request_seq = self._fire_step_seq
                return response
            goal = MoveTo.Goal()
            goal.kind = MoveTo.Goal.KIND_NAMED
            goal.named_target = 'global_photo_pose'
            moved = self._send_action(
                self._move_to, goal, 18.0, feedback=False,
                goal_handle=None)
            arrived = moved is not None and bool(
                getattr(moved, 'arrived', False))
            self._emit(
                'fire_step', '', details={
                    'step': 'photo', 'seq': self._fire_step_seq,
                    'arrived': arrived})
            response.accepted = arrived
            response.message = (
                '已到拍照位' if arrived else 'MoveTo 拍照位失败（见日志）')
        else:
            names = {
                FireStep.Request.VIEWPOINT: 'viewpoint',
                FireStep.Request.BUILD: 'build',
                FireStep.Request.APPROACH: 'approach',
                FireStep.Request.PREVIEW: 'preview',
                FireStep.Request.TOOL_DEBUG: 'tool_debug',
            }
            name = names.get(step, f'unknown_{step}')
            self._emit(
                'fire_step', '', details={
                    'step': name, 'seq': self._fire_step_seq,
                    'accepted': False})
            response.accepted = False
            response.message = (
                f'{name} 随后轮接线；当前请用 8090 调试面既有单步')
        response.request_seq = self._fire_step_seq
        return response

    def _poke(self) -> None:
        """打断批次 wait 循环（取消 / 暂停 / 观测 / 动作结束）."""
        self._wake.set()

    def _idle(self, timeout_s: float) -> None:
        """可被 _poke 提前结束的等待；替代 time.sleep 以免占满取消延迟."""
        self._wake.wait(timeout=max(timeout_s, 0.0))
        self._wake.clear()

    def _goal_if_active(self, goal_request):
        """放行或拒绝 RunHarvest goal：未激活/栈未就绪/已有批次则拒绝."""
        del goal_request
        with self._lock:
            if not self._active:
                return GoalResponse.REJECT
            if (bool(self._params.require_managed_stack)
                    and not self._stack_ready):
                self.get_logger().warning('RunHarvest 拒绝：生命周期栈未就绪')
                return GoalResponse.REJECT
            if self._harvest_busy:
                self.get_logger().warning('RunHarvest 拒绝：已有批次在跑')
                return GoalResponse.REJECT
            self._harvest_busy = True
            return GoalResponse.ACCEPT

    def _accept_cancel(self, cancel_request):
        """受理 RunHarvest 取消请求（实际收口走取消路径）."""
        del cancel_request
        return CancelResponse.ACCEPT

    def _on_obs(self, msg):
        """观测订阅：缓存最近一帧，持锁更新发现数并唤醒批次循环."""
        self._observations = msg
        if msg is not None:
            # 读-改-写须持锁（W6-A）：与 _run_harvest 复位/摘要读取并发
            with self._lock:
                self._discovered = max(
                    self._discovered, len(msg.observations))
        self._poke()

    def _on_control(self, request, response):
        """处理 ControlTask 命令：按批次权限放行，落 seq/暂停/取消/跳过."""
        cmd = int(request.command)
        if cmd == 6:
            return self._on_ack_recovery(request, response)
        with self._lock:
            allowed = permissions_for(
                self._batch_state, self._recovery_required,
                paused=self._paused)
            ok, seq, paused, cancel, skip = apply_control(
                self._state_seq, int(request.expected_state_seq),
                cmd, self._paused, allowed=allowed)
            if ok:
                self._state_seq = seq
                self._paused = paused
                # 契约（ControlTask.srv）：reason 是人读原因，写入事件
                self._control_reason = str(request.reason or '').strip()
                self._poke()
                if cmd == 2:
                    # 预留：MAINTENANCE 批次态与 EXIT_MAINTENANCE 命令未接线
                    self._operation_mode = MODE_MAINTENANCE
                elif cmd in (1, 3):
                    self._operation_mode = MODE_AUTO
                if cancel:
                    self._cancel = True
                    self._poke()
                if skip:
                    self._skip_target = True
                    self._poke()
            response.accepted = ok
            response.state_seq = self._state_seq
            if ok:
                response.message = 'ok'
            elif int(request.expected_state_seq) not in (0, self._state_seq):
                response.message = 'expected_state_seq mismatch'
            else:
                response.message = 'command not permitted'
        if response.accepted:
            response.state = self._publish_state(bump=False)
        else:
            response.state = self._make_state()
        return response

    def _on_ack_recovery(self, request, response):
        """ACK：技能确认成功后才加 state_seq，失败不消耗序号."""
        expected = int(request.expected_state_seq)
        with self._lock:
            allowed = permissions_for(
                self._batch_state, self._recovery_required,
                paused=self._paused)
            if expected != 0 and expected != self._state_seq:
                response.accepted = False
                response.message = 'expected_state_seq mismatch'
                response.state_seq = self._state_seq
                response.state = self._make_state()
                return response
            if 6 not in allowed:
                response.accepted = False
                response.message = 'command not permitted'
                response.state_seq = self._state_seq
                response.state = self._make_state()
                return response
        ack_ok, ack_msg = self._acknowledge_recovery()
        if not ack_ok:
            response.accepted = False
            response.message = ack_msg
            response.state_seq = self._state_seq
            response.state = self._make_state()
            return response
        with self._lock:
            self._state_seq += 1
        response.accepted = True
        response.message = ack_msg
        response.state_seq = self._state_seq
        response.state = self._publish_state(bump=False)
        # 人工操作审计：「真运动后须 ACK」这一安全纪律必须留在事件时间线上，
        # 否则事后无法从 jsonl 回放验证 ACK 时序。
        reason = str(request.reason or '').strip()
        self._emit(
            'recovery_acknowledged', self._run_id, self._current_target_id,
            details={'reason': reason} if reason else None)
        return response

    def _acknowledge_recovery(self) -> tuple:
        """转发技能 ~/acknowledge_recovery，成功才清批次恢复旗标."""
        timeout = min(
            5.0, float(self._params.service_timeout_s))
        # 非阻塞探测（服务不在时报错即回）；响应等待由 _await_future 有界完成。
        if not self._ack_recovery.service_is_ready():
            self.get_logger().error('acknowledge_recovery 不可用')
            return False, 'acknowledge_recovery unavailable'
        resp = self._await_future(
            self._ack_recovery.call_async(Trigger.Request()), timeout)
        if resp is None or not bool(getattr(resp, 'success', False)):
            message = str(getattr(resp, 'message', 'ack failed'))
            self.get_logger().error(f'acknowledge_recovery 失败: {message}')
            return False, message
        with self._lock:
            self._recovery_required = False
        return True, str(getattr(resp, 'message', 'ok'))

    def _apply(self, reaction, request_id: str, target_id: str = '',
               force_publish: bool = False) -> None:
        """落地 Reaction：唯一批次态写点，发布状态并按需发 canonical 事件."""
        self._batch_state = reaction.batch_state
        self._target_phase = reaction.target_phase
        self._fsm_message = reaction.message
        if reaction.operation_mode != MODE_AUTO:
            self._operation_mode = reaction.operation_mode
        self._publish_state(force=force_publish)
        if reaction.event_code:
            # 终局目标事件并入最近一条 outcome 的遥测细节（failure_code 等）：
            # 终局事件总在 _push_outcome 之后发出，否则 events/summary 只有
            # {"code": ...}，「失败可归因」在呈现层断链。
            details = None
            if (reaction.event_code in {
                    'target_succeeded', 'target_skipped', 'target_failed',
                    'target_canceled', 'target_operator_skipped'}
                    and self._outcome_details):
                details = dict(self._outcome_details[-1] or {})
            self._emit(
                reaction.event_code, request_id, target_id, details=details)

    def _apply_state(self, state: int,
                     force_publish: bool = False) -> None:
        """
        表外批次态迁移（暂停/恢复/恢复等待/终局）的统一落地.

        与 _apply 共用唯一写入点 reaction.batch_state：只换态并发布，
        不派命令、不发事件；state 一律来自 harvest_fsm 纯函数，
        节点内不得再对批次态做任何直写赋值。
        """
        self._apply(
            Reaction(state, self._target_phase, Command.NONE, '', ''),
            self._run_id, force_publish=force_publish)

    def _orch_state(self) -> OrchestratorState:
        """当前三维状态快照，供单一 reducer 使用."""
        return OrchestratorState(
            batch_state=self._batch_state,
            target_phase=self._target_phase,
            operation_mode=MODE_PAUSED if self._paused else self._operation_mode,
            recovery_latch=self._recovery_required,
            generation=int(self._action_generation),
            transaction_id=str(self._transaction_id or ''),
            session_id=str(self._run_id or ''),
            settled_transaction=str(self._settled_transaction or ''),
        )

    def _reaction_from(self, nxt: OrchestratorState, effects: list) -> Reaction:
        """把 reducer 输出写回世代/事务，再交给 _apply."""
        self._action_generation = int(nxt.generation)
        if nxt.transaction_id:
            self._transaction_id = nxt.transaction_id
        self._settled_transaction = str(nxt.settled_transaction or '')
        if effects:
            hit = effects[0]
            return Reaction(
                nxt.batch_state, nxt.target_phase, hit.command,
                hit.event_code, hit.message, nxt.operation_mode)
        return Reaction(
            nxt.batch_state, nxt.target_phase, Command.NONE, '',
            BATCH_NAMES.get(nxt.batch_state, ''), nxt.operation_mode)

    def _react(self, event: str) -> Reaction:
        """作业 phase 上的单一 reducer。暂停时先等恢复，禁止用 PAUSE_PENDING 覆盖."""
        if event == Event.CANCEL:
            self._event_hold = EventHold()
            nxt, effects = reduce_event(
                self._orch_state(),
                BatchEvent(Event.CANCEL, session_id=self._run_id))
            return self._reaction_from(nxt, effects)
        if self._paused:
            self._event_hold.apply_event(
                self._batch_state, event, paused=True)
            self._wait_pause()
            held = self._event_hold.take_after_pause(cancel=self._cancel)
            if self._cancel:
                return Reaction(
                    self._batch_state, self._target_phase, Command.NONE,
                    '', 'paused_cancel_discarded', MODE_AUTO)
            if held is not None:
                self._action_generation += 1
                self._transaction_id = f'{self._action_generation}:{event}'
                return held
        # 事件级事务 id（W2/S1 修复）：与暂停释放路径（上方 f'{gen}:{event}'）
        # 同一约定。终局结案只钉死「同一世代同一事件」的重复投递；若传节点
        # 级常驻 txn，首个终局会把 settled 钉成该常驻值，此后一切事件被
        # _stale 第三条款丢弃（CYCLE_DONE 无限重发的终局活锁）。
        nxt, effects = reduce_event(
            self._orch_state(),
            BatchEvent(
                event,
                transaction_id=f'{self._action_generation}:{event}',
                generation=self._action_generation,
                session_id=self._run_id))
        return self._reaction_from(nxt, effects)

    def _run_harvest(self, goal_handle):
        """执行 RunHarvest：开批复位 → 命令循环 → finally 收口落账."""
        goal = goal_handle.request
        self._run_goal_handle = goal_handle
        self._run_started = time.monotonic()
        with self._lock:
            self._cancel = False
            self._skip_target = False
            self._run_id = goal.request_id or 'harvest'
            self._cycle_id = self._run_id
            self._outcomes = []
            self._outcome_details = []
            self._scene_epoch = 0
            self._discovered = 0
            self._ledger_loaded = False
            self._in_flight = []
            self._operation_mode = MODE_AUTO
            self._event_hold = EventHold()
            self._settled_transaction = ''
            self._action_generation = 0
            self._transaction_id = ''
            # 批内共享态（W6-A 表化：handler 间传递）
            self._claimed = set()
            self._empty_rounds = 0
        # 批次策略与补采清单（3c-2a：跳过是调度参数不是失败；跳过目标
        # 自动入 rework_list.json 供人工补采，批末随账本落盘）
        self._batch_policy = BatchPolicy.from_goal({
            'target_harvest_ratio': getattr(goal, 'target_harvest_ratio', 0.0),
            'per_target_timeout_s': getattr(goal, 'per_target_timeout_s', 0.0),
            'sector_timeout_s': getattr(goal, 'sector_timeout_s', 0.0),
            'view_policy': getattr(goal, 'view_policy', 0),
        })
        if self._next_policy_default is not None:
            # SetBatchPolicy(apply_to_current=false) 攒下的下批默认
            self._batch_policy = self._next_policy_default
            self._next_policy_default = None
        self._rework = ReworkList(request_id=self._run_id)
        self._target_deadline = None
        # 开批复位并入 RUN_REQUESTED：从 FSM 初值 WAITING_READY 起跳，
        # 一次落地即发布 DISCOVERY（持锁内不得 _apply，_publish_state
        # 会再取非重入锁）。对外发布次数与旧「先复位后开批」一致。
        reaction = react(WAITING_READY, Event.RUN_REQUESTED)
        self._apply(reaction, goal.request_id)
        try:
            return self._run_harvest_body(goal_handle, goal, reaction)
        finally:
            self._current_target_id = ''
            self._action_active = False
            self._run_goal_handle = None
            # W6-B/S5：终局与异常路径统一在此落账（原 body 末次持久化移此，
            # 写序仍为 ledger → rework_list；幂等重写无内容差异）
            self._persist_ledger(self._claimed)
            if self._rework is not None and self._rework.entries:
                try:
                    out = self._rework.save(default_ledger_root())
                    self.get_logger().info(
                        f'补采清单已落盘：{out}（{len(self._rework.entries)} 项）')
                except OSError as error:
                    self.get_logger().warning(f'补采清单落盘失败: {error}')
            self._target_deadline = None
            with self._lock:
                self._harvest_busy = False
            # 批次收口态必达（突破限频）
            self._publish_state(force=True)

    def _run_harvest_body(self, goal_handle, goal, reaction):
        """
        批次命令循环；开批 Reaction 由 _run_harvest 传入，busy 旗标由其 finally 清.

        W6-A 表化：命令分发到 ``_COMMAND_HANDLERS``（行为与拆分前逐字
        一致——每分支的等待/事件/emit 顺序原样搬运）；handler 返回
        ``(reaction, persist)``，persist=False 对应原分支的 continue
        （跳过循环尾统一落账）。
        """
        result = RunHarvest.Result()
        self._empty_limit = max(1, int(self._params.empty_survey_limit))
        self._empty_rounds = 0
        # 使能快照在开批时刻取一次（原局部 enabled 语义不变）
        self._run_enabled = self._execution_enabled_effective()
        self._survey_only = (
            int(goal.intent) == int(RunHarvest.Goal.INTENT_SURVEY_ONLY))
        self._survey_goal = SurveyScene.Goal()
        self._survey_goal.request_id = goal.request_id
        self._survey_goal.scene_key = goal.scene_key
        terminal = {Command.SETTLE, Command.ABORT, Command.INTERRUPT}
        while reaction.command not in terminal:
            if goal_handle.is_cancel_requested:
                self._cancel = True
                self._poke()
            if self._cancel:
                reaction = self._react(Event.CANCEL)
                # 取消是关键迁移：突破限频立即发布
                self._apply(reaction, goal.request_id,
                            self._current_target_id, force_publish=True)
                self._cancel_inflight()
                break
            self._wait_pause()
            if self._cancel:
                continue
            cmd = reaction.command
            if cmd in (
                    Command.SELECT, Command.DISPATCH, Command.EXECUTE_FULL,
                    Command.NONE):
                self._wait_recovery()
                if self._cancel:
                    continue
            handler = self._COMMAND_HANDLERS.get(cmd)
            if handler is None:
                break
            reaction, persist = handler(self, goal, reaction)
            if persist:
                # 账本统一在每条命令收口后落盘一次（NONE 分支不再前置双写）
                self._persist_ledger(self._claimed)
        aborted = reaction.command == Command.ABORT
        interrupted = (
            reaction.command == Command.INTERRUPT or self._cancel)
        result.summary = build_summary(
            self._run_id, self._outcomes, self._discovered,
            time.monotonic() - self._run_started)
        no_product = (
            not aborted and not interrupted
            and int(result.summary.attempted) > 0
            and int(result.summary.succeeded) == 0)
        result.success = not aborted and not interrupted and not no_product
        if aborted:
            if reaction.message in (
                    'navigate_failed', 'begin_scene_failed', 'survey_failed'):
                result.termination_reason = reaction.message
            else:
                result.termination_reason = 'begin_scene_failed'
        elif interrupted:
            result.termination_reason = 'canceled'
        elif no_product:
            result.termination_reason = 'no_targets_succeeded'
        else:
            result.termination_reason = 'completed'
        if interrupted:
            goal_handle.canceled()
            # 终局/取消突破限频立即发
            self._publish_state(force=True)
        elif aborted:
            goal_handle.abort()
            self._publish_state(force=True)
        else:
            goal_handle.succeed()
            # 终局统一入口：COMPLETED 只经 settle_terminal 落地；仍在
            # 终态迁移之后发布，与旧序（赋值→succeed→发布）对外一致
            self._apply_state(settle_terminal(), force_publish=True)
        return result

    def _cmd_survey(self, goal, reaction):
        """SURVEY 分支（原循环分支逐字搬运）：SurveyScene + 首轮后断点恢复."""
        survey_ok = self._survey_body(self._survey_goal)
        if not self._ledger_loaded:
            claimed, restored, restored_details = self._restore_ledger(
                self._run_id)
            if restored:
                self._outcomes = restored
                # W6-B/S7：details 随账本回读（load_ledger 按 _EXTRA_KEYS
                # 回填），恢复批遥测细节不再归零成空 dict。
                self._outcome_details = restored_details
                # discovered 计数不随账本恢复（历史观测帧不可重放），维持
                # 复位值 0；在恢复事件里注明，防读账本时误读为「零发现」。
                self._emit(
                    'ledger_restored', goal.request_id,
                    details={
                        'claimed': len(claimed),
                        'outcomes': len(restored),
                        'discovered': 'not_restored'})
            self._claimed = claimed
            self._ledger_loaded = True
        if self._cancel:
            return reaction, False
        if not survey_ok:
            reaction = self._react(Event.SURVEY_FAILED)
        elif self._scene_epoch == 0:
            reaction = self._react(Event.SURVEY_AT_POSE)
        else:
            reaction = self._react(Event.SURVEY_DONE)
        self._apply(reaction, goal.request_id)
        return reaction, True

    def _cmd_wait_lock(self, goal, reaction):
        """WAIT_LOCK 分支（原循环分支逐字搬运）：等锁→快照→三路事件."""
        self._wait_lock()
        self._publish_scene_snapshot(self._survey_goal.scene_key)
        if self._cancel:
            return reaction, False
        if self._survey_only:
            reaction = self._react(Event.SURVEY_ONLY)
        elif not self._run_enabled:
            reaction = self._react(Event.EXECUTION_DISABLED)
        else:
            reaction = self._react(Event.LOCK_READY)
        self._apply(reaction, goal.request_id)
        return reaction, True

    def _cmd_select(self, goal, reaction):
        """SELECT 分支（原循环分支逐字搬运）：采收率门→联合约束选果→claim."""
        # 采收率门（3c-2a：跳过是调度参数不是失败——达标即收批，
        # 余果不再尝试，入补采清单）
        if ratio_reached(
                sum(1 for o in self._outcomes
                    if int(o.outcome) == int(TargetOutcome.SUCCEEDED)),
                int(self._discovered or 0), self._batch_policy):
            self._emit(
                'batch_ratio_satisfied', goal.request_id,
                details={
                    'target_harvest_ratio':
                        self._batch_policy.target_harvest_ratio})
            reaction = self._react(Event.EMPTY_LIMIT)
            self._apply(reaction, goal.request_id)
            return reaction, False
        # 联合约束选果：有效深度窗 ∩ 可达性（CheckReachability 把入口
        # 换成停位几何再 IK；服务不可用回退半径窗；超窗 targets_filtered）
        if not self._lock_set_ready():
            ik_results, ik_note = None, 'lock_set_not_ready'
            target_id, filtered = '', {}
        else:
            ik_results, ik_note = self._query_reachability(
                reach_queries(self._observations, self._claimed))
            target_id, filtered = next_target(
                self._observations, self._claimed, goal.target_ids,
                depth_range=(
                    float(self._params.selection_depth_min_m),
                    float(self._params.selection_depth_max_m)),
                ik_results=ik_results,
                fallback_reach_range=(
                    float(self._params.selection_reach_min_m),
                    float(self._params.selection_reach_max_m)))
        if filtered or (
                ik_note and ik_note != 'lock_set_not_ready'):
            details = {'filtered': filtered} if filtered else {}
            if ik_note:
                details['reach_check'] = ik_note
            self._emit(
                'targets_filtered', goal.request_id, details=details)
        if not target_id:
            self._empty_rounds += 1
            event = (
                Event.EMPTY_LIMIT
                if self._empty_rounds >= self._empty_limit
                else Event.NO_TARGET)
            reaction = self._react(event)
            self._apply(reaction, goal.request_id)
            return reaction, False
        self._empty_rounds = 0
        self._claimed.add(target_id)
        self._current_target_id = target_id
        self._cycle_id = f'{self._run_id}:{target_id}'
        if self._batch_policy.per_target_timeout_s > 0.0:
            self._target_deadline = TargetDeadline(
                time.monotonic(),
                self._batch_policy.per_target_timeout_s)
        reaction = self._react(Event.TARGET_SELECTED)
        self._apply(reaction, goal.request_id, target_id)
        return reaction, True

    def _cmd_cycle_done(self, goal, reaction):
        """NONE 分支（原循环分支逐字搬运）：本目标周期收口回选果."""
        reaction = self._react(Event.CYCLE_DONE)
        self._current_target_id = ''
        self._apply(reaction, goal.request_id)
        return reaction, True

    def _cmd_navigate(self, goal, reaction):
        """固定座直通：到位一步当 NAV_OK（NavigateToWorksite 预留）."""
        reaction = self._react(Event.NAV_OK)
        self._apply(reaction, goal.request_id)
        return reaction, True

    def _cmd_begin(self, goal, reaction):
        """BEGIN_SCENE 分支：调感知 BeginScene，成功读世代并清观测缓存."""
        begin = BeginScene.Request()
        begin.request_id = goal.request_id
        begin.scene_key = goal.scene_key
        resp = self._call_service(self._begin, begin)
        if resp is None or not bool(getattr(resp, 'accepted', False)):
            reaction = self._react(Event.BEGIN_FAILED)
            self._apply(reaction, goal.request_id)
            return reaction, True
        self._scene_epoch = int(getattr(resp, 'scene_epoch', 0) or 0)
        self._observations = None
        reaction = self._react(Event.BEGIN_OK)
        self._apply(reaction, goal.request_id)
        return reaction, True

    def _cmd_dispatch(self, goal, reaction):
        """DISPATCH 分支（原循环分支逐字搬运）：单果时限门 + 观察周期派发."""
        request_id = goal.request_id
        if self._target_deadline_exceeded(request_id):
            # 时限门不落 TargetOutcome（淘汰语义由单果时限自身决定）
            return self._fail_dispatch(
                self._react(Event.OBSERVE_FAILED), request_id,
                self._current_target_id)
        timeout = float(self._params.action_timeout_s)
        min_views = int(self._params.reconstruction_min_views)
        start_timeout = float(
            self._params.build_start_timeout_s)
        grace_s = float(self._params.observe_build_grace_s)
        target_id = self._current_target_id
        dispatch_t0 = time.monotonic()
        self._cycle_observe_extra = {}
        self._cycle_dispatch_t0 = 0.0
        if self._take_skip():
            return self._fail_dispatch(
                self._react(Event.SKIP), request_id, target_id,
                outcome=TargetOutcome.CANCELED, reason='skip_target',
                failure_code='canceled', dispatch_t0=dispatch_t0)
        if not self._wait_target_in_locked_set(target_id, 2.5):
            return self._fail_dispatch(
                self._react(Event.OBSERVE_FAILED), request_id, target_id,
                outcome=TargetOutcome.SKIPPED_QUALITY,
                reason='observe_failed: target_not_in_locked_set',
                failure_code='observe_failed', dispatch_t0=dispatch_t0)
        build_goal = BuildTargetModel.Goal()
        build_goal.request_id = request_id
        build_goal.target_id = target_id
        build_goal.scene_epoch = self._scene_epoch
        self._build_feedback = {
            'view_count': 0, 'status': '', 'started_s': time.monotonic()}
        build_handle = self._send_goal(
            self._build, build_goal, timeout,
            feedback_cb=self._on_build_feedback)
        if build_handle is None:
            return self._fail_dispatch(
                self._react(Event.BUILD_FAILED), request_id, target_id,
                outcome=TargetOutcome.SKIPPED_QUALITY,
                reason='build_target_model rejected',
                failure_code='build_rejected', dispatch_t0=dispatch_t0)
        self._in_flight.append(build_handle)
        if not self._wait_build_started(build_handle, start_timeout):
            self._cancel_handle(build_handle)
            # 单槽 Build：取消后必须等该动作结束再派下一颗（08-28 轮次 E
            # 教训：不等结束就派，重建会因单槽占用拒下一颗 Build，空等
            # action_timeout）。等待上限 10 s，超时由 _wait_result 自行收口。
            self._wait_result(
                build_handle, min(timeout, 10.0),
                goal_handle=self._run_goal_handle)
            self._forget_handle(build_handle)
            return self._fail_dispatch(
                self._react(Event.BUILD_FAILED), request_id, target_id,
                outcome=TargetOutcome.SKIPPED_QUALITY,
                reason='build_start_timeout: reconstruction not COLLECTING',
                failure_code='build_start_timeout', dispatch_t0=dispatch_t0,
                extra=build_details(self._build_feedback, dispatch_t0, None))
        # 视点策略开关（3c-2c）：fast=supervisor 直驱补视（单视优先封顶
        # 3 视）；conservative=现行多视观察（arm OBSERVE_ONLY 原值路径）。
        # plan_id 两档共用（W13-A：fast 档不发 arm 观察动作，但
        # _cmd_full 组装 FULL goal 时读 _cycle_plan_id，故保持在分支外）。
        self._cycle_plan_id = f'{request_id}:{target_id}:{self._action_generation}'
        fast_policy = (
            int(self._batch_policy.view_policy) == BatchPolicy.VIEW_FAST)
        if fast_policy:
            observe_ok, observe_details, observed = self._dispatch_fast(
                request_id, target_id)
        else:
            # OBSERVE goal 组装（W13-A 自无条件段移入本分支）：仅
            # conservative 档需要 arm 观察动作，fast 档跳过装配。
            observe = ExecuteTarget.Goal()
            observe.request_id = request_id
            observe.run_id = self._run_id
            observe.cycle_id = self._cycle_id
            observe.target_id = target_id
            observe.mode = ExecuteTarget.Goal.OBSERVE_ONLY
            observe.scene_epoch = int(self._scene_epoch or 0)
            observe.tool_profile_id = str(self._params.tool.profile_id)
            observe.generation = int(self._action_generation)
            observe.plan_id = self._cycle_plan_id
            observe_ok, observe_details, observed = self._dispatch_observe(
                target_id, observe, timeout)
        if self._cancel or self._peek_skip() or not observe_ok:
            self._take_skip()
            self._cancel_handle(build_handle)
            self._wait_result(
                build_handle, min(timeout, 10.0),
                goal_handle=self._run_goal_handle)
            if not observe_ok and not self._cancel:
                return self._fail_dispatch(
                    self._react(Event.OBSERVE_FAILED), request_id, target_id,
                    outcome=TargetOutcome.SKIPPED_QUALITY,
                    reason=(
                        str(getattr(observed, 'reason', '')
                            or 'observe_only failed')
                        if observed is not None else
                        'observe_only rejected (skills locked set)'),
                    failure_code='observe_failed', dispatch_t0=dispatch_t0,
                    extra=observe_details)
            return self._fail_dispatch(
                self._react(Event.SKIP), request_id, target_id,
                outcome=TargetOutcome.CANCELED, reason='canceled_or_skipped',
                failure_code='canceled', dispatch_t0=dispatch_t0,
                extra=observe_details)
        built, wait_kind = self._wait_build_after_observe(
            build_handle, timeout, grace_s, min_views)
        self._forget_handle(build_handle)
        details = build_details(self._build_feedback, dispatch_t0, built)
        details.update(observe_details)
        if self._cancel or self._take_skip():
            return self._fail_dispatch(
                self._react(Event.SKIP), request_id, target_id,
                outcome=TargetOutcome.CANCELED, reason='canceled_or_skipped',
                failure_code='canceled', dispatch_t0=dispatch_t0,
                extra=details)
        views = int(self._build_feedback.get('view_count') or 0)
        if wait_kind == 'observe_build_view_race' or (
                built is None and views < min_views):
            self._emit(
                'observe_build_view_race', request_id, target_id,
                details={
                    'view_count': views, 'min_views': min_views,
                    'timeout_source': 'observe_build_view_race',
                })
            return self._fail_dispatch(
                self._react(Event.BUILD_FAILED), request_id, target_id,
                outcome=TargetOutcome.SKIPPED_QUALITY,
                reason=(f'observe_build_view_race: views={views} '
                        f'min_views={min_views}'),
                failure_code='observe_build_view_race',
                dispatch_t0=dispatch_t0, extra=details)
        if built is None or not bool(getattr(built, 'success', False)):
            message = (
                'build_timeout:executor_wait' if built is None
                else str(getattr(built, 'message', '') or ''))
            failure_code, reason = classify_build_failure(built, message)
            return self._fail_dispatch(
                self._react(Event.BUILD_FAILED), request_id, target_id,
                outcome=TargetOutcome.SKIPPED_QUALITY, reason=reason,
                failure_code=failure_code, dispatch_t0=dispatch_t0,
                extra={**details, 'timeout_source': failure_code})
        model = getattr(built, 'model', None)
        self._last_model_revision = str(
            getattr(model, 'model_revision', '') or '')
        self._last_calibration_revision = str(
            getattr(model, 'calibration_revision', '') or '')
        self._last_config_revision = str(
            getattr(model, 'config_revision', '') or '')
        reaction = self._react(Event.READY_FULL)
        self._cycle_observe_extra = dict(details)
        self._cycle_dispatch_t0 = dispatch_t0
        self._apply(reaction, request_id, target_id)
        return reaction, True

    def _fail_dispatch(self, reaction, request_id: str, target_id: str, *,
                       outcome=None, reason: str = '',
                       failure_code: str = '', dispatch_t0: float = 0.0,
                       extra: dict | None = None):
        """
        DISPATCH 分支统一失败收尾（W13-A 收敛原 7 处收尾四连）.

        outcome 给 None 时只 _apply 不落账（如单果时限门：淘汰由时限
        本身决定，不走 TargetOutcome 账目）。

        Args:
            reaction: 已按失败事件求得的 Reaction（self._react 产物）.
            request_id: 本批请求 ID.
            target_id: 当前目标 ID.
            outcome: TargetOutcome.* 码；None 跳过 _record_skip.
            reason: 人读失败原因（落 outcome.reason）.
            failure_code: 稳定失败码（落遥测 details）.
            dispatch_t0: 本目标派发起始 monotonic 时刻（计 elapsed_s）.
            extra: 附加遥测字段（observe_details / build_details 等）.

        Returns
        -------
            (reaction, True)：分支终止元组，调用方直接 return.

        """
        if outcome is not None:
            self._record_skip(
                target_id, outcome, reason,
                failure_code=failure_code,
                elapsed_s=time.monotonic() - dispatch_t0,
                extra=extra)
        self._apply(reaction, request_id, target_id)
        return reaction, True

    def _dispatch_fast(self, request_id: str, target_id: str) -> tuple:
        """Fast 档观察（3c-2c）：supervisor 直驱补视，无 arm 观察结果."""
        observe_ok, observe_details = self._fast_observe_loop(
            request_id, target_id)
        return observe_ok, observe_details, None

    def _dispatch_observe(self, target_id: str, observe, timeout: float) -> tuple:
        """Conservative 档观察：arm OBSERVE_ONLY 原值路径（4 次重试）."""
        observed = None
        for attempt in range(4):
            if self._cancel or self._peek_skip():
                break
            observed = self._send_action(
                self._exec, observe, timeout, feedback=True,
                goal_handle=self._run_goal_handle)
            if observed is not None:
                break
            self.get_logger().warning(
                f'ExecuteTarget OBSERVE_ONLY rejected {target_id} '
                f'attempt={attempt + 1}/4')
            self._idle(0.4)
        observe_ok = (
            observed is not None
            and int(getattr(observed, 'outcome', 3)) == 0)
        observe_details = stages_from_execute(observed)
        return observe_ok, observe_details, observed

    def _cmd_full(self, goal, reaction):
        """EXECUTE_FULL 分支（原循环分支逐字搬运）：时限门 + 接触执行入账."""
        request_id = goal.request_id
        if self._target_deadline_exceeded(request_id):
            reaction = self._react(
                event_for_outcome(TargetOutcome.SKIPPED_QUALITY, False))
            self._apply(reaction, request_id, self._current_target_id)
            return reaction, True
        timeout = float(self._params.action_timeout_s)
        target_id = self._current_target_id
        t0 = time.monotonic()
        full = ExecuteTarget.Goal()
        full.request_id = request_id
        full.run_id = self._run_id
        full.cycle_id = self._cycle_id
        full.target_id = target_id
        full.mode = (
            ExecuteTarget.Goal.PREGRASP_ONLY
            if bool(self._params.execute_pregrasp_only)
            else ExecuteTarget.Goal.FULL)
        full.skip_observation = True
        # 清洁重写轮（3c-2a）：profile 档位显式下发（与 mode 等价衔接，
        # 臂侧 profile 优先）；接触许可令牌随 goal（臂侧双路复检：令牌
        # 优先/快照回退），装配自最新 GraspDecision 缓存。
        full.profile = (
            ExecuteTarget.Goal.PROFILE_PREGRASP_HOLD
            if bool(self._params.execute_pregrasp_only)
            else ExecuteTarget.Goal.PROFILE_FULL)
        decision = self._decision_cache
        # 令牌绑目标：缓存决策不绑定当前目标时整体不装配（臂侧走快照回退
        # 复检路径），旧目标的许可不得授权新目标（Clearance.target_id 契约）。
        if decision is not None and str(
                getattr(decision, 'target_id', '') or '') == target_id:
            full.clearance.target_id = target_id
            full.clearance.valid_until = decision.valid_until
            full.clearance.model_stamp = decision.header.stamp
            full.clearance.allowed = bool(decision.allowed)
            full.clearance.radial_margin_m = float(
                getattr(decision, 'radial_margin_m', 0.0) or 0.0)
            full.clearance.axial_margin_m = float(
                getattr(decision, 'axial_margin_m', 0.0) or 0.0)
            full.clearance.model_fingerprint = str(
                getattr(decision, 'model_revision', '') or '')
            full.clearance.reason = str(getattr(decision, 'reason', '') or '')
        full.scene_epoch = int(self._scene_epoch or 0)
        full.tool_profile_id = str(self._params.tool.profile_id)
        full.model_revision = str(self._last_model_revision or '')
        full.calibration_revision = str(self._last_calibration_revision or '')
        full.config_revision = str(self._last_config_revision or '')
        full.plan_id = str(self._cycle_plan_id or '')
        full.generation = int(self._action_generation)
        executed = self._send_action(
            self._exec, full, timeout, feedback=True,
            goal_handle=self._run_goal_handle)
        operator_skip = self._take_skip()
        outcome = TargetOutcome()
        outcome.target_id = target_id
        extra = merge_cycle_extra(
            self._cycle_observe_extra, stages_from_execute(executed))
        if executed is None:
            outcome.outcome = TargetOutcome.FAILED
            extra['failure_code'] = (
                'canceled' if operator_skip else 'full_failed')
            outcome.reason = (
                'skip_target' if operator_skip
                else 'full_failed: execute_target failed')
        else:
            record = getattr(executed, 'outcome_record', None)
            if record is not None and getattr(record, 'target_id', ''):
                outcome = record
            else:
                outcome.outcome = int(getattr(
                    executed, 'outcome', TargetOutcome.FAILED))
                outcome.reason = str(getattr(executed, 'reason', ''))
            if int(outcome.outcome) != int(TargetOutcome.SUCCEEDED):
                extra['failure_code'] = full_failure_code(
                    outcome, operator_skip)
                prefix = extra['failure_code']
                if prefix and not str(outcome.reason).startswith(prefix):
                    outcome.reason = (
                        f'{prefix}: {outcome.reason}').strip(': ')
            # W7：顶层镜像已删，harvest 证据单源 harvest/verification 块。
            harvest = getattr(executed, 'harvest', None)
            extra['harvest_confirmed'] = bool(
                getattr(harvest, 'grasped', False))
            extra['completion_level'] = int(
                getattr(executed, 'completion_level', 0) or 0)
            if int(getattr(executed, 'failure_code', 0) or 0):
                extra['failure_code_n'] = int(executed.failure_code)
            with self._lock:
                # ACK 清旗标（_acknowledge_recovery）与反馈置位并发，
                # 读-改-写须持锁，否则可能把已 ACK 的恢复门写回 True。
                self._recovery_required = self._recovery_required or bool(
                    getattr(executed, 'recovery_required', False))
        started = self._cycle_dispatch_t0 or t0
        set_elapsed(outcome, time.monotonic() - started)
        self._cycle_observe_extra = {}
        self._cycle_dispatch_t0 = 0.0
        self._push_outcome(outcome, extra)
        event = event_for_outcome(outcome.outcome, operator_skip)
        reaction = self._react(event)
        self._apply(reaction, request_id, target_id)
        return reaction, True

    # 命令处理器表（W6-A）：Command → 节点方法（统一签名
    # (goal, reaction) -> (Reaction, persist)）。RECORD_DISABLED 与未知
    # 命令不入表 → 循环 break（原 else 分支语义）。
    _COMMAND_HANDLERS = {
        Command.NAVIGATE: _cmd_navigate,
        Command.BEGIN_SCENE: _cmd_begin,
        Command.SURVEY: _cmd_survey,
        Command.WAIT_LOCK: _cmd_wait_lock,
        Command.SELECT: _cmd_select,
        Command.DISPATCH: _cmd_dispatch,
        Command.EXECUTE_FULL: _cmd_full,
        Command.NONE: _cmd_cycle_done,
    }

    # ---- fast 档观察（3c-2c）：supervisor 直驱补视，循环纯核在 observe.py ----

    def _view_signals(self, target_id: str) -> ViewSignals:
        """当前机位质量信号（W6-A 薄接线：纯核在 supervisor.observe）."""
        return observe.view_signals(self._observations, target_id)

    def _target_anchor(self, target_id: str):
        """目标锚点（W6-A 薄接线：纯核在 supervisor.observe）."""
        return observe.target_anchor(self._observations, target_id)

    def _camera_position(self):
        """相机位（W6-A 薄接线）：TF 查询闭包在节点侧，解析在纯核."""
        def lookup_exact(target, source, timeout_s):
            """按当前时刻查 TF（带超时；供纯核精确 stamp 查询回调）."""
            from rclpy.duration import Duration
            return self._tf_buffer.lookup_transform(
                target, source, self.get_clock().now().to_msg(),
                timeout=Duration(seconds=timeout_s))

        def lookup_latest(target, source):
            """按最新可用 TF 查（Time()=0；供纯核回退查询回调）."""
            from rclpy.time import Time
            return self._tf_buffer.lookup_transform(target, source, Time())

        now_s = self.get_clock().now().nanoseconds * 1e-9
        return observe.camera_position(
            lookup_exact, lookup_latest, now_s,
            warn=self.get_logger().warning)

    def _fast_move_to(self, position, quat, timeout_s: float):
        """补视移动（MoveTo KIND_POSE，camera_frame 直线）."""
        goal = MoveTo.Goal()
        goal.kind = MoveTo.Goal.KIND_POSE
        goal.camera_frame = True
        goal.lin_only = True
        goal.pose.header.frame_id = 'base_link'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(position[0])
        goal.pose.pose.position.y = float(position[1])
        goal.pose.pose.position.z = float(position[2])
        goal.pose.pose.orientation.x = quat[0]
        goal.pose.pose.orientation.y = quat[1]
        goal.pose.pose.orientation.z = quat[2]
        goal.pose.pose.orientation.w = quat[3]
        return self._send_action(
            self._move_to, goal, timeout_s, feedback=False,
            goal_handle=self._run_goal_handle)

    def _fast_observe_loop(self, request_id: str, target_id: str) -> tuple:
        """
        Fast 档观察循环薄接线（W6-A）：循环与判定在 supervisor.observe 纯核.

        返回 (observe_ok, details)；取消/跳过/几何缺失 → not ok。
        """
        del request_id  # 原签名保留（调用方兼容）；循环体不消费
        return observe.fast_observe_loop(
            target_id,
            signals=lambda: self._view_signals(target_id),
            camera_pos=self._camera_position,
            anchor=lambda: self._target_anchor(target_id),
            move=self._fast_move_to,
            canceled=lambda: self._cancel,
            skip_requested=self._peek_skip,
            warn=self.get_logger().warning,
            action_timeout_s=float(self._params.action_timeout_s),
            min_views=int(self._params.reconstruction_min_views))

    def _target_deadline_exceeded(self, request_id: str) -> bool:
        """单果时限门（0=不限）：超限记账 timeout 跳过并复位本果时限."""
        if (self._target_deadline is None
                or not self._target_deadline.exceeded()):
            return False
        target_id = self._current_target_id
        self._emit(
            'target_timeout', request_id, target_id,
            details={'per_target_timeout_s':
                     self._batch_policy.per_target_timeout_s})
        self._record_skip(
            target_id, TargetOutcome.SKIPPED_QUALITY,
            'per_target_timeout: 单果时限超限', failure_code='timeout')
        self._target_deadline = None
        return True

    def _record_skip(
            self, target_id: str, code: int, reason: str,
            failure_code: str = '', elapsed_s: float = 0.0,
            extra: dict | None = None) -> None:
        """跳过/失败落账：TargetOutcome + 遥测细节 + 补采清单挂钩."""
        outcome = TargetOutcome()
        outcome.target_id = target_id
        outcome.outcome = code
        outcome.reason = reason
        if elapsed_s:
            set_elapsed(outcome, elapsed_s)
        details = dict(extra or {})
        if failure_code:
            details['failure_code'] = failure_code
        self._push_outcome(outcome, details)
        # 补采清单挂钩（3c-2a）：跳过/失败目标入人工补采出口
        if self._rework is not None and target_id:
            self._rework.append(
                target_id,
                rework_kind(failure_code or str(reason), code),
                str(reason), attempted=True)

    def _push_outcome(self, outcome, extra=None) -> None:
        """将 outcomes 与遥测附加字段等长追加."""
        self._outcomes.append(outcome)
        self._outcome_details.append(dict(extra or {}))

    def _wait_build_after_observe(
            self, handle, timeout_s: float, grace_s: float, min_views: int):
        """
        OBSERVE 之后等 Build：机位未达 min_views 只给 grace_s，达线后用满超时.

        Returns
        -------
            (result, kind)：kind 为 '' / observe_build_view_race /
            build_timeout:executor_wait.

        """
        if handle is None:
            return None, 'build_timeout:executor_wait'
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda _: self._poke())
        started = time.monotonic()
        race_deadline = started + max(grace_s, 0.0)
        full_deadline = started + max(timeout_s, 0.0)
        while not result_fut.done():
            now = time.monotonic()
            views = int(self._build_feedback.get('view_count') or 0)
            if now >= full_deadline:
                self.get_logger().warning(
                    f'build wait timeout (executor_wait), views={views}')
                self._cancel_handle(handle)
                self._action_active = False
                return None, 'build_timeout:executor_wait'
            if views < min_views and now >= race_deadline:
                # rclpy logger 不支持 printf 风格多参数（曾致 execute 回调崩溃）
                self.get_logger().warning(
                    f'observe_build_view_race: views={views} < min_views={min_views}')
                self._cancel_handle(handle)
                self._action_active = False
                return None, 'observe_build_view_race'
            if self._cancel or self._peek_skip():
                self._cancel_handle(handle)
            self._idle(0.05)
        self._action_active = False
        try:
            wrapped = result_fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'action result failed: {exc}')
            return None, 'build_finalize_failed'
        return getattr(wrapped, 'result', wrapped), ''

    def _lock_set_ready(self) -> bool:
        """本世代锁定集才可 SELECT / CheckReachability."""
        obs = self._observations
        epoch = int(getattr(obs, 'scene_epoch', 0) or 0) if obs else 0
        return (
            obs is not None
            and epoch == int(self._scene_epoch or 0)
            and epoch > 0
            and bool(getattr(obs, 'target_set_locked', False)))

    def _wait_lock(self) -> None:
        """WAIT_LOCK：等到本世代锁定或 survey_wait_s。允许空集锁后 SELECT 走 NO_TARGET."""
        wait_s = float(self._params.survey_wait_s)
        deadline = time.monotonic() + max(wait_s, 0.0)
        while time.monotonic() < deadline and not self._cancel:
            self._wait_pause()
            if self._lock_set_ready():
                return
            self._idle(0.1)

    def _publish_scene_snapshot(self, scene_key: str) -> None:
        """WAIT_LOCK 结束或回访 dwell 后发布；scene_epoch 须为 Begin 之后."""
        snap = SceneSnapshot()
        snap.scene_epoch = self._scene_epoch
        snap.scene_key = scene_key
        snap.message = 'survey'
        obs = self._observations
        if obs is not None and self._lock_set_ready():
            snap.snapshot_id = str(obs.snapshot_id)
            snap.degraded = not obs.target_set_locked
            snap.observation_count = len(obs.observations)
            snap.target_ids = [
                item.target_id for item in obs.observations
                if item.target_id]
            self._discovered = max(self._discovered, snap.observation_count)
        else:
            snap.degraded = True
            if obs is not None:
                snap.snapshot_id = str(obs.snapshot_id)
                snap.observation_count = len(obs.observations)
        if hasattr(self, '_pub_snapshot'):
            self._pub_snapshot.publish(snap)

    def _survey_body(self, goal) -> bool:
        """SurveyScene：核 GoalStatus。首巡不等锁；回访 dwell 后再出快照."""
        timeout = float(self._params.action_timeout_s)
        self._action_active = True
        self._publish_state()
        result = None
        status = 0
        while not self._cancel:
            result, status = self._send_action(
                self._survey, goal, timeout, interrupt_on_pause=True,
                goal_handle=self._run_goal_handle, want_status=True)
            if self._paused and not self._cancel:
                self._wait_pause()
                continue
            break
        self._action_active = False
        if self._cancel:
            return False
        if result is None or status != GoalStatus.STATUS_SUCCEEDED:
            return False
        self._emit('photo_pose_reached', goal.request_id)
        if int(self._scene_epoch or 0) > 0:
            dwell = float(self._params.survey_dwell_s)
            deadline = time.monotonic() + max(dwell, 0.0)
            while time.monotonic() < deadline and not self._cancel:
                self._wait_pause()
                self._idle(0.1)
            self._publish_scene_snapshot(goal.scene_key)
        return not self._cancel

    def _send_goal(self, client, goal_msg, timeout_s: float,
                   feedback: bool = False, feedback_cb=None):
        """发 action goal：切片等服务器、取消可打断；被拒/超时给 None."""
        # W2/S2：服务器等待切片化（≤5s）+ 取消可打断——旧式单次
        # wait_for_server(action_timeout_s=180s) 纯阻塞不可取消，违仓规。
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or self._cancel:
                self.get_logger().warning('action server not ready')
                return None
            if client.wait_for_server(timeout_sec=min(5.0, remaining)):
                break
        kwargs = {}
        callback = feedback_cb
        if callback is None and feedback:
            callback = self._on_exec_feedback
        if callback is not None:
            kwargs['feedback_callback'] = callback
        send_fut = client.send_goal_async(goal_msg, **kwargs)
        handle = self._await_future(send_fut, timeout_s)
        if handle is None or not handle.accepted:
            tid = str(getattr(goal_msg, 'target_id', '') or '')
            self.get_logger().warning(
                f'action goal rejected target_id={tid or "-"}')
            return None
        self._action_active = True
        return handle

    def _wait_target_in_locked_set(
            self, target_id: str, timeout_s: float) -> bool:
        """等感知锁定集出现该 ID，再派 ExecuteTarget，避免技能空缓存秒拒."""
        if not target_id:
            return False
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while time.monotonic() < deadline:
            obs = self._observations
            if self._lock_set_ready() and obs is not None:
                for item in obs.observations:
                    tid = str(getattr(item, 'target_id', ''))
                    if tid == target_id and bool(
                            getattr(item, 'confirmed', False)):
                        return True
            if self._cancel or self._peek_skip():
                return False
            self._idle(0.05)
        self.get_logger().warning(
            f'target {target_id} not in locked set after {timeout_s:.1f}s')
        return False

    def _on_build_feedback(self, feedback_msg) -> None:
        """记录 BuildTargetModel 的 view_count/status 供 OBSERVE 收口核对."""
        feedback = getattr(feedback_msg, 'feedback', feedback_msg)
        self._build_feedback['view_count'] = int(
            getattr(feedback, 'view_count', 0) or 0)
        self._build_feedback['status'] = str(getattr(feedback, 'status', '') or '')
        self._poke()

    def _query_reachability(self, queries):
        """
        批量 TCP IK 预检（技能 CheckReachability：入口→停位后再 setFromIK）.

        返回 (ik_results, note)：ik_results 为 tid→(reachable, code)，
        服务不可用/超时/异常返回 (None, 原因说明)——调用方回退半径窗。
        空查询直接 (None, '')（无目标可检，next_target 无 IK 分支自然跳过）。
        """
        if not queries:
            return None, ''
        if not self._reach.service_is_ready():
            return None, 'service_unavailable'
        request = CheckReachability.Request()
        request.timeout_s = 0.1
        for _tid, (px, py, pz, qx, qy, qz, qw) in queries:
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = float(pz)
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            request.tcp_poses.append(pose)
        response = self._await_future(
            self._reach.call_async(request), 2.0)
        if response is None:
            return None, 'service_timeout'
        if len(response.reachable) != len(queries):
            return None, 'response_mismatch'
        results = {}
        for (tid, _pose), ok, code in zip(
                queries, response.reachable, response.error_codes):
            results[tid] = (bool(ok), str(code or ''))
        note = str(response.message or '')
        return results, note

    def _wait_build_started(self, handle, timeout_s: float) -> bool:
        """等 Build 绑定目标进入采集态；未确认前严禁发观察运动."""
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda _: self._poke())
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while time.monotonic() < deadline:
            if str(self._build_feedback.get('status') or '') in (
                    'COLLECTING', 'READY'):
                return True
            if result_fut.done() or self._cancel or self._peek_skip():
                return False
            self._idle(0.02)
        return False

    def _on_exec_feedback(self, feedback_msg) -> None:
        """镜像 ExecuteTarget 反馈：阶段/周期消息/恢复旗标并发布状态."""
        feedback = getattr(feedback_msg, 'feedback', feedback_msg)
        state = getattr(feedback, 'state', None)
        if state is None:
            return
        phase = int(getattr(state, 'target_phase', self._target_phase) or 0)
        if phase:
            self._target_phase = phase
        cycle = str(getattr(state, 'cycle_id', '') or '')
        if cycle:
            self._cycle_id = cycle
        feedback_message = str(getattr(state, 'message', '') or '')
        if feedback_message:
            self._cycle_message = feedback_message
        if bool(getattr(state, 'recovery_required', False)):
            with self._lock:
                self._recovery_required = True
        self._grasp_enabled = bool(getattr(state, 'grasp_enabled', False))
        self._tool_enabled = bool(getattr(state, 'tool_enabled', False))
        self._action_active = True
        self._publish_state()

    def _wait_result(
            self, handle, timeout_s: float,
            interrupt_on_pause: bool = False, goal_handle=None,
            want_status: bool = False):
        """等动作结果；可轮询 RunHarvest goal 的取消请求及时止损."""
        if handle is None:
            # 早退路径与正常收口同样清旗标，避免 HarvestState.action_active
            # 残留 True 直到下一个动作收口（反馈回调可能已置位）。
            self._action_active = False
            return (None, 0) if want_status else None
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda _: self._poke())
        deadline = time.monotonic() + max(timeout_s, 0.0)

        def _done(result, status=0):
            """Optionally pair the action result with GoalStatus."""
            return (result, status) if want_status else result

        while not result_fut.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning('action result timeout')
                self._cancel_handle(handle)
                return _done(None)
            if interrupt_on_pause and self._paused:
                self._cancel_handle(handle)
            if goal_handle is not None and goal_handle.is_cancel_requested:
                # RunHarvest 动作级取消：不等当前命令走完，立即走与
                # ControlTask 取消同一路径（置取消旗标并清 in-flight）
                self._cancel = True
                self._cancel_inflight()
            if self._cancel or self._peek_skip():
                self._cancel_handle(handle)
            self._idle(0.05)
        self._action_active = False
        try:
            wrapped = result_fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'action result failed: {exc}')
            return _done(None)
        status = int(getattr(wrapped, 'status', 0) or 0)
        return _done(getattr(wrapped, 'result', wrapped), status)

    def _send_action(self, client, goal_msg, timeout_s: float,
                     feedback: bool = False, interrupt_on_pause: bool = False,
                     goal_handle=None, want_status: bool = False):
        """一站式动作调用：_send_goal + _wait_result + 清 in-flight."""
        handle = self._send_goal(
            client, goal_msg, timeout_s, feedback=feedback)
        if handle is not None:
            self._in_flight.append(handle)
        result = self._wait_result(
            handle, timeout_s, interrupt_on_pause=interrupt_on_pause,
            goal_handle=goal_handle, want_status=want_status)
        self._forget_handle(handle)
        return result

    def _cancel_handle(self, handle) -> None:
        """取消单个 action goal（异常仅告警不抛）."""
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'cancel goal failed: {exc}')

    def _cancel_inflight(self) -> None:
        """取消全部在途 action 并清表."""
        for handle in list(self._in_flight):
            self._cancel_handle(handle)
        self._in_flight = []

    def _forget_handle(self, handle) -> None:
        """把已收口的 handle 移出在途表."""
        self._in_flight = [item for item in self._in_flight if item is not handle]

    def _take_skip(self) -> bool:
        """取走并清零「跳过当前目标」旗标（锁内）."""
        with self._lock:
            skip = self._skip_target
            self._skip_target = False
            return skip

    def _peek_skip(self) -> bool:
        """只读查看「跳过当前目标」旗标（不清零）."""
        with self._lock:
            return self._skip_target

    def _take_control_reason(self) -> dict:
        """取出并清空最近一次 ControlTask.reason，无则空 dict."""
        with self._lock:
            reason = self._control_reason
            self._control_reason = ''
        return {'reason': reason} if reason else {}

    def _ledger_path(self):
        """本批账本文件路径（default_ledger_root 下按 run_id）."""
        return ledger_file(default_ledger_root(), self._run_id)

    def _restore_ledger(self, run_id: str):
        """读回 (claimed, outcomes, details)；persist_ledger 关时空恢复."""
        if not bool(self._params.persist_ledger):
            return set(), [], []
        claimed, outcomes, details = load_ledger(
            ledger_file(default_ledger_root(), run_id))
        if claimed:
            self.get_logger().info(
                f'resume ledger {run_id}: {len(claimed)} claimed')
        return claimed, outcomes, details

    def _persist_ledger(self, claimed) -> None:
        """落盘批次账本（claimed/outcomes/details；开关关时空操作）."""
        if not bool(self._params.persist_ledger):
            return
        try:
            save_ledger(
                self._ledger_path(), claimed, self._outcomes,
                self._outcome_details)
        except OSError as exc:
            self.get_logger().warning(f'ledger write failed: {exc}')

    def _call_service(self, client, request):
        """调用服务：切片等可用、取消可打断；超时/取消给 None."""
        timeout = float(self._params.service_timeout_s)
        # W2/S2：等待切片化（≤5s）+ 取消可打断（旧式 30s 单次阻塞）。
        deadline = time.monotonic() + max(timeout, 0.0)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or self._cancel:
                self.get_logger().warning('service not ready')
                return None
            if client.wait_for_service(timeout_sec=min(5.0, remaining)):
                break
        fut = client.call_async(request)
        return self._await_future(fut, timeout)

    def _await_future(self, fut, timeout_s: float):
        """按 ≤1s 切片等 future：取消/超时给 None，异常仅告警."""
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        # W2/S2：future 按 ≤1s 切片等待，批取消时可提前退场（旧式单次
        # 整段 Event.wait 不可打断）。
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while not fut.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.get_logger().warning('future timeout')
                return None
            if done.wait(timeout=min(1.0, remaining)):
                break
            if self._cancel:
                self.get_logger().warning('future abandoned on cancel')
                return None
        try:
            return fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'future failed: {exc}')
            return None

    def _wait_pause(self) -> None:
        """暂停门：只改 operation_mode，不覆盖作业 batch_state."""
        entered = False
        while True:
            with self._lock:
                paused = self._paused
                cancel = self._cancel
                maintenance = self._operation_mode == MODE_MAINTENANCE
            if cancel or not paused:
                if entered:
                    if not maintenance:
                        self._operation_mode = MODE_AUTO
                    # 暂停/恢复是审计关键迁移：突破限频立即发布
                    self._publish_state(force=True)
                    self._emit(
                        'batch_resumed', self._run_id,
                        self._current_target_id,
                        details={
                            'to_state': BATCH_NAMES.get(
                                self._batch_state, ''),
                            **self._take_control_reason()})
                return
            if not entered:
                entered = True
                self._paused_batch = self._batch_state
                self._operation_mode = (
                    MODE_MAINTENANCE if maintenance else MODE_PAUSED)
                self._publish_state(force=True)
                self._emit(
                    'batch_paused', self._run_id,
                    self._current_target_id,
                    details={
                        'from_state':
                            BATCH_NAMES.get(self._paused_batch, ''),
                        **self._take_control_reason()})
            self._idle(0.1)

    def _wait_recovery(self) -> None:
        """技能接触锁未 ACK 时不派下一颗；CANCEL 可打断."""
        entered = False
        while True:
            with self._lock:
                recovery = self._recovery_required
                cancel = self._cancel
            if cancel or not recovery:
                if entered:
                    self._apply_state(
                        apply_recovery_ack(self._recovery_batch),
                        force_publish=True)
                return
            if not entered:
                entered = True
                self._recovery_batch = self._batch_state
                self._apply_state(
                    enter_recovery(self._recovery_batch),
                    force_publish=True)
                # 人工操作审计：进入恢复等待（真运动后停驻）进事件时间线
                self._emit(
                    'recovery_required', self._run_id,
                    self._current_target_id)
            self._idle(0.1)

    def _make_state(self) -> HarvestState:
        """组装 HarvestState 快照（seq/进度/权限/使能投影，不发布）."""
        msg = HarvestState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        seq = self._state_seq
        msg.revision = seq
        msg.state_seq = seq
        msg.run_id = self._run_id
        msg.cycle_id = self._cycle_id
        msg.target_id = self._current_target_id
        msg.operation_mode = self._operation_mode
        msg.batch_state = self._batch_state
        msg.target_phase = self._target_phase
        msg.action_active = self._action_active
        msg.auto_start_enabled = False
        # 投影统一（W6-A）：读有效使能（操作台 override 优先），消除
        # 本地参数/override 双口径投影失真
        msg.execution_enabled = self._execution_enabled_effective()
        msg.grasp_enabled = self._grasp_enabled
        msg.tool_enabled = self._tool_enabled
        msg.recovery_required = self._recovery_required
        attempted = len(self._outcomes)
        total = max(self._discovered, attempted, 1)
        msg.progress = float(attempted) / float(total)
        msg.message = BATCH_NAMES.get(self._batch_state, '')
        if self._batch_state == DISCOVERY and self._fsm_message:
            msg.message = self._fsm_message
        if self._batch_state == RUNNING and self._cycle_message:
            msg.message = self._cycle_message
        # blockers 字段保持消息默认空表（无写入方）
        msg.permissions = permissions_for(
            self._batch_state, self._recovery_required, paused=self._paused)
        msg.action_generation = int(getattr(self, '_action_generation', 0) or 0)
        msg.transaction_id = str(getattr(self, '_transaction_id', '') or '')
        msg.scene_epoch = int(self._scene_epoch or 0)
        return msg

    # state/feedback 广播节流下限（W6-A 限频）：高频调用方（_apply 每次
    # 迁移、_on_exec_feedback 每条反馈）限频到 10 Hz；终局/取消/暂停/
    # 恢复等稀疏关键迁移由调用方 force=True 突破节流立即发。topic 为
    # latched，晚订户仍取最后值，web 轮询不受影响；被节流的调用仍返回
    # 最新 _make_state（服务响应携带新 seq）。
    _STATE_PUBLISH_MIN_INTERVAL_S = 0.1

    def _publish_state(self, bump: bool = True, force: bool = False):
        """发布（限频）HarvestState 并回传；goal 活跃时同步发 RunHarvest 反馈."""
        if bump:
            with self._lock:
                self._state_seq += 1
        if not hasattr(self, '_pub_state'):
            return self._make_state()
        now = time.monotonic()
        with self._lock:
            throttled = (
                not force
                and now - self._state_last_publish_s
                < self._STATE_PUBLISH_MIN_INTERVAL_S)
            if not throttled:
                self._state_last_publish_s = now
        if throttled:
            return self._make_state()
        state = self._make_state()
        self._pub_state.publish(state)
        handle = self._run_goal_handle
        is_active = getattr(handle, 'is_active', False) if handle else False
        if callable(is_active):
            is_active = is_active()
        if handle is not None and is_active:
            try:
                feedback = RunHarvest.Feedback()
                feedback.state = state
                handle.publish_feedback(feedback)
            except Exception:  # noqa: BLE001
                pass
        return state

    def _emit(self, code: str, request_id: str, target_id: str = '',
              details: dict | None = None) -> None:
        """发 canonical 事件：终局码按最近 outcome 规范化，severity 按契约."""
        if code in {
            'target_succeeded', 'target_skipped', 'target_failed',
            'target_canceled', 'target_operator_skipped',
        }:
            # Prefer canonical mapping if last outcome exists.
            if self._outcomes:
                code = canonical_code_for_outcome(
                    self._outcomes[-1].outcome,
                    code == 'target_operator_skipped')
        event = CanonicalEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.code = code
        event.request_id = request_id
        event.run_id = self._run_id
        event.target_id = target_id
        event.state_seq = self._state_seq
        payload = {'code': code}
        if details:
            payload.update(details)
        event.message = json.dumps(payload, ensure_ascii=False)
        # severity 对齐 CanonicalEvent.msg 契约：AUDIT=人工操作审计，
        # ERROR=失败/中断，WARNING=可恢复告警，其余 INFO。
        if code in {
            'batch_paused', 'batch_resumed',
            'recovery_required', 'recovery_acknowledged',
        }:
            event.severity = CanonicalEvent.AUDIT
        elif code in {'survey_failed', 'target_failed', 'target_canceled'}:
            event.severity = CanonicalEvent.ERROR
        elif code == 'observe_build_view_race':
            event.severity = CanonicalEvent.WARNING
        if hasattr(self, '_pub_event'):
            self._pub_event.publish(event)


def main(args=None):
    """生命周期执行器入口."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = TaskExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
