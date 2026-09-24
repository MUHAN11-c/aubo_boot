"""E1 感知注入器（perception_sim）——阶段三核心件.

对照协议（S0 尖峰定论，决策 0029⑤）：
- 感知节点观测发布纯帧驱动、相机关即静默 → 注入器在
  /peach/perception/target_observations 上是单发布者；
- 锁判定只查 epoch 相等∧>0∧target_set_locked（executor_node._lock_set_ready），
  不查 run_id/selected → 从 latched /peach_supervisor/state 对齐 epoch/run_id；
- robot_status 10Hz 仿 sim_field_targets（mock 下 launch 已置
  require_robot_status=false，保持同构防回退）。

两段式观测（PeachTargetObservationArray 头注释契约）：锁定前 observations 恒空、
只推进 collecting_count；锁定后 target_set_locked=true + 固定 ID 集。

零 ROS 纯核（Scenario/Timeline）与 rclpy 壳分离：纯核可单测。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# 纯核：场景脚本（零 ROS）
# ---------------------------------------------------------------------------

@dataclass
class SimTarget:
    """单目标：几何取 base 系（与 perception_constraint_grid 同口径）."""
    target_id: str
    entry_xyz: tuple
    axis: tuple
    camera_distance_m: float = 0.85
    confidence: float = 0.95


@dataclass
class Scenario:
    """一轮注入脚本：进度期 → 锁定 → 锁定后行为（阶段四扩展 flicker/lost）."""
    targets: list
    progress_s: float = 1.5        # 进度期时长（collecting_count 爬升）
    hold_locked_s: float = 600.0   # 锁定后持续发布
    decision_allowed: bool = True  # grasp_decision 汇总位（C03+ 用）


def observe_at(scenario: Scenario, t0: float, now: float, epoch: int,
               run_id: str, snapshot_id: int = 1):
    """纯核：t0 起的时刻 → 数组级发布内容（dict，壳层负责填 IDL）."""
    elapsed = now - t0
    if elapsed < 0:
        return {'locked': False, 'collecting': 0, 'observations': []}
    if elapsed < scenario.progress_s:
        # 进度期：collecting_count 单调爬升（不承诺线性，只要不减）
        frac = elapsed / max(scenario.progress_s, 1e-6)
        n = min(len(scenario.targets),
                int(frac * len(scenario.targets)) + (1 if frac > 0 else 0))
        n = max(0, min(n, len(scenario.targets)))
        return {'locked': False, 'collecting': n, 'observations': []}
    if elapsed < scenario.progress_s + scenario.hold_locked_s:
        return {
            'locked': True,
            'collecting': len(scenario.targets),
            'observations': [
                {
                    'target_id': t.target_id,
                    'confirmed': True,
                    'entry_xyz': t.entry_xyz,
                    'axis': t.axis,
                    'camera_distance_m': t.camera_distance_m,
                    'confidence': t.confidence,
                } for t in scenario.targets],
            'snapshot_id': snapshot_id,
            'epoch': epoch,
            'run_id': run_id,
        }
    return None  # 停发（阶段四 C20 断流脚本用）


# ---------------------------------------------------------------------------
# rclpy 壳：测试进程内节点
# ---------------------------------------------------------------------------

class PerceptionSim:
    """launch_testing 用注入节点（外部 executor spin，避免内部线程）."""

    def __init__(self, node):
        import rclpy.qos as qos
        from aubo_msgs.msg import RobotStatus
        from peach_interfaces.action import RunHarvest
        from peach_interfaces.msg import HarvestState
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.node = node
        self.state = None            # 最近 HarvestState
        self.ledger_events = []      # /peach_supervisor/events 裸文本码收集
        self._scenario = None
        self._t0 = None
        self._snapshot = int(time.time()) % 10000

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        obs_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self.node.create_subscription(
            HarvestState, '/peach_supervisor/state', self._on_state, latched)

        self.pub_obs = node.create_publisher(
            _msg('PeachTargetObservationArray'),
            '/peach/perception/target_observations', obs_qos)
        self.pub_decision = node.create_publisher(
            _msg('GraspDecision'), '/peach/reconstruction/grasp_decision',
            latched)
        self.pub_rs = node.create_publisher(
            RobotStatus, '/aubo_io_controller/robot_status', obs_qos)

        self.run_client = ActionClient(node, RunHarvest,
                                       '/peach_supervisor/run_harvest')
        from peach_interfaces.srv import SetEnables
        self.enables_client = node.create_client(
            SetEnables, '/peach_supervisor/set_enables')
        node.create_timer(0.1, self._on_rs_tick)
        node.create_timer(0.5, self._on_obs_tick)

    # -- 订阅 ------------------------------------------------------------
    def _on_state(self, msg):
        self.state = msg

    # -- 发布 ------------------------------------------------------------
    def _on_rs_tick(self):
        from aubo_msgs.msg import RobotStatus
        msg = RobotStatus()
        for attr in ('powered', 'motion_possible', 'in_motion', 'e_stopped'):
            if hasattr(msg, attr):
                setattr(msg, attr, False)
        if hasattr(msg, 'motion_possible'):
            msg.motion_possible = True
        if hasattr(msg, 'powered'):
            msg.powered = True
        self.pub_rs.publish(msg)

    def _on_obs_tick(self):
        if self._scenario is None or self.state is None:
            return
        content = observe_at(
            self._scenario, self._t0, time.monotonic(),
            int(self.state.scene_epoch or 0), self.state.run_id or '',
            snapshot_id=self._snapshot)
        if content is None:
            return
        msg = _msg('PeachTargetObservationArray')()
        msg.scene_epoch = int(self.state.scene_epoch or 0)
        msg.harvest_run_id = self.state.run_id or ''
        msg.target_set_locked = bool(content['locked'])
        msg.collecting_count = int(content['collecting'])
        msg.pending_count = 0
        msg.snapshot_id = self._snapshot
        msg.target_count = len(content['observations'])
        for item in content['observations']:
            msg.observations.append(_observation_msg(item))
        self.pub_obs.publish(msg)

    # -- 用例 API ---------------------------------------------------------
    def start(self, scenario: Scenario):
        self._scenario = scenario
        self._t0 = time.monotonic()

    def send_harvest(self, request_id: str, intent: int,
                     scene_key: str = 'e1'):
        from peach_interfaces.action import RunHarvest
        goal = RunHarvest.Goal()
        goal.request_id = request_id
        goal.scene_key = scene_key
        goal.intent = intent
        future = self.run_client.send_goal_async(goal)
        return future

    def set_execution_enabled(self, value: bool, timeout: float = 10.0):
        """调度侧 execution_enabled 参数（与臂侧 enables 广播独立的一维）."""
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import SetParameters
        cli = self.node.create_client(
            SetParameters, '/peach_supervisor/set_parameters')
        if not cli.wait_for_service(timeout_sec=timeout):
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(
            'execution_enabled',
            type_=Parameter.Type.BOOL, value=value).to_parameter_msg()]
        fut = cli.call_async(req)
        import time as _t
        deadline = _t.monotonic() + timeout
        while _t.monotonic() < deadline:
            if fut.done():
                return all(r.successful for r in fut.result().results)
            _t.sleep(0.1)
        return False

    def set_enables(self, execution: bool, grasp: bool = False,
                    tool: bool = False, timeout: float = 10.0) -> bool:
        """操作台使能（Survey=TRANSIT 类运动需 execution=true）."""
        from peach_interfaces.srv import SetEnables
        if not self.enables_client.wait_for_service(timeout_sec=timeout):
            return False
        req = SetEnables.Request()
        req.execution = execution
        req.grasp = grasp
        req.tool = tool
        req.reason = 'E1 launch_testing'
        import rclpy
        fut = self.enables_client.call_async(req)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if fut.done():
                return bool(fut.result().accepted)
            time.sleep(0.1)
        return False

    @property
    def batch_state(self):
        return int(self.state.batch_state) if self.state else None

    @property
    def epoch(self):
        return int(self.state.scene_epoch or 0) if self.state else 0


# ---------------------------------------------------------------------------
# IDL 构造帮助（延迟 import，模块可被零 ROS 环境收集）
# ---------------------------------------------------------------------------

def _msg(name):
    import importlib
    mod = importlib.import_module(f'peach_interfaces.msg')
    return getattr(mod, name)


def _observation_msg(item: dict):
    """单条观测：几何按 BagGraspCandidate 契约（entry_pose=Pose、
    bottom/neck Point、translation_direction=bottom→neck 单位向量）."""
    from geometry_msgs.msg import Point, Pose

    obs = _msg('PeachTargetObservation')()
    obs.target_id = item['target_id']
    obs.confirmed = True
    obs.tracking_status = 0          # OBSERVED
    obs.camera_distance_m = item['camera_distance_m']
    obs.confidence = item['confidence']
    cand = _msg('BagGraspCandidate')()
    cand.target_id = item['target_id']
    cand.status = 0                  # ACCEPT（=0；REOBSERVE=1 勿写反）
    ex, ey, ez = item['entry_xyz']
    ax, ay, az = item['axis']
    cand.entry_pose.position.x = ex
    cand.entry_pose.position.y = ey
    cand.entry_pose.position.z = ez
    # 袋底=入口沿 −axis 半袋长；袋颈=入口沿 +axis（axis 为袋底→袋口方向）
    half = 0.05
    cand.bag_bottom = Point(x=ex - ax * half, y=ey - ay * half, z=ez - az * half)
    cand.bag_neck = Point(x=ex + ax * half, y=ey + ay * half, z=ez + az * half)
    cand.translation_direction.x = ax
    cand.translation_direction.y = ay
    cand.translation_direction.z = az
    cand.bag_diameter_upper_m = 0.06
    cand.suggested_travel_m = 0.10
    cand.confidence = item['confidence']
    obs.candidate = cand
    return obs
