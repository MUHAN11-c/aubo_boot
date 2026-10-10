"""
启动自检 runner：ROS 探针 + 只读查询 + 工件落盘 + 闩锁发布.

设计约束：
- 全部只读（不动臂、不 SetIO、不发 goal）；service/action 只查询可达性
- 非阻塞：周期检查在 executor 回调内即时完成；控制器状态经异步查询
  在 1Hz tick 里收割（首次报告可能 WARN「查询未返回」，不 FAIL）
- 探针常驻订阅（joint_states/相机/robot_status/managed 旗标/vegetation），
  RateProbe 滑窗计数，检查时零等待
- 出口四路：/peach/observability/selfcheck_passed（Bool latched，
  autostart 硬等）、runs/session_*/selfcheck.json + history.jsonl、
  /api/state selfcheck 分区、/diagnostics selfcheck 任务（节点侧注册）
"""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import threading
import time

from aubo_msgs.msg import RobotStatus
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetPlanningScene
from peach_interfaces.action import (
    BuildTargetModel,
    ExecuteTarget,
    RunHarvest,
    SurveyScene,
)
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String

from .checks import (
    check_controllers,
    check_disk_free,
    check_fresh,
    check_joint_order,
    check_min_rate,
    check_param_consistency,
    check_present,
    check_tcp_norm,
    CheckResult,
    evaluate,
    EXPECTED_CONTROLLERS,
    RateProbe,
)

MANAGED_FLAG_TOPIC = '/peach/lifecycle/managed_nodes_activated'
PASSED_TOPIC = '/peach/observability/selfcheck_passed'
VEGETATION_STATUS_TOPIC = '/peach/vegetation/status'


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE)


class SelfCheckRunner:
    """常驻探针 + 周期/按需自检；生命周期由宿主节点管理."""

    def __init__(self, node, params, runs_root: Path, session_dir_of):
        """
        构造常驻探针与周期检查.

        node 为宿主 ObservabilityNode；params 为 ObservabilityParams 快照
        （selfcheck_* 字段）；session_dir_of() 返回当前会话目录或 None.
        """
        self._node = node
        self._params = params
        self._runs_root = Path(runs_root)
        self._session_dir_of = session_dir_of
        self._lock = threading.Lock()
        self._subs = []
        self._clients = []
        self._action_clients = []
        self._tick_timer = None
        self._period_timer = None
        self._passed_pub = None
        self._controller_client = None
        self._controller_future = None
        self._controller_states = None
        self._controller_queried_at = 0.0
        self._mg_client = None
        self._managed_seen = None
        self._joint_names = None
        window = params.selfcheck_rate_window_s
        self._probes = {
            'joint_states': RateProbe(window_s=window),
            'camera_color': RateProbe(window_s=window),
            'camera_depth': RateProbe(window_s=window),
            'robot_status': RateProbe(window_s=window),
            'vegetation': RateProbe(window_s=window),
        }
        self._facts = {}
        self._activated_at = 0.0
        self._first_run_done = False
        self._running = False
        self._last_report: dict = {}
        self._create_entities()

    # ------------------------------------------------------------------
    # 实体创建（configure 期；全部只读）
    # ------------------------------------------------------------------
    def _create_entities(self) -> None:
        p = self._params
        node = self._node
        latched = _latched_qos()
        self._subs.append(node.create_subscription(
            JointState, p.topics['joint_states_topic'],
            self._on_joint_state, qos_profile_sensor_data))
        self._subs.append(node.create_subscription(
            Bool, MANAGED_FLAG_TOPIC, self._on_managed_flag, latched))
        self._subs.append(node.create_subscription(
            String, VEGETATION_STATUS_TOPIC,
            self._make_probe_cb('vegetation'), latched))
        if p.selfcheck_camera_probe_enabled:
            self._subs.append(node.create_subscription(
                Image, p.selfcheck_color_image_topic,
                self._make_probe_cb('camera_color'),
                qos_profile_sensor_data))
            self._subs.append(node.create_subscription(
                Image, p.selfcheck_depth_image_topic,
                self._make_probe_cb('camera_depth'),
                qos_profile_sensor_data))
        if p.robot_status_probe_enabled:
            self._subs.append(node.create_subscription(
                RobotStatus, p.topics['robot_status_topic'],
                self._make_probe_cb('robot_status'), 10))
        self._controller_client = node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self._clients.append(self._controller_client)
        self._mg_client = node.create_client(
            GetPlanningScene, '/get_planning_scene')
        self._clients.append(self._mg_client)
        for action_type, name in (
                (RunHarvest, p.debug_endpoints['run_harvest_action']),
                (SurveyScene, p.debug_endpoints['survey_action']),
                (ExecuteTarget, p.debug_endpoints['execute_action']),
                (BuildTargetModel, p.debug_endpoints['build_action'])):
            client = rclpy.action.ActionClient(node, action_type, name)
            self._action_clients.append((name.rsplit('/', 1)[-1], client))
        self._passed_pub = node.create_publisher(Bool, PASSED_TOPIC, latched)

    def _make_probe_cb(self, key: str):
        def callback(_msg) -> None:
            with self._lock:
                self._probes[key].push(time.monotonic())
        return callback

    def _on_joint_state(self, msg) -> None:
        with self._lock:
            self._joint_names = list(msg.name)
            self._probes['joint_states'].push(time.monotonic())

    def _on_managed_flag(self, msg) -> None:
        if msg.data:
            with self._lock:
                self._managed_seen = time.monotonic()

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start(self) -> None:
        """启动 1Hz tick 与周期检查定时器（activate 后调用）."""
        self._activated_at = time.monotonic()
        self._tick_timer = self._node.create_timer(1.0, self._tick)
        if self._params.selfcheck_period_s > 0.0:
            self._period_timer = self._node.create_timer(
                self._params.selfcheck_period_s, self._periodic)

    def stop(self) -> None:
        """停定时器（deactivate 用；实体保留，重 activate 复用）."""
        for timer in (self._tick_timer, self._period_timer):
            if timer is not None:
                self._node.destroy_timer(timer)
        self._tick_timer = None
        self._period_timer = None

    def destroy(self) -> None:
        """cleanup：释放探针订阅/客户端/发布器."""
        self.stop()
        for sub in self._subs:
            self._node.destroy_subscription(sub)
        self._subs = []
        for client in self._clients:
            self._node.destroy_client(client)
        self._clients = []
        for _, client in self._action_clients:
            client.destroy()
        self._action_clients = []
        if self._passed_pub is not None:
            self._node.destroy_publisher(self._passed_pub)
            self._passed_pub = None

    def _tick(self) -> None:
        """1Hz：控制器查询收割/补发 + 初次自检编排（全程非阻塞）."""
        self._harvest_controller_query()
        if (self._controller_future is None
                and time.monotonic() - self._controller_queried_at > 5.0):
            self._fire_controller_query()
        if not self._first_run_done:
            settle = self._params.selfcheck_initial_settle_s
            timeout = self._params.selfcheck_initial_timeout_s
            now = time.monotonic()
            elapsed = now - self._activated_at
            managed_ok = self._managed_seen is not None
            if (managed_ok and elapsed >= settle) or elapsed >= timeout:
                self.run('initial')

    def _periodic(self) -> None:
        """周期复检；与初次/手动检查互斥（同刻至多一轮）."""
        if self._first_run_done:
            self.run('periodic')

    # ------------------------------------------------------------------
    # 控制器状态（异步两拍，避免 executor 回调内阻塞等响应）
    # ------------------------------------------------------------------
    def _fire_controller_query(self) -> None:
        self._controller_queried_at = time.monotonic()
        self._controller_future = self._controller_client.call_async(
            ListControllers.Request())

    def _harvest_controller_query(self) -> None:
        future = self._controller_future
        if future is None or not future.done():
            return
        self._controller_future = None
        try:
            response = future.result()
            self._controller_states = {
                c.name: c.state for c in response.controller}
        except Exception as exc:  # noqa: BLE001 服务失败保持上次值
            self._node.get_logger().warning(f'list_controllers 查询失败: {exc}')

    # ------------------------------------------------------------------
    # 检查执行（触发：initial / periodic / manual(8090)）
    # ------------------------------------------------------------------
    def run(self, trigger: str = 'manual') -> dict:
        """执行一轮全量检查（非阻塞）并发布/落盘；返回报告字典."""
        if self._running:
            return dict(self._last_report)
        self._running = True
        try:
            return self._run_checks(trigger)
        finally:
            self._running = False
            self._first_run_done = True

    def _run_checks(self, trigger: str) -> dict:
        started = time.time()
        mono = time.monotonic()
        p = self._params
        with self._lock:
            joint_names = list(self._joint_names or [])
            probes = dict(self._probes)
            managed_seen = self._managed_seen
        facts = self._parse_facts()
        window = p.selfcheck_rate_window_s
        checks = []

        # 1) 托管生命周期
        checks.append(check_present(
            None if managed_seen is None else True,
            'managed_lifecycle', '托管节点已激活',
            'managed_nodes_activated 未置位（lifecycle 栈未就绪）'))
        # 2) 控制器（异步两拍：首轮可能 WARN 查询未返回）
        mode = str(facts.get('hardware_mode', '')) or 'mock'
        checks.append(check_controllers(
            self._controller_states,
            EXPECTED_CONTROLLERS.get(mode, EXPECTED_CONTROLLERS['mock'])))
        # 3) joint_states 流 + MUST 关节序
        checks.append(check_min_rate(
            probes['joint_states'].count(mono), window, 10.0,
            'joint_states_flow'))
        checks.append(check_joint_order(
            joint_names or None, p.selfcheck_expected_joints))
        # 4) TF：wrist3_Link→tcp 对档案
        checks.append(check_tcp_norm(
            self._lookup_tcp_norm(), p.selfcheck_expected_tcp_norm_m))
        # 5) 相机帧率（camera_enabled=false → SKIP）
        camera_on = p.selfcheck_camera_probe_enabled
        for key, label in (('camera_color', 'camera_color_rate'),
                           ('camera_depth', 'camera_depth_rate')):
            checks.append(check_min_rate(
                probes[key].count(mono) if camera_on else None,
                window, p.selfcheck_camera_min_hz, label,
                skip_reason='camera_enabled=false'))
        # 6) move_group / 四动作服务端
        checks.append(check_present(
            None if not p.selfcheck_moveit_expected
            else self._mg_client.service_is_ready(),
            'move_group', 'GetPlanningScene 可用',
            'move_group / get_planning_scene 不可用',
            skip_reason='moveit_enabled=false'))
        for label, client in self._action_clients:
            checks.append(check_present(
                client.server_is_ready(), f'action_{label}',
                '服务端就绪', '动作服务端未就绪'))
        # 7) robot_status（真机才启用探针；否则 SKIP）
        checks.append(check_fresh(
            probes['robot_status'].age(mono) if p.robot_status_probe_enabled
            else None, 2.0, 'robot_status',
            skip_reason='mock 模式不要求'))
        # 8) IMU 话题在场
        checks.append(self._check_imu(p))
        # 9) 磁盘余量
        free = None
        try:
            free = shutil.disk_usage(str(self._runs_root)).free
        except OSError:
            pass
        checks.append(check_disk_free(
            free, int(p.selfcheck_disk_min_gb * (1 << 30))))
        # 10) 模型权重文件
        missing = [
            path for path in p.selfcheck_model_paths
            if path and not Path(path).exists()]
        checks.append(check_present(
            None if not p.selfcheck_model_paths else not missing,
            'model_files', '模型权重文件在位',
            f'缺失: {", ".join(missing)}', skip_reason='未配置检查清单'))
        # 11) 启动参数组合自洽
        checks.append(check_param_consistency(facts))
        # 12) vegetation（独立 launch，在图才查；否则 SKIP）
        veg_on = self._topic_has_publisher(VEGETATION_STATUS_TOPIC)
        checks.append(check_fresh(
            probes['vegetation'].age(mono) if veg_on else None, 30.0,
            'vegetation_status', skip_reason='vegetation 不在图'))

        report = {
            'trigger': trigger,
            'checked_at': time.strftime(
                '%Y-%m-%dT%H:%M:%S', time.localtime(started)),
            'duration_s': round(time.time() - started, 3),
            'facts': facts,
            'checks': [c.to_dict() for c in checks],
            **evaluate(checks),
        }
        self._last_report = report
        self._emit(report)
        return dict(report)

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _parse_facts(self) -> dict:
        raw = (self._params.startup_facts or '').strip()
        if not raw:
            return {}
        try:
            facts = json.loads(raw)
            return facts if isinstance(facts, dict) else {}
        except ValueError:
            return {}

    def _lookup_tcp_norm(self) -> float | None:
        buffer = getattr(self._node, '_tf_buffer', None)
        if buffer is None:
            return None
        try:
            transform = buffer.lookup_transform(
                'wrist3_Link', 'tcp', rclpy.time.Time())
            tr = transform.transform.translation
            return float((tr.x ** 2 + tr.y ** 2 + tr.z ** 2) ** 0.5)
        except Exception:  # noqa: BLE001 tf2 各类异常统一视为不可用
            return None

    def _topic_has_publisher(self, topic: str) -> bool:
        try:
            return any(
                name == topic
                for name, _ in self._node.get_topic_names_and_types())
        except Exception:  # noqa: BLE001
            return False

    def _check_imu(self, p) -> CheckResult:
        if not p.selfcheck_imu_expected:
            return CheckResult('imu_topic', 'skip', 'imu_enabled=false', None)
        found = ''
        for name, _ in self._node.get_topic_names_and_types():
            if 'imu' in name.lower():
                found = name
                break
        if found:
            return CheckResult(
                'imu_topic', 'pass', f'话题在场: {found}', found)
        return CheckResult('imu_topic', 'fail', '图上无 imu 话题', None)

    # ------------------------------------------------------------------
    # 出口：日志 + 闩锁 Bool + /api/state + 会话工件
    # ------------------------------------------------------------------
    def _emit(self, report: dict) -> None:
        logger = self._node.get_logger()
        prefix = f'自检[{report["trigger"]}]'
        if report['status'] == 'pass':
            logger.info(f'{prefix} {report["summary"]}')
        else:
            logger.warning(f'{prefix} {report["summary"]}')
        if self._passed_pub is not None:
            message = Bool()
            message.data = bool(report['passed'])
            self._passed_pub.publish(message)
        state = getattr(self._node, '_state', None)
        if state is not None:
            state.update('selfcheck', 'report', report)
        session_dir = self._session_dir_of()
        if session_dir is None:
            return
        try:
            payload = json.dumps(report, ensure_ascii=False, indent=2)
            tmp = Path(session_dir) / 'selfcheck.json.tmp'
            tmp.write_text(payload, encoding='utf-8')
            tmp.replace(Path(session_dir) / 'selfcheck.json')
            with (Path(session_dir) / 'selfcheck_history.jsonl').open(
                    'a', encoding='utf-8') as stream:
                stream.write(json.dumps({
                    'checked_at': report['checked_at'],
                    'trigger': report['trigger'],
                    'status': report['status'],
                    'summary': report['summary'],
                }, ensure_ascii=False) + '\n')
        except OSError as exc:
            logger.warning(f'自检工件落盘失败: {exc}')

    @property
    def last_report(self) -> dict:
        """最近一次报告（8090 /api/state 与诊断任务读）."""
        return dict(self._last_report)

    def controller_states(self) -> dict | None:
        """最近一次 list_controllers 结果（诊断任务附带）."""
        return dict(self._controller_states or {}) or None
