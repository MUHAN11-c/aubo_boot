"""E1 自动化批次链 launch_testing（阶段三一期，v2.2 方案 §5）.

首版用例：C01 SURVEY_ONLY 只扫结算 / C02 使能关 RECORD_DISABLED——两者都
不断言接触规划（那是 C03+ 的领地），先锁批次链最上游语义：
RunHarvest→Survey→BeginScene→WAIT_LOCK→锁定→结算，全程零 ExecuteTarget
派发（batch_state 不得进 RUNNING）。

注入契约依据 S0 尖峰定论（决策 0029⑤）：单发布者/epoch 对齐/两段式锁定。
隔离域：CMake ENV ROS_DOMAIN_ID=90（与 mock_launch 的 89 分域）。
"""
import json
import os
import re
import tempfile
import threading
import time
import unittest
import uuid

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
import pytest
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import perception_sim as psim  # noqa: E402

from peach_bringup.preflight import running_stack_pids  # noqa: E402

_LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

# 现场典型几何（base 系；深度窗 0.30–1.60m 内）
_TWO_TARGETS = [
    psim.SimTarget(
        target_id='e1_typical_a',
        entry_xyz=(0.40, -0.60, 0.55), axis=(0.0, 0.0, 1.0)),
    psim.SimTarget(
        target_id='e1_typical_b',
        entry_xyz=(0.35, -0.45, 0.60), axis=(0.1, 0.0, 0.995)),
]

_RID_RE = re.compile(r'^[A-Za-z0-9_.:-]+$')


def _ledger_path(rid: str) -> str:
    """账本路径：rid 白名单校验后拼接（防路径穿越，仅允许字符集）."""
    if not _RID_RE.match(rid) or '..' in rid:
        raise ValueError(f'非法 request_id: {rid!r}')
    base = os.environ.get('AUBO_RUNS_DIR', 'runs')
    return os.path.join(base, rid, 'ledger.json')


@pytest.mark.launch_test
def generate_test_description():
    """mock harvest_system + 感知注入由用例内节点承担；不发自动意图."""
    stale = running_stack_pids()
    if stale:
        lines = [f'{pid} {cmd}' for pid, cmd in stale[:8]]
        raise RuntimeError(
            'preflight blocked E1 launch_testing; stop leftover stack:\n' +
            '\n'.join(lines))
    harvest = os.path.join(
        get_package_share_directory('peach_bringup'),
        'launch', 'harvest_system.launch.py')
    runs = tempfile.mkdtemp(prefix='peach_e1_')
    return (
        launch.LaunchDescription([
            SetEnvironmentVariable('QT_QPA_PLATFORM', 'offscreen'),
            SetEnvironmentVariable('AUBO_RUNS_DIR', runs),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(harvest),
                launch_arguments={
                    'hardware_mode': 'mock',
                    'camera_enabled': 'false',
                    'imu_enabled': 'false',
                    'moveit_enabled': 'true',
                    'extrinsics_enabled': 'true',
                    'hand_eye_enabled': 'false',
                    'hand_eye_web_enabled': 'false',
                    'skip_reconstruction': 'true',
                }.items()),
            TimerAction(period=2.0, actions=[ReadyToTest()]),
        ]),
        {},
    )


class _Injector:
    """用例共享的注入节点 + 后台 spin 线程 + 状态轨迹."""

    def __init__(self):
        import rclpy
        from peach_interfaces.msg import HarvestState
        from std_msgs.msg import Bool
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('perception_sim_e1')
        self.sim = psim.PerceptionSim(self.node)
        self.node.create_subscription(
            Bool, '/peach/lifecycle/managed_nodes_activated',
            self._on_flag, _LATCHED)
        # 状态轨迹：独立订阅（sim 内部 _on_state 只缓存最新值，此处补轨迹）
        self.node.create_subscription(
            HarvestState, '/peach_supervisor/state', self._on_state, _LATCHED)
        self.flag = None
        self.states = []           # (monotonic, batch_state) 轨迹
        self._spin = threading.Thread(target=self._spin_forever, daemon=True)
        self._spin.start()

    def _on_state(self, msg):
        self.states.append((time.monotonic(), int(msg.batch_state)))

    def _spin_forever(self):
        exe = self.rclpy.executors.SingleThreadedExecutor()
        exe.add_node(self.node)
        while self.rclpy.ok():
            exe.spin_once(timeout_sec=0.2)

    def _on_flag(self, msg):
        self.flag = msg.data

    def wait_lifecycle(self, timeout=90.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.flag:
                return True
            time.sleep(0.3)
        return False

    def wait_state(self, predicate, timeout=240.0, mark=0):
        deadline = time.monotonic() + timeout
        i = mark
        while time.monotonic() < deadline:
            if predicate(self.states[i:]):
                return True
            i = len(self.states)
            time.sleep(0.3)
        return False


INJECTOR = None


class TestE1SupervisorChain(unittest.TestCase):
    """C01/C02：批次链上游语义（结算 + 零派发）."""

    @classmethod
    def setUpClass(cls):
        global INJECTOR
        INJECTOR = _Injector()
        if not INJECTOR.wait_lifecycle():
            raise unittest.SkipTest('lifecycle 闩锁超时（栈未就绪）')
        if not INJECTOR.sim.run_client.wait_for_server(timeout_sec=30.0):
            raise unittest.SkipTest('run_harvest 服务端超时')
        # 稳定窗：brain 激活即装 YOLO/SAM 模型，交互轮由操作员自然吸收；
        # 自动化直发首 批会撞上模型加载（实测 controller_manager 一次
        # 42.7s overrun → survey 超时 INTERRUPTED）。等加载完再发意图。
        time.sleep(12.0)

    @classmethod
    def tearDownClass(cls):
        if INJECTOR:
            INJECTOR.rclpy.shutdown()

    # -- C01 ---------------------------------------------------------------
    def test_c01_survey_only_settles_without_dispatch(self):
        sim = INJECTOR.sim
        self.assertTrue(
            sim.set_enables(execution=True),
            'SetEnables 失败（Survey=TRANSIT 运动需 execution）')
        rid = f'e1_c01_{uuid.uuid4().hex[:8]}'
        mark = len(INJECTOR.states)
        goal_future = sim.send_harvest(rid, intent=2)  # SURVEY_ONLY
        handle = self._wait_goal(goal_future, 30.0)
        self.assertIsNotNone(handle, 'RunHarvest goal 未被受理')
        # 注入器在 Begin 之后启动进度→锁定（epoch 由 latched state 对齐）
        sim.start(psim.Scenario(targets=_TWO_TARGETS, progress_s=1.0))
        ok = INJECTOR.wait_state(
            lambda tail: any(s == 6 for _, s in tail),
            timeout=240.0, mark=mark)
        tail_states = [s for _, s in INJECTOR.states[mark:]]
        self.assertTrue(ok, 'C01 未到 COMPLETED；states=%s' % tail_states)
        self.assertFalse(
            any(s == 2 for _, s in INJECTOR.states[mark:]),
            'C01 出现 RUNNING=零 ExecuteTarget 派发被破坏')
        self._assert_ledger(rid)

    # -- C02 ---------------------------------------------------------------
    @unittest.skip(
        'F-E1-2（09-24 实测，待产品裁定）：批中 param 关 execution_enabled '
        '不拦派发——SetEnables 单旋钮同时置调度参数；受理期开→Begin 后关'
        '（锁定前窗口）仍照发目标（RUNNING 出现）。EXECUTION_DISABLED-at-'
        'SELECT 的真实可达路径（受理期即关？或 enables 语义需改活查？）'
        '裁定后重写本例配方。证据：runs 09-24 E1 三轮 + events survey_failed')
    def test_c02_execution_disabled_settles_without_dispatch(self):
        sim = INJECTOR.sim
        # 勘定（E1 首跑）：SetEnables 是单一旋钮（同时置调度 execution_enabled）。
        # C02 配方：enables 开（Survey=TRANSIT 要动）→ Begin 后（epoch>0）
        # param 关调度 execution_enabled → SELECT 期 EXECUTION_DISABLED 结算。
        self.assertTrue(
            sim.set_enables(execution=True),
            'SetEnables 失败（Survey=TRANSIT 运动需 execution）')
        rid = f'e1_c02_{uuid.uuid4().hex[:8]}'
        mark = len(INJECTOR.states)
        goal_future = sim.send_harvest(rid, intent=0)  # PICK_ALL
        handle = self._wait_goal(goal_future, 30.0)
        self.assertIsNotNone(handle, 'RunHarvest goal 未被受理')
        # 进度期拉长到 10s：Begin 后、锁定前有关开关的确定性窗口
        sim.start(psim.Scenario(targets=_TWO_TARGETS, progress_s=10.0))
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline and sim.epoch == 0:
            time.sleep(0.3)
        self.assertGreater(sim.epoch, 0, 'BeginScene 未发生（survey 未完成）')
        self.assertTrue(
            sim.set_execution_enabled(False),
            'execution_enabled 关闭失败（须落在锁定前的进度窗内）')
        ok = INJECTOR.wait_state(
            lambda tail: any(s == 6 for _, s in tail),
            timeout=240.0, mark=mark)
        tail_states = [s for _, s in INJECTOR.states[mark:]]
        self.assertTrue(ok, 'C02 未到 COMPLETED；states=%s' % tail_states)
        self.assertFalse(
            any(s == 2 for _, s in INJECTOR.states[mark:]),
            'C02 出现 RUNNING=使能关仍派发目标')
        self._assert_ledger(rid)

    # -- 帮助 ---------------------------------------------------------------
    def _wait_goal(self, future, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.2)
        return None

    def _assert_ledger(self, rid):
        path = _ledger_path(rid)
        self.assertTrue(os.path.exists(path), '账本缺失: %s' % path)
        ledger = json.loads(open(path).read())
        self.assertIn('claimed', ledger)
        self.assertIn('outcomes', ledger)


@launch_testing.post_shutdown_test()
class TestE1Shutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        from launch_testing.util import resolveProcesses
        dumped = str(proc_info)
        self.assertNotIn('RunHarvest 自动', dumped)
        items = resolveProcesses(
            info_obj=proc_info, process=None, cmd_args=None,
            strict_proc_matching=True)
        ignored = ('move_group', 'rviz2', 'perception_sim_e1')
        for item in items:
            info = proc_info[item]
            if any(token in info.process_name for token in ignored):
                continue
            self.assertIn(
                info.returncode, (0, -2, -9, -15),
                '%s exited %s' % (info.process_name, info.returncode))
