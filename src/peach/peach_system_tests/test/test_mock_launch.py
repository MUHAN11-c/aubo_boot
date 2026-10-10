"""Isolated launch_testing: mock harvest_system, no arm, no auto RunHarvest."""
import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from bond.msg import Status as BondStatus
import launch
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.util import resolveProcesses
from launch_testing_ros import WaitForTopics
import pytest
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from peach_bringup.preflight import running_stack_pids

_MUST_JOINTS = (
    'shoulder_joint',
    'upperArm_joint',
    'foreArm_joint',
    'wrist1_joint',
    'wrist2_joint',
    'wrist3_joint',
)

_LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# /bond 心跳：订阅端 BEST_EFFORT 与任何发布端兼容（bondcpp/bondpy 均可收）
_BOND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE)


@pytest.mark.launch_test
def generate_test_description():
    """Start mock harvest_system. Do not send RunHarvest."""
    stale = running_stack_pids()
    if stale:
        lines = [f'{pid} {cmd}' for pid, cmd in stale[:8]]
        raise RuntimeError(
            'preflight blocked launch_testing; stop leftover stack:\n' +
            '\n'.join(lines))
    harvest = os.path.join(
        get_package_share_directory('peach_bringup'),
        'launch', 'harvest_system.launch.py')
    runs = tempfile.mkdtemp(prefix='peach_lt_')
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
                }.items()),
            # ReadyToTest 提前到 2s：bond 用例必须赶在 peach_arm 激活后的
            # 心跳窗口内已订阅（无 sister 时 bondcpp ~10s 后停发；激活实测
            # +1.5s 左右）。其余用例是事件驱动等待，提前开跑无碍。
            TimerAction(period=2.0, actions=[ReadyToTest()]),
        ]),
        {},
    )


class TestMockHarvestSystem(unittest.TestCase):
    """Mock stack publishes joints and lifecycle; launch never sends a goal."""

    def test_mock_joint_states_frozen_order(self):
        waiter = WaitForTopics(
            [('/joint_states', JointState)],
            timeout=45.0,
            qos_profile=qos_profile_sensor_data)
        try:
            self.assertTrue(
                waiter.wait(),
                'no /joint_states: %s' % waiter.topics_not_received())
            names = list(waiter.received_messages('/joint_states')[-1].name)
            missing = [joint for joint in _MUST_JOINTS if joint not in names]
            self.assertFalse(missing, 'joint_states names=%s' % names)
            # JSB may emit names alphabetically; command order is controllers.yaml.
        finally:
            waiter.shutdown()

    def test_lifecycle_active_without_run_harvest(self):
        waiter = WaitForTopics(
            [('/peach/lifecycle/managed_nodes_activated', Bool)],
            timeout=75.0,
            qos_profile=_LATCHED)
        try:
            self.assertTrue(
                waiter.wait(),
                'no lifecycle flag: %s' % waiter.topics_not_received())
            flagged = waiter.received_messages(
                '/peach/lifecycle/managed_nodes_activated')
            self.assertTrue(any(message.data for message in flagged))
        finally:
            waiter.shutdown()

    def test_selfcheck_passes_and_artifacts_written(self):
        """P0 启动自检：mock 栈（相机/robot_status=SKIP）应自检通过并落工件."""
        import json as _json
        from pathlib import Path
        waiter = WaitForTopics(
            [('/peach/observability/selfcheck_passed', Bool)],
            timeout=150.0, qos_profile=_LATCHED)
        try:
            self.assertTrue(
                waiter.wait(),
                'no selfcheck verdict: %s' % waiter.topics_not_received())
            verdicts = waiter.received_messages(
                '/peach/observability/selfcheck_passed')
            self.assertTrue(
                any(message.data for message in verdicts),
                'selfcheck never passed; verdicts=%s' % verdicts)
        finally:
            waiter.shutdown()
        runs = os.environ.get('AUBO_RUNS_DIR', '')
        self.assertTrue(runs, 'AUBO_RUNS_DIR not set in test env')
        root = Path(runs).resolve()
        sessions = sorted(
            entry for entry in root.iterdir()
            if entry.is_dir() and entry.name.startswith('session_'))
        self.assertTrue(sessions, 'no session dir under %s' % root)
        session = sessions[-1]
        # 路径 containment 校验：工件只允许位于本测试的 runs 根内
        for name in ('selfcheck.json', 'startup.json'):
            path = (session / name).resolve()
            self.assertTrue(
                path.is_relative_to(root) and path.is_file(),
                'missing %s (session=%s)' % (name, session))
        with (session / 'selfcheck.json').open(encoding='utf-8') as stream:
            report = _json.load(stream)
        self.assertEqual(report.get('status'), 'pass')
        self.assertEqual(report.get('facts', {}).get('hardware_mode'), 'mock')
        with (session / 'startup.json').open(encoding='utf-8') as stream:
            self.assertIn('hardware_mode', stream.read())

    def test_arm_bond_heartbeats(self):
        """W14：peach_arm（bondcpp）激活后有 1Hz bond 心跳（约 10s 爆发）.

        无 sister（nav2_lm bond_timeout=0）时 bondcpp 约 10s 后 ConnectTimeout
        进 Dead 停发——本用例锁「激活后确有心跳爆发」这一接线前提；lm 开
        bond_timeout 后 sister 存在、心跳常驻。直接 rclpy 订阅（WaitForTopics
        在本用例上收不到该话题，机制未明；/joint_states 对照同订阅方式收得
        正常）。Python 三节点心跳依赖 ros-jazzy-bondpy（未装时守卫降级不
        发言），不在本用例范围。
        """
        import rclpy
        counts = {'bond': 0, 'joints': 0}
        seen_ids = set()
        owned_init = not rclpy.ok()
        if owned_init:
            rclpy.init()
        probe = rclpy.create_node('bond_test_probe')
        try:
            probe.create_subscription(
                BondStatus, '/bond',
                lambda msg: (counts.__setitem__('bond', counts['bond'] + 1),
                             seen_ids.add(str(msg.id))), _BOND_QOS)
            probe.create_subscription(
                JointState, '/joint_states',
                lambda msg: counts.__setitem__('joints', counts['joints'] + 1),
                qos_profile_sensor_data)
            deadline = time.monotonic() + 90.0
            while time.monotonic() < deadline and counts['bond'] < 3:
                rclpy.spin_once(probe, timeout_sec=0.5)
        finally:
            probe.destroy_node()
            if owned_init:
                rclpy.shutdown()
        self.assertGreater(
            counts['joints'], 0, 'control topic /joint_states not received; '
            'test environment broken')
        arm = counts['bond']
        self.assertGreaterEqual(
            arm, 3,
            'peach_arm bond heartbeats too few (%d, ids=%s)'
            % (arm, sorted(seen_ids)))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Processes must exit; this is not an e-stop."""

    def test_exit_codes(self, proc_info):
        dumped = str(proc_info)
        self.assertNotIn('RunHarvest', dumped)
        items = resolveProcesses(
            info_obj=proc_info, process=None, cmd_args=None,
            strict_proc_matching=True)
        ignored = ('move_group', 'rviz2')
        for item in items:
            info = proc_info[item]
            name = info.process_name
            if any(token in name for token in ignored):
                continue
            self.assertIn(
                info.returncode, (0, -2, -9, -15),
                '%s exited %s' % (name, info.returncode))
