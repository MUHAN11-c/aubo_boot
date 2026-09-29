"""Isolated launch_testing: per-tool-profile mock smoke（TF 对档案/标签/跟随接线）.

三把末端（shear_v1 / bite_shear_v1 / adaptive_shear_v1）逐档起 mock harvest_system：
  - TF wrist3_Link→tcp 平移 == 档案 frames.tool_axis.xyz（aub_description 单一事实源）
  - /peach_arm tool.profile_id 标签与档名一致（testing.md P0 门的自动化）
  - /imu_follow/* 服务：仅 adaptive_shear_v1 在图（launch 条件 Include + C++
    usesImuFollowContact 名单；bite/shear 不起跟随、走 MTC LIN 套入/撤退）
收编 09-28 手工 _tools/smoke_shear.sh（域 91 手跑对账）进 colcon test 常驻回归。
档位由 PEACH_LT_TOOL_PROFILE 注入（CMake 三注册；直接单跑默认 adaptive_shear_v1）。
"""
import os
import tempfile
import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.util import resolveProcesses
import pytest
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool

from ament_index_python.packages import get_package_share_directory

from peach_bringup.preflight import running_stack_pids

# TF 对账容差：档案测量核定值 vs RSP 数值（09-28 实测两者一致，留 2mm 余量）
_TF_TOLERANCE_M = 0.002
# 仅 adaptive 名单内（peach_arm grasp_geometry.hpp usesImuFollowContact）
_IMU_FOLLOW_PROFILES = ('adaptive_shear_v1',)

_LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _active_profile():
    return os.environ.get('PEACH_LT_TOOL_PROFILE', 'adaptive_shear_v1')


def _archive_tcp_xyz(profile_id):
    """档案 frames.tool_axis.xyz（期望值单源；档案即真相，不在测试里抄数）."""
    import yaml
    path = os.path.join(
        get_package_share_directory('aubo_description'),
        'config', f'{profile_id}.yaml')
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    return [float(value) for value in data['frames']['tool_axis']['xyz']]


@pytest.mark.launch_test
def generate_test_description():
    """Start mock harvest_system with one tool profile. Do not send RunHarvest."""
    profile = _active_profile()
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
                    'tool_profile': profile,
                }.items()),
            # ReadyToTest 2s：起栈期间测试类即可订阅（latch 话题不丢）
            TimerAction(period=2.0, actions=[ReadyToTest()]),
        ]),
        {'tool_profile': profile},
    )


class TestToolProfileSmoke(unittest.TestCase):
    """Per-profile wiring: TCP frame vs archive, label, imu_follow presence."""

    def test_tcp_label_and_imu_wiring(self):
        profile = _active_profile()
        # 1) readiness：托管节点全部 Active（latch 一次即可）
        import rclpy
        from launch_testing_ros import WaitForTopics
        waiter = WaitForTopics(
            [('/peach/lifecycle/managed_nodes_activated', Bool)],
            timeout=75.0, qos_profile=_LATCHED)
        try:
            self.assertTrue(
                waiter.wait(), 'no lifecycle flag: %s' %
                waiter.topics_not_received())
            self.assertTrue(any(
                message.data for message
                in waiter.received_messages('/peach/lifecycle/managed_nodes_activated')))
        finally:
            waiter.shutdown()

        from rcl_interfaces.srv import GetParameters
        from rclpy.node import Node
        from rclpy.time import Time
        from tf2_ros import Buffer
        from tf2_ros import TransformListener
        owned_init = not rclpy.ok()
        if owned_init:
            rclpy.init()
        probe = Node(f'tool_profile_smoke_{profile}')
        buffer = Buffer()
        TransformListener(buffer, probe)
        try:
            # 2) TF wrist3_Link→tcp 平移对档案（固定帧链在 /tf_static，latest 可查）
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline and not buffer.can_transform(
                    'wrist3_Link', 'tcp', Time()):
                rclpy.spin_once(probe, timeout_sec=0.5)
            self.assertTrue(
                buffer.can_transform('wrist3_Link', 'tcp', Time()),
                'no TF wrist3_Link->tcp (profile=%s): tool xacro if 块未展开?'
                % profile)
            translation = buffer.lookup_transform(
                'wrist3_Link', 'tcp', Time()).transform.translation
            expected = _archive_tcp_xyz(profile)
            for axis, got, want in zip(
                    'xyz', (translation.x, translation.y, translation.z), expected):
                self.assertAlmostEqual(
                    got, want, delta=_TF_TOLERANCE_M,
                    msg='profile=%s TF wrist3_Link->tcp %s=%.5f, archive %.5f'
                        % (profile, axis, got, want))

            # 3) /peach_arm tool.profile_id 标签对齐（P0 门自动化）
            client = probe.create_client(
                GetParameters, '/peach_arm/get_parameters')
            self.assertTrue(
                client.wait_for_service(timeout_sec=30.0),
                '/peach_arm get_parameters service missing')
            request = GetParameters.Request()
            request.names = ['tool.profile_id']
            future = client.call_async(request)
            rclpy.spin_until_future_complete(probe, future, timeout_sec=10.0)
            self.assertTrue(future.done() and future.result() is not None)
            label = future.result().values[0].string_value
            self.assertEqual(
                label, profile,
                'stack label %r != requested profile %r (整栈须同 arg 重启)'
                % (label, profile))

            # 4) /imu_follow/* 服务：adaptive 档在图；bite/shear 档必须不在
            expect_imu = profile in _IMU_FOLLOW_PROFILES
            grace = time.monotonic() + (30.0 if expect_imu else 6.0)
            found = []
            while time.monotonic() < grace:
                rclpy.spin_once(probe, timeout_sec=0.5)
                found = [name for name, _ in probe.get_service_names_and_types()
                         if name.startswith('/imu_follow/')]
                if expect_imu and found:
                    break
            if expect_imu:
                self.assertTrue(
                    found,
                    'profile=%s 应随栈 Include imu_follow_servo，图上无 /imu_follow/*'
                    % profile)
            else:
                self.assertFalse(
                    found,
                    'profile=%s 不应起 imu_follow（图上发现 %s；忘传 tool_profile?）'
                    % (profile, found))
        finally:
            probe.destroy_node()
            if owned_init:
                rclpy.shutdown()


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
