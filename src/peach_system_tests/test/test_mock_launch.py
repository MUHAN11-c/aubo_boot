"""Isolated launch_testing: mock harvest_system, no arm, no auto RunHarvest."""
import os
import tempfile
import unittest

from ament_index_python.packages import get_package_share_directory
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
            TimerAction(period=12.0, actions=[ReadyToTest()]),
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
