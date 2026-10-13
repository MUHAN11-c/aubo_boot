"""P2 验收门（isolated launch_testing）：统一机体控制器与关节状态.

起 sim.launch.py（无头、无 moveit——move_group 已在手动验收覆盖），等待：
1. /joint_states 出现且六关节名 = AUBO 契约集合；
2. joint_state_broadcaster 与 joint_trajectory_controller 双双 active。
重世界（368 模型）首载 40-90 s，超时预算 300 s。
"""

import unittest
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState

ARM_JOINTS = {'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
              'wrist1_joint', 'wrist2_joint', 'wrist3_joint'}
WAIT_S = 300.0


def generate_test_description():
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / 'launch' / 'sim.launch.py')),
        launch_arguments={'gui': 'false', 'moveit': 'false'}.items())
    return LaunchDescription([sim_launch, ReadyToTest()]), {}


class TestUnifiedControllers(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_sim_controllers')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_joint_states_six_aubo_joints(self):
        """/joint_states 六关节名 = AUBO 契约（带位置与速度）。"""
        seen: dict = {}

        def _cb(msg: JointState):
            seen['names'] = set(msg.name)
            seen['fields'] = bool(msg.position) and bool(msg.velocity)

        sub = self.node.create_subscription(
            JointState, '/joint_states', _cb, 10)
        self._spin_until(lambda: 'names' in seen, WAIT_S)
        self.node.destroy_subscription(sub)
        self.assertEqual(seen.get('names'), ARM_JOINTS)
        self.assertTrue(seen.get('fields'))

    def test_controllers_active(self):
        """JSB 与 JTC 双双 active。"""
        client = self.node.create_client(ListControllers,
                                         '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=WAIT_S))
        states: dict = {}

        def _poll() -> bool:
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            states.clear()
            states.update({c.name: c.state
                           for c in future.result().controller})
            want = {'joint_state_broadcaster', 'joint_trajectory_controller'}
            return want <= states.keys() and all(
                states[name] == 'active' for name in want)

        self._spin_until(_poll, WAIT_S)
        self.node.destroy_client(client)
        self.assertEqual(states.get('joint_state_broadcaster'), 'active')
        self.assertEqual(states.get('joint_trajectory_controller'), 'active')

    def _spin_until(self, predicate, timeout_s: float, period_s: float = 1.0):
        import time
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=period_s)
            if predicate():
                return
        self.fail(f'等待超时（{timeout_s:.0f}s）：条件未满足')
