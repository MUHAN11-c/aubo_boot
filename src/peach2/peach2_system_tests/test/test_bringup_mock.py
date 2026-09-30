"""Isolated launch_testing: mock peach2_system comes up Active, enables all false, no goals."""
from pathlib import Path
import sys
import unittest

from launch_testing import post_shutdown_test
from lifecycle_msgs.msg import State as LifecycleState
from peach2_interfaces.action import HarvestTarget, MoveTo, ObserveTarget, RunBatch
from peach2_interfaces.srv import CheckReachability, GetDecision, SetEnables
import pytest
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parent))
import peach2_harness as h  # noqa: E402, I100

MANAGED = ('peach2_target_model', 'peach2_manipulation', 'peach2_task')


@pytest.mark.launch_test
def generate_test_description():
    return h.stack_launch_description(), {}


class TestBringupMock(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.node = h.Session.get(bags=())
        cls.node.publish_observations = False

    @classmethod
    def tearDownClass(cls):
        h.Session.shutdown()

    def test_0_lifecycle_active(self):
        for name in MANAGED:
            self.assertTrue(self.node.wait_active(name, 180.0), f'{name} not active')
        self.assertIsNone(self.node.lifecycle_state('peach2_perception', 1.0),
                          'perception must not be launched with camera_enabled:=false')
        self.assertIsNone(self.node.lifecycle_state('peach2_scene', 1.0),
                          'scene must not be launched with camera_enabled:=false')
        is_active = self.node.create_client(
            Trigger, f'/{h.MANAGER}/is_active', callback_group=self.node._group)
        self.assertTrue(
            is_active.wait_for_service(timeout_sec=60.0),
            f'/{h.MANAGER}/is_active not advertised')
        response = h.wait_future(is_active.call_async(Trigger.Request()), 20.0)
        self.assertIsNotNone(response, 'lifecycle manager is_active unavailable')
        self.assertTrue(response.success, 'lifecycle manager reports inactive stack')
        self.node.destroy_client(is_active)

    def test_0b_mock_controllers_and_joints(self):
        self.assertTrue(self.node.wait_controllers(h.MOCK_CONTROLLERS, 60.0),
                        f'controllers active: {self.node.active_controllers()}')
        self.assertTrue(h.wait_until(lambda: self.node.joints() is not None, 30.0),
                        'no /joint_states')
        self.assertEqual(set(self.node.joints()), set(h.JOINTS))

    def test_1_enables_default_false(self):
        self.assertTrue(h.wait_until(lambda: self.node.enables is not None, 30.0),
                        'no /peach/enables')
        e = self.node.enables
        self.assertFalse(e.execution or e.grasp or e.tool, f'enables not all false: {e}')

    def test_2_tool_state_published(self):
        self.assertTrue(h.wait_until(lambda: self.node.tool_state is not None, 30.0),
                        'no /peach/end_effector/tool_state')

    def test_3_servers_ready(self):
        n = self.node
        actions = {
            '/peach/task/run_batch': RunBatch,
            '/peach/manipulation/harvest_target': HarvestTarget,
            '/peach/manipulation/move_to': MoveTo,
            '/peach/target_model/observe': ObserveTarget,
        }
        for name, kind in actions.items():
            client = ActionClient(n, kind, name)
            self.assertTrue(client.wait_for_server(timeout_sec=20.0), f'{name} not ready')
            client.destroy()
        services = {
            '/peach/task/set_enables': SetEnables,
            '/peach/task/acknowledge_recovery': Trigger,
            '/peach/manipulation/check_reachability': CheckReachability,
            '/peach/manipulation/acknowledge_recovery': Trigger,
            '/peach/target_model/get_decision': GetDecision,
        }
        for name, kind in services.items():
            client = n.create_client(kind, name)
            self.assertTrue(client.wait_for_service(timeout_sec=20.0), f'{name} not ready')
            n.destroy_client(client)

    def test_4_launch_sent_nothing(self):
        n = self.node
        self.assertEqual(n.set_io_calls, 0, 'SetIO was called')
        self.assertEqual(n.begin_scene_calls, 0, 'BeginScene called without a batch')
        self.assertEqual(n.snapshot_calls, 0, 'BuildSceneSnapshot called without a batch')
        state = n.batch_state
        self.assertIsNotNone(state, 'no /peach/task/state')
        self.assertIn(state.phase, (state.IDLE,), f'batch phase {state.phase} without RunBatch')
        self.assertEqual(state.request_id, '')

    def test_5_bond_heartbeats_cpp(self):
        # Node-side evidence: each managed node publishes /bond. The test stack may run with
        # bond_timeout:=0 (manager creates no bonds) or the launch default 4.0.
        self.assertTrue(
            h.wait_until(lambda: {'peach2_manipulation', 'peach2_task'} <= self.node.bond_ids,
                         30.0),
            f'bond ids seen: {sorted(self.node.bond_ids)}')

    def test_6_bond_heartbeat_target_model(self):
        try:
            from bondpy.bondpy import Bond  # noqa: F401
        except ImportError:
            self.skipTest('ros-jazzy-bondpy not installed; target_model has no /bond')
        self.assertTrue(
            h.wait_until(lambda: 'peach2_target_model' in self.node.bond_ids, 15.0),
            f'bond ids seen: {sorted(self.node.bond_ids)}')

    def test_7_still_active(self):
        for name in MANAGED:
            self.assertEqual(self.node.lifecycle_state(name), LifecycleState.PRIMARY_STATE_ACTIVE,
                             f'{name} left Active')


@post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        h.check_exit_codes(self, proc_info)
