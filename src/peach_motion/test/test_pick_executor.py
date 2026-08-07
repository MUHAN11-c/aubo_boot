"""PickExecutor 六阶段流程：假 MoveItClient 下的成功与 unreachable 路径."""
import threading
import time
import unittest

from geometry_msgs.msg import Pose
from peach_motion.pick_executor import PickExecutor
from peach_pose_msgs.action import Pick
from peach_pose_msgs.msg import BagGraspCandidate, BagGraspCandidateArray
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class _FakeMoveIt:
    """假 MoveItClient：记录调用，plan_only 可配失败（模拟不可达）."""

    def __init__(self, plan_only_ok=True):
        self.calls = []              # [(plan_only, ...), ...]
        self.plan_only_ok = plan_only_ok

    def plan_and_execute(self, pose, plan_only=False, cancel_check=None):
        self.calls.append(plan_only)
        if plan_only and not self.plan_only_ok:
            return False, 'unreachable'
        return True, ''


def _tcp():
    """固定 TCP（目标侧向 0.4m）."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0.3, 0.4, 0.5
    pose.orientation.w = 1.0
    return pose


def _candidates_msg():
    """单 ACCEPT 候选（target_0，轴 +Z）."""
    msg = BagGraspCandidateArray()
    c = BagGraspCandidate()
    c.target_id = 'target_0'
    c.status = 0
    c.confidence = 0.9
    c.entry_pose.position.x, c.entry_pose.position.y = 0.3, 0.0
    c.entry_pose.position.z = 0.5
    c.entry_pose.orientation.w = 1.0
    c.translation_direction.z = 1.0
    c.suggested_travel_m = 0.1
    msg.candidates.append(c)
    return msg


class PickExecutorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.fake = _FakeMoveIt()
        overrides = {'refine_timeout_sec': 0.2, 'min_support_frames': 99,
                     'orbit_points': 3, 'frame_gate_timeout_sec': 0.05}
        self.exec_node = PickExecutor(
            moveit_factory=lambda node, params: self.fake,
            tcp_provider=_tcp, param_overrides=overrides)
        self.client_node = Node('pick_test_client')
        self.publisher = self.client_node.create_publisher(
            BagGraspCandidateArray, '/peach_pose_node/grasp_candidates', 10)
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.exec_node)
        self.executor.add_node(self.client_node)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()
        self._stop_pub = threading.Event()
        self._pub_thread = threading.Thread(target=self._pub_loop, daemon=True)
        self._pub_thread.start()

    def tearDown(self):
        self._stop_pub.set()
        self._pub_thread.join(timeout=2.0)
        self.executor.shutdown()
        self.exec_node.destroy_node()
        self.client_node.destroy_node()
        self._spin.join(timeout=2.0)

    def _pub_loop(self):
        """50 Hz 灌候选帧（帧门控放行 + 找目标）."""
        while not self._stop_pub.is_set():
            self.publisher.publish(_candidates_msg())
            time.sleep(0.02)

    def _send_goal(self, execute_insert=False):
        client = ActionClient(self.client_node, Pick, '/pick_executor/pick')
        self.assertTrue(client.wait_for_server(timeout_sec=5.0))
        goal = Pick.Goal()
        goal.target_id = 'target_0'
        goal.execute_insert = execute_insert
        send_future = client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not send_future.done():
            self.assertLess(time.monotonic(), deadline, 'goal 发送超时')
            time.sleep(0.02)
        handle = send_future.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 10.0
        while not result_future.done():
            self.assertLess(time.monotonic(), deadline, '结果等待超时')
            time.sleep(0.02)
        return result_future.result().result

    def test_success_path_full_stages(self):
        """假 MoveIt 全成功：成功结果 + 弧线 2 点执行 + 1 次 plan_only 验证."""
        result = self._send_goal()
        self.assertTrue(result.success, result.message)
        self.assertIn('axis_gravity_deg', result.message)
        self.assertEqual(result.final_status, 0)
        plan_only_calls = [c for c in self.fake.calls if c]
        exec_calls = [c for c in self.fake.calls if not c]
        # RETREAT 也算执行段；弧线 3 点 → 步进 2 次 + RETREAT 1 次
        self.assertEqual(len(plan_only_calls), 1)
        self.assertEqual(len(exec_calls), 3)

    def test_unreachable_when_plan_only_fails(self):
        """FINAL_APPROACH plan_only 失败 → FAILED(unreachable) 并 RETREAT."""
        self.fake.plan_only_ok = False
        result = self._send_goal()
        self.assertFalse(result.success)
        self.assertIn('unreachable', result.message)


if __name__ == '__main__':
    unittest.main()
