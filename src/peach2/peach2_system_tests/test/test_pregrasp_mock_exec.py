"""
Isolated launch_testing: PREGRASP_ONLY executed on mock hardware (never a real robot).

SetEnables(execution=true, grasp=false, tool=false) -> RunBatch(PREGRASP_ONLY). The mock joints
must reach the pregrasp, the batch must stop for an operator ACK, and
/peach/task/acknowledge_recovery must let it finish. SetIO may never be called.
"""
from pathlib import Path
import sys
import time
import unittest

from launch_testing import post_shutdown_test
from peach2_interfaces.action import RunBatch
from peach2_interfaces.msg import BatchState, HarvestResult
import pytest
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parent))
import peach2_harness as h  # noqa: E402, I100

MANAGED = ('peach2_target_model', 'peach2_manipulation', 'peach2_task')
MOVED_RAD = 0.05
PREGRASP_TOL_M = 0.02


class TestPregraspMockExec(unittest.TestCase):

    joints0 = None
    pregrasp = None
    handle = None
    min_dist = float('inf')
    max_delta = 0.0

    @classmethod
    def setUpClass(cls):
        cls.node = h.Session.get(bags=(h.BAG_A,))

    @classmethod
    def tearDownClass(cls):
        node = h.Session._instance.node if h.Session._instance else None
        if node is not None:
            node.set_enables(False)
        h.Session.shutdown()

    def sample(self):
        cls = type(self)
        tcp = self.node.tcp_position()
        if tcp is not None and cls.pregrasp is not None:
            cls.min_dist = min(cls.min_dist, h.distance(tcp, cls.pregrasp))
        joints = self.node.joints()
        if joints is not None and cls.joints0 is not None:
            cls.max_delta = max(cls.max_delta, h.max_joint_delta(cls.joints0, joints))

    def test_0_stack_active(self):
        for name in MANAGED:
            self.assertTrue(self.node.wait_active(name, 180.0), f'{name} not active')
        self.assertTrue(self.node.wait_controllers(h.MOCK_CONTROLLERS, 60.0),
                        f'controllers active: {self.node.active_controllers()}')
        self.assertTrue(h.wait_until(lambda: self.node.joints() is not None, 30.0))
        type(self).joints0 = self.node.joints()

    def test_1_model_and_decision(self):
        ok = self.node.wait_converged(h.BAG_A.target_id, 90.0)
        self.assertTrue(ok, f'model not converged: {self.node.model(h.BAG_A.target_id)}')
        response = self.node.decision(h.BAG_A.target_id)
        self.assertIsNotNone(response)
        self.assertTrue(response.found)
        d = response.decision
        self.assertTrue(d.approach_allowed, f'approach not allowed: {d.reason}')
        p = d.pregrasp_tcp.position
        type(self).pregrasp = (p.x, p.y, p.z)

    def test_2_enable_execution_only(self):
        response = self.node.set_enables(True, False, False)
        self.assertIsNotNone(response, 'set_enables unavailable')
        self.assertTrue(response.accepted, response.message)
        self.assertTrue(h.wait_until(
            lambda: self.node.enables is not None and self.node.enables.execution, 10.0))
        e = self.node.enables
        self.assertFalse(e.grasp or e.tool)

    def test_3_run_batch_reaches_pregrasp_and_waits_for_ack(self):
        n = self.node
        goal = RunBatch.Goal()
        goal.request_id = f'lt_pregrasp_{int(time.time())}'
        goal.intent = RunBatch.Goal.INTENT_PREGRASP_ONLY
        goal.tool_id = h.TOOL_ID
        goal.target_ids = [h.BAG_A.target_id]
        goal.max_targets = 1
        goal.per_target_timeout_s = 300.0
        handle = n.send_goal(n.run_batch, goal)
        self.assertIsNotNone(handle, 'RunBatch rejected or server missing')
        type(self).handle = handle
        result_future = handle.get_result_async()
        type(self).result_future = result_future

        def waiting_ack() -> bool:
            self.sample()
            s = n.batch_state
            return result_future.done() or (
                s is not None and s.request_id == goal.request_id
                and s.phase == BatchState.WAITING_ACK)

        h.wait_until(waiting_ack, 420.0, 0.1)
        s = n.batch_state
        print(f'[lt] phases={n.batch_phases} state={s} min_dist={type(self).min_dist:.4f} '
              f'max_delta={type(self).max_delta:.3f}', flush=True)
        if result_future.done():
            r = result_future.result().result
            self.fail(f'batch ended before ACK: {r.termination_reason} '
                      f'{[(x.outcome, x.failure_code, x.reason) for x in r.results]}')
        self.assertEqual(s.phase, BatchState.WAITING_ACK, f'batch state {s}')
        self.assertTrue(s.recovery_required)
        self.assertGreater(type(self).max_delta, MOVED_RAD, 'mock arm did not move')
        self.assertLess(type(self).min_dist, PREGRASP_TOL_M, 'tcp never came near the pregrasp')
        self.assertEqual(n.set_io_calls, 0, 'SetIO was called')
        # Still blocked without an ACK.
        time.sleep(3.0)
        self.assertFalse(result_future.done(), 'batch finished without an operator ACK')
        self.assertEqual(n.batch_state.phase, BatchState.WAITING_ACK)

    def test_4_ack_finishes_batch(self):
        n = self.node
        future = getattr(type(self), 'result_future', None)
        self.assertIsNotNone(future, 'no batch in flight')
        response = n.call(n.ack_client, Trigger.Request(), 15.0)
        self.assertIsNotNone(response, 'acknowledge_recovery unavailable')
        self.assertTrue(response.success, response.message)
        wrapped = h.wait_future(future, 180.0)
        self.assertIsNotNone(wrapped, 'batch did not finish after ACK')
        result = wrapped.result
        summary = [(r.target_id, r.outcome, r.reached, r.failure_code, r.reason, r.plan_only)
                   for r in result.results]
        print(f'[lt] RunBatch done: termination={result.termination_reason!r} '
              f'results={summary}', flush=True)
        self.assertEqual(len(result.results), 1, summary)
        r = result.results[0]
        type(self).harvest_result = r
        self.assertEqual(r.outcome, HarvestResult.OUTCOME_SUCCEEDED, r.reason)
        self.assertFalse(r.plan_only)
        self.assertEqual(r.reached, HarvestResult.REACHED_PREGRASP)
        self.assertEqual(result.succeeded, 1)
        self.assertTrue(h.wait_until(
            lambda: n.batch_state is not None and n.batch_state.phase == BatchState.COMPLETED,
            10.0), f'final batch state {n.batch_state}')
        self.assertEqual(n.set_io_calls, 0, 'SetIO was called')

    def test_4b_reached_is_pregrasp(self):
        r = getattr(type(self), 'harvest_result', None)
        self.assertIsNotNone(r, 'no harvest result from test_4')
        self.assertEqual(r.reached, HarvestResult.REACHED_PREGRASP)

    def test_5_resurvey_when_no_target(self):
        n = self.node
        goal = RunBatch.Goal()
        goal.request_id = f'lt_resurvey_{int(time.time())}'
        goal.intent = RunBatch.Goal.INTENT_PREGRASP_ONLY
        goal.tool_id = h.TOOL_ID
        goal.target_ids = ['bag_never_observed']
        goal.max_targets = 1
        goal.per_target_timeout_s = 60.0
        handle = n.send_goal(n.run_batch, goal)
        self.assertIsNotNone(handle, 'RunBatch rejected or server missing')
        wrapped = h.wait_future(handle.get_result_async(), 240.0)
        self.assertIsNotNone(wrapped, 'batch did not finish')
        result = wrapped.result
        print(f'[lt] RunBatch no-target: termination={result.termination_reason!r}', flush=True)
        self.assertEqual(len(result.results), 0)
        self.assertEqual(n.set_io_calls, 0, 'SetIO was called')
        self.assertFalse(result.termination_reason.startswith('survey_failed'),
                         result.termination_reason)

    def test_6_disable_execution(self):
        response = self.node.set_enables(False, False, False)
        self.assertIsNotNone(response)
        self.assertTrue(response.accepted, response.message)


@pytest.mark.launch_test
def generate_test_description():
    return h.stack_launch_description(), {}


@post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        h.check_exit_codes(self, proc_info)
