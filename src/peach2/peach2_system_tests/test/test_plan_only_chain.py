"""
Isolated launch_testing: plan-only chain on the mock stack (execution disabled).

The test process publishes synthetic observations of two bags, waits for converged target models,
then sends RunBatch(PREGRASP_ONLY) with enables all false. Nothing may move and SetIO may never be
called. A direct HarvestTarget(PREGRASP_ONLY) checks the manipulation plan-only chain on its own.
"""
from pathlib import Path
import sys
import time
import unittest

from launch_testing import post_shutdown_test
from peach2_interfaces.action import HarvestTarget, RunBatch
from peach2_interfaces.msg import FailureCode, HarvestResult
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import peach2_harness as h  # noqa: E402, I100

MANAGED = ('peach2_target_model', 'peach2_manipulation', 'peach2_task')
STILL_RAD = 1e-3
# Batch-level plan-only outcomes that are acceptable when execution is disabled.
PLAN_ONLY_CODES = (
    FailureCode.NONE, FailureCode.SAFETY_GATE_CLOSED, FailureCode.PLAN_NO_IK,
    FailureCode.PLAN_COLLISION, FailureCode.PLAN_FAILED, FailureCode.PLAN_CARTESIAN_INCOMPLETE,
)


@pytest.mark.launch_test
def generate_test_description():
    return h.stack_launch_description(), {}


class TestPlanOnlyChain(unittest.TestCase):

    batch_result = None
    joints0 = None

    @classmethod
    def setUpClass(cls):
        cls.node = h.Session.get(bags=(h.BAG_A, h.BAG_B))

    @classmethod
    def tearDownClass(cls):
        h.Session.shutdown()

    def assert_still(self):
        now = self.node.joints()
        self.assertIsNotNone(now)
        delta = h.max_joint_delta(type(self).joints0, now)
        self.assertLess(delta, STILL_RAD, f'arm moved by {delta:.4f} rad with execution disabled')
        self.assertEqual(self.node.set_io_calls, 0, 'SetIO was called')

    def test_0_stack_active(self):
        for name in MANAGED:
            self.assertTrue(self.node.wait_active(name, 180.0), f'{name} not active')
        self.assertTrue(h.wait_until(lambda: self.node.joints() is not None, 30.0))
        type(self).joints0 = self.node.joints()

    def test_1_models_converge(self):
        for bag in (h.BAG_A, h.BAG_B):
            ok = self.node.wait_converged(bag.target_id, 90.0)
            self.assertTrue(ok, f'{bag.target_id} not converged: {self.node.model(bag.target_id)}')
        response = self.node.decision(h.BAG_A.target_id)
        self.assertIsNotNone(response, 'get_decision unavailable')
        self.assertTrue(response.found)
        self.assertTrue(response.decision.approach_allowed,
                        f'approach not allowed: {response.decision.reason}')

    def test_2_run_batch_execution_disabled(self):
        n = self.node
        self.assertTrue(h.wait_until(lambda: n.enables is not None, 10.0))
        self.assertFalse(n.enables.execution, 'execution must be disabled for this test')
        goal = RunBatch.Goal()
        goal.request_id = f'lt_plan_only_{int(time.time())}'
        goal.intent = RunBatch.Goal.INTENT_PREGRASP_ONLY
        goal.tool_id = h.TOOL_ID
        goal.target_ids = [h.BAG_A.target_id]
        goal.max_targets = 1
        goal.per_target_timeout_s = 120.0
        handle = n.send_goal(n.run_batch, goal)
        self.assertIsNotNone(handle, 'RunBatch rejected or server missing')
        wrapped = h.wait_future(handle.get_result_async(), 300.0)
        self.assertIsNotNone(wrapped, 'RunBatch did not finish')
        result = wrapped.result
        type(self).batch_result = result
        summary = [(r.target_id, r.outcome, r.failure_code, r.reason, r.plan_only)
                   for r in result.results]
        print(f'[lt] RunBatch plan-only: termination={result.termination_reason!r} '
              f'results={summary}', flush=True)
        self.assertTrue(result.termination_reason, 'empty termination_reason')
        for r in result.results:
            self.assertIn(r.failure_code, PLAN_ONLY_CODES, f'unexpected failure: {r.reason}')
            self.assertEqual(r.reached, HarvestResult.REACHED_NONE)
        self.assert_still()

    def test_3_run_batch_reports_plan_only(self):
        result = type(self).batch_result
        self.assertIsNotNone(result, 'no RunBatch result from test_2')

        def check():
            self.assertFalse(result.termination_reason.startswith('safety:'),
                             result.termination_reason)
            self.assertTrue(result.results, 'no per-target results')
            self.assertTrue(result.results[0].plan_only)

        h.xfail(self, 'README 已知失败 #2: peach2_task requires execution for PREGRASP_ONLY, '
                'batch aborts at CheckSafety before any plan-only result', check)

    def test_4_harvest_target_plan_only(self):
        n = self.node
        goal = HarvestTarget.Goal()
        goal.request_id = f'lt_direct_plan_{int(time.time())}'
        goal.target_id = h.BAG_A.target_id
        goal.tool_id = h.TOOL_ID
        goal.mode = HarvestTarget.Goal.MODE_PREGRASP_ONLY
        handle = n.send_goal(n.harvest, goal)
        self.assertIsNotNone(handle, 'HarvestTarget rejected or server missing')
        wrapped = h.wait_future(handle.get_result_async(), 180.0)
        self.assertIsNotNone(wrapped, 'HarvestTarget did not finish')
        r = wrapped.result.result
        print(f'[lt] HarvestTarget plan-only: outcome={r.outcome} code={r.failure_code} '
              f'reason={r.reason!r} plan_only={r.plan_only} reached={r.reached}', flush=True)
        self.assertTrue(r.plan_only, 'execution disabled but result not plan_only')
        self.assertEqual(r.reached, HarvestResult.REACHED_NONE)
        self.assertEqual(r.outcome, HarvestResult.OUTCOME_SKIPPED)
        self.assertEqual(r.failure_code, FailureCode.NONE, f'plan chain failed: {r.reason}')
        self.assertFalse(r.recovery_required)
        self.assert_still()

    def test_5_nothing_moved(self):
        self.assert_still()
        self.assertNotEqual(self.node.recovery, True, 'manipulation recovery latched')


@post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        h.check_exit_codes(self, proc_info)
