"""批次策略/补采清单纯核测试（重写轮 3c；随用户单独验证轮运行）."""
from __future__ import annotations

import json

from peach_harvester.cycle_core.batch_policy import (
    BatchPolicy,
    ratio_reached,
    rework_kind,
    ReworkList,
    TargetDeadline,
)


def test_policy_from_goal_defaults():
    p = BatchPolicy.from_goal({})
    assert p.target_harvest_ratio == 0.0
    assert p.view_policy == BatchPolicy.VIEW_FAST
    q = BatchPolicy.from_goal(
        {'target_harvest_ratio': 0.8, 'view_policy': 1,
         'per_target_timeout_s': 45.0})
    assert (q.target_harvest_ratio, q.view_policy,
            q.per_target_timeout_s) == (0.8, 1, 45.0)


def test_policy_with_updates_partial():
    p = BatchPolicy.from_goal({'per_target_timeout_s': 30.0})
    q = p.with_updates(view_policy=1)
    assert q.per_target_timeout_s == 30.0 and q.view_policy == 1


def test_deadline_semantics():
    d = TargetDeadline(started_s=100.0, timeout_s=45.0)
    assert not d.exceeded(now_s=144.0)
    assert d.exceeded(now_s=145.0)
    assert d.remaining_s(now_s=100.0) == 45.0
    unlimited = TargetDeadline(started_s=0.0, timeout_s=0.0)
    assert not unlimited.exceeded(now_s=1e9)
    assert unlimited.remaining_s() == float('inf')


def test_ratio_gate():
    p = BatchPolicy(target_harvest_ratio=0.8)
    assert not ratio_reached(7, 10, p)
    assert ratio_reached(8, 10, p)
    assert not ratio_reached(99, 10, BatchPolicy())  # 0=不限


def test_rework_list_roundtrip(tmp_path):
    rl = ReworkList(request_id='run_x')
    rl.append('t1', 'timeout', '单果时限超限', attempted=True)
    rl.append('t2', 'ratio_satisfied', '采收率已达', attempted=False)
    out = rl.save(tmp_path)
    loaded = ReworkList.load(tmp_path, 'run_x')
    assert loaded.entries == rl.entries
    assert loaded.kinds_count() == {'timeout': 1, 'ratio_satisfied': 1}
    assert json.loads(out.read_text())['request_id'] == 'run_x'


def test_rework_list_rejects_unknown_kind():
    rl = ReworkList(request_id='r')
    try:
        rl.append('t', 'mystery', 'x')
    except ValueError:
        return
    raise AssertionError('未知类别应拒绝')


def test_rework_kind_maps_failure_codes():
    assert rework_kind('timeout', 0) == 'timeout'
    assert rework_kind('operator_skip', 0) == 'operator_skipped'
    assert rework_kind('ik_no_solution', 0) == 'unreachable'
    assert rework_kind('observe_failed', 0) == 'occluded'
    assert rework_kind('sleeve_stuck', 0) == 'contact_failed'
    assert rework_kind('', 1) == 'quality'
    assert rework_kind('other', 3) == 'quality'
