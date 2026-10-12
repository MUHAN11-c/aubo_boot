"""终局显示采用账本完成证据，取消不抹掉已验证的预抓取."""
import json
from pathlib import Path

from peach_observability.job import build_harvest_job


def snapshot(level=2, run_id='r', ledger_id='r'):
    return {
        'task_executor': {'state': {
            'run_id': run_id, 'cycle_id': f'{run_id}:target_0',
            'batch_state': 8, 'target_phase': 0, 'recovery_required': True,
            'execution_enabled': False, 'grasp_enabled': True,
            'tool_enabled': False, 'message': 'interrupted'}},
        'perception': {'targets': {'selected_target_id': 'target_0'}},
        'manipulation': {'status': {'state': 'CANCELED', 'running': False}},
        'ledger': {'live': {'request_id': ledger_id, 'rows': [{
            'target_id': 'target_0', 'outcome_name': 'CANCELED',
            'completion_level': level, 'reason': 'canceled',
            'failure_code': 'canceled'}]}},
    }


def test_cancellation_preserves_verified_pregrasp():
    job = build_harvest_job(snapshot())
    assert '预抓取已验证' in job['why']
    assert '取消' in job['why']
    statuses = {s['id']: s['status'] for s in job['stages']}
    assert statuses['approach'] == 'done'
    assert statuses['tool'] != 'done'
    assert job['completion']['completion_level'] == 2


def test_ledger_from_previous_batch_cannot_claim_current_completion():
    job = build_harvest_job(snapshot(ledger_id='previous'))
    assert not job.get('completion')
    assert '预抓取已验证' not in job['why']


def test_retreat_level_does_not_claim_harvest():
    job = build_harvest_job(snapshot(level=6))
    assert '撤退已确认' in job['why']
    assert '采摘已确认' not in job['why']


def test_real_pregrasp_canceled_snapshot_regression():
    path = Path('/tmp/peach_1710_live.json')
    if not path.exists():
        return
    job = build_harvest_job(json.loads(path.read_text(encoding='utf-8')))
    assert '预抓取已验证' in job['why']
    assert '取消' in job['why']
