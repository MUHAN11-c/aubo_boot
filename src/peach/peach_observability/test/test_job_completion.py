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


def test_zero_refined_axis_is_unavailable_not_a_unit_direction():
    data = snapshot()
    data['refined'] = {'axis': {'xyz': [0.0, 0.0, 0.0]}}
    assert build_harvest_job(data)['coords']['refined_axis'] is None


def test_geometry_analysis_compares_visible_span_with_commanded_travel():
    data = snapshot()
    bottom = [0.36271119837874255, -0.8527248670953206, 0.584436819928776]
    neck = [0.395449295019497, -0.8742648632634334, 0.6722344251242477]
    data['perception']['targets']['observations'] = [{
        'target_id': 'target_0',
        'candidate': {
            'bag_bottom': bottom,
            'bag_neck': neck,
            'diagnostic_flags': ['gravity_neck_fallback', 'axis_from_primary'],
        },
        'fitting': {'valid_depth_ratio': 0.957, 'n_points': 2451},
    }]
    data['reconstruction'] = {'grasp_decision': {
        'allowed': True,
        'reason': 'bag_mouth_geometry',
        'tool_profile_id': 'bite_shear_v1',
        'entry': bottom,
        'cut_pose': neck,
        'axis': [0.340501769642865, -0.22403278033618573, 0.9131636535719738],
        'd95_m': 0.08934070513962294,
        'travel_m': 0.09614662700605572,
        'cut_travel_m': 0.09614662700605572,
        'radial_margin_m': -0.018307343172584166,
        'axial_margin_m': -0.002,
    }}
    analysis = build_harvest_job(data)['analysis']
    assert abs(analysis['bottom_neck_m'] - 0.096146627) < 1e-6
    assert abs(analysis['entry_cut_m'] - analysis['cut_travel_m']) < 1e-9
    assert analysis['d95_m'] == 0.08934070513962294
    assert analysis['radial_margin_m'] < 0.0
    assert analysis['cut_radius_m'] > analysis['entry_radius_m']
    assert analysis['flags'] == ['gravity_neck_fallback', 'axis_from_primary']
    assert analysis['n_points'] == 2451
    assert analysis['tool_profile_id'] == 'bite_shear_v1'


def test_geometry_analysis_stays_empty_without_points():
    analysis = build_harvest_job(snapshot())['analysis']
    assert analysis['bottom_neck_m'] is None
    assert analysis['entry_cut_m'] is None
    assert analysis['flags'] == []


def test_valid_refined_axis_preserves_measured_direction():
    data = snapshot()
    data['refined'] = {'axis': {'xyz': [0.0, 0.0, 1.0]}}
    assert build_harvest_job(data)['coords']['refined_axis'] == [0.0, 0.0, 1.0]


def test_real_pregrasp_canceled_snapshot_regression():
    path = Path('/tmp/peach_1710_live.json')
    if not path.exists():
        return
    job = build_harvest_job(json.loads(path.read_text(encoding='utf-8')))
    assert '预抓取已验证' in job['why']
    assert '取消' in job['why']
