"""Zero-ROS tests for bag_report / retention（合成 bag 流，不 import rclpy）."""
import json
from pathlib import Path

from peach_executor.observability import bag_report as br
from peach_executor.observability import retention

_BASE = 1_700_000_000.0


def _ns(t: float) -> int:
    return int(t * 1e9)


def _ev(t, code, target_id='', request_id='run_A', message=''):
    return {
        'stamp': t, 'sequence': 0, 'severity': 0, 'severity_name': 'INFO',
        'code': code, 'message': message, 'request_id': request_id,
        'run_id': request_id, 'cycle_id': '', 'state_seq': 0,
        'target_id': target_id, 'details': {},
    }


def _tf_stamped(x=0.0, y=0.0, z=1.0, static=False):
    """一批 TFMessage 记录：动态 base→link1 + 静态 link1→tcp 两跳链."""
    dynamic = {
        'header': {'frame_id': 'base_link'},
        'child_frame_id': 'link1',
        'transform': {
            'translation': {'x': x, 'y': y, 'z': z},
            'rotation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
        },
    }
    static_tf = {
        'header': {'frame_id': 'link1'},
        'child_frame_id': 'tcp',
        'transform': {
            'translation': {'x': 0, 'y': 0, 'z': 0.1},
            'rotation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
        },
    }
    if static:
        return {'transforms': [static_tf]}
    return {'transforms': [dynamic]}


def _streams():
    events = [
        (_ns(_BASE), _ev(_BASE, 'target_dispatched', 't1')),
        (_ns(_BASE), _ev(_BASE, 'target_dispatched', 't2')),
        (_ns(_BASE + 10), _ev(
            _BASE + 10, 'target_succeeded', 't1',
            message=json.dumps({'reason': 'ok'}))),
        (_ns(_BASE + 12), _ev(
            _BASE + 12, 'target_failed', 't2',
            message=json.dumps({'failure_code': 'observe_failed'}))),
    ]
    states = [
        (_ns(_BASE), {'batch_state': 2, 'target_phase': 1, 'cycle_id': 'c1',
                      'target_id': 't1', 'run_id': 'run_A'}),
        (_ns(_BASE + 5), {'batch_state': 2, 'target_phase': 2,
                          'cycle_id': 'c1', 'target_id': 't1',
                          'run_id': 'run_A'}),
        (_ns(_BASE + 12), {'batch_state': 6, 'target_phase': 9,
                           'cycle_id': 'c1', 'target_id': 't1',
                           'run_id': 'run_A'}),
    ]
    targets = [
        (_ns(_BASE + 1.0), {'stamp': _BASE + 1.0, 'target_count': 2,
                            'observations': [
                                {'target_id': 't1', 'priority': 1},
                                {'target_id': 't2', 'priority': 2}]}),
        (_ns(_BASE + 1.4), {'stamp': _BASE + 1.4, 'target_count': 2,
                            'observations': []}),
        (_ns(_BASE + 1.8), {'stamp': _BASE + 1.8, 'target_count': 1,
                            'observations': []}),
    ]
    tf = [
        (_ns(_BASE + 3.0), _tf_stamped(z=1.0)),
        (_ns(_BASE + 4.0), _tf_stamped(z=1.001)),  # 3mm 门槛内：应被丢弃
        (_ns(_BASE + 5.0), _tf_stamped(z=1.05)),
    ]
    tf_static = [(_ns(_BASE), _tf_stamped(static=True))]
    recon = [(_ns(_BASE + 6.6), {
        'state': 'done', 'target_id': 't1', 'tf_failures': 0,
        'captured_views': 3})]
    decision = [(_ns(_BASE + 6.5), {
        'allowed': True, 'reason': 'dynamic_budget_accept'})]
    debug = [(_ns(_BASE + 6.2), {'tsdf': {'points': 123}})]
    status = [(_ns(_BASE + 6.0), {
        'data': json.dumps({'state': 'surveying'})})]
    job = [(_ns(_BASE + 7.0), {'data': json.dumps({
        'target_id': 't1', 'active_id': 'lock',
        'stages': [{'id': 'observe', 'status': 'done'}],
        'grasp': {'allowed': True, 'reason': 'ok'}, 'coords': {},
        'flags': {}})})]
    metrics = [(_ns(_BASE + 8.0), {
        'data': json.dumps({'cpu_percent': 40.0, 'memory_percent': 30.0})})]
    return {
        br.T_EVENTS: events,
        br.T_STATE: states,
        br.T_TARGETS: targets,
        br.T_TF: tf,
        br.T_RECON_DIAG: recon,
        br.T_RECON_DEBUG: debug,
        br.T_RECON_STATUS: status,
        br.T_DECISION: decision,
        br.T_TF_STATIC: tf_static,
        br.T_JOB: job,
        br.T_METRICS: metrics,
    }


def test_build_target_rows_outcomes():
    streams = _streams()
    events = br._events_records(streams)
    rows = br.build_target_rows(events, {'t1': 1, 't2': 2})
    by_id = {row['target_id']: row for row in rows}
    assert by_id['t1']['outcome'] == 'succeeded'
    assert by_id['t1']['duration_s'] == 10.0
    assert by_id['t2']['outcome'] == 'failed'
    assert by_id['t2']['reason'] == 'observe_failed'


def test_gate_rows_pass():
    streams = _streams()
    perception = br.perception_stats(br._targets_records(streams))
    assert perception['fps'] == 2.5
    recon = br.reconstruction_final(br.merge_reconstruction_streams(streams))
    rows = br.build_target_rows(br._events_records(streams), {})
    gates = br._gate_rows(perception, recon, rows)
    verdicts = [row[3] for row in gates]
    assert verdicts == ['✓', '✓', '✓']


def test_merge_reconstruction_merges_debug_and_decision():
    records = br.merge_reconstruction_streams(_streams())
    topics = [item['topic'] for item in records]
    assert topics == ['status', 'grasp_decision', 'diagnostics']
    diag = records[-1]['data']
    assert diag['tsdf'] == {'points': 123}
    assert diag['grasp_decision']['allowed'] is True
    assert diag['tf_failures'] == 0


def test_tcp_points_min_step_gate_and_phases():
    streams = _streams()
    points = br.tcp_points_from_tf(streams)
    # link1→tcp 静态偏移 0.1：z = 1.0+0.1 / 1.05+0.1
    assert [round(point['z'], 3) for point in points] == [1.1, 1.15]
    annotated = br.annotate_tf(points, br._state_records(streams))
    assert annotated[0]['phase'] == 1
    assert annotated[1]['phase'] == 2


def test_build_session_report_end_to_end():
    report, markdown = br.build_session_report(_streams(), bag_dir='/r/x/bag',
                                               session_dir='/r/x')
    assert report['gates']['succeeded'] == 1
    assert report['requests'][0]['request_id'] == 'run_A'
    assert report['requests'][0]['batch_state'] == 'COMPLETED'
    assert report['requests'][0]['terminal'] is True
    assert '采摘会话分析' in markdown
    assert '验收门对照' in markdown
    assert '`/r/x/bag`' in markdown
    assert '| ✓ |' in markdown or '✓' in markdown


def test_select_bags_oldest_first_and_keep():
    entries = [
        retention.BagEntry(Path('/r/session_a/bag'), 100, 1.0, 'session_bag'),
        retention.BagEntry(Path('/r/session_b/bag'), 60, 2.0, 'session_bag'),
        retention.BagEntry(Path('/r/session_c/bag'), 60, 3.0, 'session_bag'),
    ]
    doomed = retention.select_bags_to_delete(entries, 150)
    assert [item.path.parent.name for item in doomed] == ['session_a']
    doomed = retention.select_bags_to_delete(
        entries, 150, keep=['/r/session_a/bag'])
    assert [item.path.parent.name for item in doomed] == [
        'session_b', 'session_c']
    assert retention.select_bags_to_delete(entries, 0) == []
    assert retention.select_bags_to_delete(entries, 10 ** 9) == []
