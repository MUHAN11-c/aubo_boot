"""Offline contracts: these tests never initialize ROS or open a camera."""
import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parents[1] / 'scripts/run_live_camera_mock.py'


def load():
    spec = importlib.util.spec_from_file_location('live_camera_mock', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_defaults_and_no_real_hardware_switch():
    m = load()
    args = m.parser().parse_args([])
    assert (args.rounds, args.models, args.soak_seconds, args.domain, args.rviz) == (10, 30, 1800, 86, True)
    with pytest.raises(SystemExit):
        m.parser().parse_args(['--hardware-mode', 'real'])
    assert m.parser().parse_args(['--no-rviz']).rviz is False


def test_accepted_and_skipped_are_not_full_cycle_success():
    m = load()
    assert not m.full_cycle({'accepted': True}, 'r')
    assert not m.full_cycle({'request_id': 'r', 'rows': [
        {'outcome_name': 'SKIPPED_UNREACHABLE', 'completion_level': 6}]}, 'r')
    assert m.full_cycle({'request_id': 'r', 'rows': [
        {'outcome_name': 'SUCCEEDED', 'completion_level': 6}]}, 'r')
    assert not m.full_cycle({'request_id': 'old', 'rows': [
        {'outcome_name': 'SUCCEEDED', 'completion_level': 6}]}, 'r')


def test_parameter_filter_keeps_false_zero_and_rejects_untyped_values():
    assert load().rviz_parameters({'a': None, 'b': [], 'c': False, 'd': 0}) == {'c': False, 'd': 0}


def test_target_rotation_and_model_heartbeat_deduplication():
    m = load()
    targets = {'observations': [{'target_id': 'target_2', 'tracking_status': 'TRACKED'},
                                {'target_id': 'target_1', 'tracking_status': 'LOST'},
                                {'target_id': 'target_3', 'tracking_status': 'TRACKED'}]}
    assert m.choose_target(targets, 0) == 'target_2'
    assert m.choose_target(targets, 1) == 'target_3'
    assert m.model_key({'target_id': 't', 'scene_epoch': 2, 'model_revision': 'm'}) == ('t', 2, 'm')
    assert m.model_key({'model_revision': 'm'}) is None


def test_group_cleanup_only_signals_owned_sessions(monkeypatch):
    m = load()
    called = []
    monkeypatch.setattr(m.os, 'killpg', lambda pgid, sig: called.append(pgid))
    owner = m.OwnedProcesses({})
    owner.signal_group(123456, m.signal.SIGINT)
    assert called == []


def test_percentile_and_recovery_require_current_terminal_evidence():
    m = load()
    assert m.statistics([1, 2, 3])['p95'] == pytest.approx(2.9)
    assert not m.can_ack('real', 'r', {'run_id': 'r', 'recovery_required': True}, True)
    assert not m.can_ack('mock', 'r', {'run_id': 'old', 'recovery_required': True}, True)
    assert not m.can_ack('mock', 'r', {'run_id': 'r', 'recovery_required': True}, False)
    assert m.can_ack('mock', 'r', {'run_id': 'r', 'recovery_required': True}, True)


def test_test_profile_and_rviz_queues_are_bounded():
    import yaml
    cfg = Path(__file__).parents[1] / 'config'
    profile = yaml.safe_load((cfg / 'live_camera_mock.yaml').read_text())
    assert profile['schema_version'] == 1
    assert profile['hardware_mode'] == 'mock'
    assert profile['tool_enabled'] is False
    assert len(profile['camera_calibration']['xyz_m']) == 3
    assert len(profile['mock_initial_positions']) == 6
    def check(node):
        if isinstance(node, dict):
            typ = node.get('Class')
            if typ in ('rviz_default_plugins/Image', 'rviz_default_plugins/PointCloud2'):
                assert node['Topic']['Depth'] == 1
                assert node['Topic']['Reliability Policy'] == 'Best Effort'
            if typ == 'rviz_default_plugins/PointCloud2':
                assert node['Decay Time'] == 0
                assert node['Style'] == 'Points'
            for value in node.values():
                check(value)
        elif isinstance(node, list):
            for value in node:
                check(value)
    check(yaml.safe_load((cfg / 'live_camera_mock.rviz').read_text()))


def test_action_wait_timeout_is_explicit_unknown():
    from types import SimpleNamespace
    driver = object.__new__(load().RosDriver)
    driver.node = None
    driver.ros = SimpleNamespace(spin_until_future_complete=lambda *a, **k: None)
    with pytest.raises(TimeoutError, match='outcome unknown'):
        driver.wait(SimpleNamespace(done=lambda: False), .01)


def test_display_absence_cannot_silently_pass_rviz(monkeypatch):
    m = load()
    monkeypatch.setattr(m.Path, 'glob', lambda *args: [])
    with pytest.raises(RuntimeError, match='DISPLAY'):
        m.gui_environment({'QT_QPA_PLATFORM_PLUGIN_PATH': '/cv2/plugins'})
    env = m.gui_environment({'DISPLAY': ':0', 'QT_QPA_PLATFORM_PLUGIN_PATH': '/cv2/plugins'})
    assert 'QT_QPA_PLATFORM_PLUGIN_PATH' not in env
    assert env['QT_QPA_PLATFORM'] == 'xcb'


def history(name):
    import json
    return json.loads((Path(__file__).parent / 'fixtures' / f'live_camera_mock_{name}.json').read_text())


def test_historical_ready_uses_real_checks_and_domain():
    m = load()
    state = history('complete')
    assert 'stack_ready' not in state['task_executor']['state']
    assert m.startup_ready(state, 86)
    assert not m.startup_ready(state, 85)


def test_historical_hold_ack_precedes_batch_result_without_fake_ledger():
    m = load()
    state = history('hold')
    task = state['task_executor']['state']
    assert state['ledger']['live']['rows'] == []
    proof = m.ack_evidence(state, task['run_id'], task['target_id'])
    assert proof['source'] == 'canonical_target_succeeded'
    assert proof['completion_level'] == 6
    assert m.ack_evidence(state, 'old', task['target_id']) is None
    assert m.ack_evidence(state, task['run_id'], 'other') is None


def test_bounded_stream_evidence_preserves_total_across_eviction():
    m = load()
    stream = m.StreamEvidence(3)
    for stamp in range(100):
        stream.add(stamp)
    assert len(stream.stamps) == 3
    assert stream.total == 100
    assert stream.evidence(90)['new_frames'] == 10


def test_soak_failure_is_not_hidden_by_successful_rounds():
    m = load()
    assert not m.soak_passed(10, 2, [{'streams_fresh': True, 'rviz_alive': True}])
    assert not m.soak_passed(10, 10, [{'streams_fresh': False, 'rviz_alive': True}])
    assert m.soak_passed(0, 0, [])


def test_action_tick_can_ack_before_batch_future_finishes(monkeypatch):
    import sys
    from types import SimpleNamespace
    m = load()
    complete = {'value': False, 'ticks': 0}
    result = SimpleNamespace(status=4, result='historical terminal')
    future = SimpleNamespace(done=lambda: complete['value'], result=lambda: result)
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: future)
    client = SimpleNamespace(wait_for_server=lambda **kw: True,
                             send_goal_async=lambda goal: SimpleNamespace(result=lambda: handle),
                             destroy=lambda: None)
    monkeypatch.setitem(sys.modules, 'rclpy.action', SimpleNamespace(ActionClient=lambda *a: client))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.convert',
                        SimpleNamespace(message_to_ordereddict=lambda value: {'success': True}))
    driver = object.__new__(m.RosDriver)
    driver.node, driver.active = None, []
    driver.ros = SimpleNamespace(spin_once=lambda *a, **k: None)
    driver.wait = lambda f, timeout: f.result()
    def tick():
        assert not future.done()
        complete['ticks'] += 1
        complete['value'] = True
    outcome = driver.action(None, '/run_harvest', None, 1, on_tick=tick)
    assert outcome['status'] == 4
    assert complete['ticks'] == 1
    assert driver.active == []


def test_signal_handlers_restore_previous_owners(monkeypatch):
    m = load()
    previous = {m.signal.SIGINT: object(), m.signal.SIGTERM: object()}
    installed = dict(previous)
    monkeypatch.setattr(m.signal, 'getsignal', lambda sig: installed[sig])
    monkeypatch.setattr(m.signal, 'signal', lambda sig, fn: installed.__setitem__(sig, fn))
    def interrupted(args):
        with pytest.raises(KeyboardInterrupt):
            installed[m.signal.SIGTERM](m.signal.SIGTERM, None)
        raise RuntimeError('runtime failure')
    monkeypatch.setattr(m, 'run', interrupted)
    with pytest.raises(RuntimeError):
        m.main(['--no-rviz'])
    assert installed == previous


def test_cancel_failure_still_disables_before_context_close():
    from types import SimpleNamespace
    m = load()
    calls = []
    def cancel():
        calls.append('cancel')
        raise RuntimeError('cancel failed')
    driver = SimpleNamespace(cancel=cancel,
        enables=lambda *args: calls.append('disable'), close=lambda: calls.append('close'))
    errors = []
    m.cleanup_driver(driver, errors)
    assert calls == ['cancel', 'disable', 'close']
    assert errors == ['cancel: cancel failed']


def test_close_does_not_shutdown_an_already_closed_context():
    from types import SimpleNamespace
    m = load()
    calls = []
    driver = object.__new__(m.RosDriver)
    driver.node = SimpleNamespace(destroy_node=lambda: calls.append('destroy'))
    driver.ros = SimpleNamespace(ok=lambda: False, shutdown=lambda: calls.append('shutdown'))
    driver.close()
    assert calls == ['destroy']


def test_owned_groups_are_audited_immediately_on_start(monkeypatch, tmp_path):
    import json
    from types import SimpleNamespace
    m = load()
    captured = []
    monkeypatch.setattr(m.subprocess, 'Popen', lambda *a, **kw: SimpleNamespace(pid=12345))
    manager = m.OwnedProcesses({}, tmp_path / 'owned.json', on_start=lambda pid: captured.append(pid))
    manager.start(['ros2', 'launch', 'mock'], tmp_path / 'log')
    assert captured == [12345]
    audit = json.loads((tmp_path / 'owned.json').read_text())
    assert audit['processes'][0]['pgid'] == 12345
    for handle in manager.logs:
        handle.close()


def test_ros_init_leaves_signal_ownership_with_runner(monkeypatch):
    import sys
    from types import SimpleNamespace
    m = load()
    calls = []
    no_handlers = object()
    node = SimpleNamespace(create_subscription=lambda *args: None,
                           destroy_node=lambda: calls.append('destroy'))
    ros = SimpleNamespace(init=lambda **kw: calls.append(kw), ok=lambda: True,
                          shutdown=lambda: calls.append('shutdown'))
    for name, value in {
        'rclpy': ros, 'rclpy.node': SimpleNamespace(Node=lambda name: node),
        'rclpy.signals': SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=no_handlers)),
        'rclpy.qos': SimpleNamespace(qos_profile_sensor_data=object()),
        'sensor_msgs.msg': SimpleNamespace(Image=object(), JointState=object()),
    }.items():
        monkeypatch.setitem(sys.modules, name, value)
    driver = m.RosDriver()
    assert calls[0] == {'signal_handler_options': no_handlers}
    driver.close()
    assert calls[-2:] == ['destroy', 'shutdown']


def test_real_empty_refined_snapshot_is_safe_and_not_a_model():
    import json
    m = load()
    state = json.loads((Path(__file__).parent / 'fixtures/live_camera_mock_empty_model.json').read_text())
    assert state['reconstruction']['diagnostics']['refined'] is None
    assert m.section(state, 'reconstruction', 'diagnostics', 'refined') == {}
    assert m.build_generation({}) is None


def matching_model_and_state():
    model = {'target_id': 't', 'scene_epoch': 1, 'model_revision': 'rev',
             'run_id': 'run', 'calibration_revision': 'cal', 'config_revision': 'cfg',
             'tool_profile_id': 'tool',
             'generated_at': {'sec': 100, 'nanosec': 500000000},
             'valid_until': {'sec': 105, 'nanosec': 500000000}}
    decision = {k: v for k, v in model.items() if k not in ('generated_at', 'valid_until', 'run_id')}
    decision.update(harvest_run_id='run', stamp=100.5, valid_until=105.5,
                    entry=[.1, .2, .3], cut_pose=[.1, .2, .4], axis=[0., 0., 1.], d95_m=.06)
    return model, {'reconstruction': {'diagnostics': {'model_stamp': 100.5},
                                     'grasp_decision': decision}}


def test_model_generation_uses_target_model_idl_times_not_heartbeat():
    m = load()
    model, state = matching_model_and_state()
    assert m.build_generation(model) == ('t', 1, 'rev', 100.5, 105.5)
    assert m.web_model_matches(state, model)
    state['reconstruction']['diagnostics']['stamp'] = 99999
    assert m.web_model_matches(state, model)
    state['reconstruction']['diagnostics']['model_stamp'] = 99
    assert not m.web_model_matches(state, model)


@pytest.mark.parametrize('field,value', [('harvest_run_id', 'old'), ('scene_epoch', 0),
    ('model_revision', 'old'), ('tool_profile_id', 'old'), ('calibration_revision', ''),
    ('config_revision', ''), ('stamp', 99), ('valid_until', 999)])
def test_web_model_rejects_incomplete_or_mismatched_decision(field, value):
    m = load()
    model, state = matching_model_and_state()
    state['reconstruction']['grasp_decision'][field] = value
    assert not m.web_model_matches(state, model)


def test_geometry_statistics_use_actual_matched_decision_entry():
    m = load()
    model, state = matching_model_and_state()
    evidence = m.model_geometry(model, state['reconstruction']['grasp_decision'])
    stats = m.geometry_statistics([{'identity': ('t', 1, 'rev'), 'geometry': evidence}])['t']
    assert stats['d95_m']['count'] == 1
    assert stats['entry_0_m']['min'] == .1
    assert stats['axis_angle_from_first_deg']['max'] == 0.


@pytest.mark.parametrize('field,value', [('d95_m', 0.), ('axis', [0., 0., 0.]),
                                        ('cut_pose', [.1, .2, .3])])
def test_geometry_statistics_reject_degenerate_evidence(field, value):
    m = load()
    model, state = matching_model_and_state()
    decision = state['reconstruction']['grasp_decision']
    decision[field] = value
    with pytest.raises(ValueError):
        m.model_geometry(model, decision)


def test_launch_cleanup_delivers_sigint_once_via_launch_leader(monkeypatch):
    from types import SimpleNamespace
    m = load()
    calls = []
    monkeypatch.setattr(m.os, 'kill', lambda pid, sig: calls.append(('pid', pid, sig)))
    monkeypatch.setattr(m.os, 'killpg', lambda pid, sig: calls.append(('group', pid, sig)))
    owner = m.OwnedProcesses({})
    launch = SimpleNamespace(pid=123, args=['ros2', 'launch', 'peach_bringup', 'harvest_system.launch.py'],
                             poll=lambda: None)
    owner.processes = [launch]
    owner.interrupt(launch)
    assert calls == [('pid', 123, m.signal.SIGINT)]
    # Once the leader has exited, remaining owned children still need a signal.
    launch.poll = lambda: 0
    owner.interrupt(launch)
    assert calls[-1] == ('group', 123, m.signal.SIGINT)


def test_physical_bag_coverage_survives_scene_target_renaming():
    m = load()
    models = [{'identity': ('target_0', 1, 'm0'), 'geometry': {'entry': [0., 0., .5]}},
              {'identity': ('target_1', 1, 'm1'), 'geometry': {'entry': [.2, 0., .5]}}]
    rounds = [{'success': True, 'entry_position': [.001, 0., .5]},
              {'success': True, 'entry_position': [.201, 0., .5]}]
    assert m.physical_coverage(models, rounds) == {'target_0': 1, 'target_1': 1}
    rounds.append({'success': False, 'entry_position': [.201, 0., .5]})
    assert m.physical_coverage(models, rounds)['target_1'] == 1


def test_cancel_timeout_never_swallows_operator_interruption(monkeypatch):
    import sys
    from types import SimpleNamespace
    m = load()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: SimpleNamespace(done=lambda: False),
                             cancel_goal_async=lambda: 'cancel')
    client = SimpleNamespace(wait_for_server=lambda **kw: True, send_goal_async=lambda goal: 'send',
                             destroy=lambda: None)
    monkeypatch.setitem(sys.modules, 'rclpy.action', SimpleNamespace(ActionClient=lambda *a: client))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.convert', SimpleNamespace(message_to_ordereddict=lambda x: x))
    driver = object.__new__(m.RosDriver)
    driver.node, driver.active = None, []
    def interrupt(*args, **kw):
        raise KeyboardInterrupt('operator stop')
    def wait(future, timeout):
        if future == 'send':
            return handle
        raise TimeoutError('cancel result unavailable')
    driver.ros, driver.wait = SimpleNamespace(spin_once=interrupt), wait
    with pytest.raises(KeyboardInterrupt, match='operator stop'):
        driver.action(object, '/mock/action', object(), 10)


def test_repeated_stop_signals_cannot_interrupt_cleanup():
    import subprocess
    import sys
    script = f'''
import importlib.util, os, signal
spec = importlib.util.spec_from_file_location('runner', {str(SCRIPT)!r})
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
old = signal.getsignal(signal.SIGINT)
with m.defer_stop_signals():
    os.kill(os.getpid(), signal.SIGINT)
    os.kill(os.getpid(), signal.SIGTERM)
    print('cleanup completed')
assert signal.getsignal(signal.SIGINT) == old
'''
    result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert 'cleanup completed' in result.stdout
