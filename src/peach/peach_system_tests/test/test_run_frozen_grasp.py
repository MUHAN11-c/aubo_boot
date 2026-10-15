"""Wrapper process ownership regressions; no ROS initialization."""
import importlib.util
from pathlib import Path
import sys

import pytest
from types import SimpleNamespace


SCRIPTS = Path(__file__).parents[1] / 'scripts'


def load():
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location('run_frozen_test', SCRIPTS / 'run_frozen_grasp.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.pop(0)


def test_default_headless_and_existing_cli_preserved():
    m = load()
    args = m.create_parser().parse_args([])
    assert (args.index, args.hz, args.timeout, args.ready_timeout, args.rviz) == (70, 2., 180., 120., False)
    assert m.create_parser().parse_args(['--rviz']).rviz is True


@pytest.mark.parametrize("skip_launch", [True, False])
@pytest.mark.parametrize("cleanup_failed", [False, True])
def test_skip_launch_only_cleans_owned_driver(monkeypatch, tmp_path, skip_launch, cleanup_failed):
    m = load()
    (tmp_path / 'metadata.yaml').write_text('read-only fixture')
    started, cleaned, app_cleanup = [], [], []
    class Owner:
        def __init__(self, *args, **kwargs):
            self.processes = []
        def start(self, command, log=None, env=None):
            started.append(command)
            process = SimpleNamespace(pid=111, wait=lambda **kw: 0,
                                      poll=lambda: None if 'launch' in command else 0)
            self.processes.append(process)
            return process
        def cleanup(self):
            cleaned.extend(p.pid for p in self.processes)
            return []
    monkeypatch.setattr(m, 'OwnedProcesses', Owner)
    monkeypatch.setattr(m, '_pgrep', lambda *args: [] if args else [(999, 'external_ros')])
    monkeypatch.setattr(m, 'assert_mock_stack', lambda *args: None)
    monkeypatch.setattr(m, 'cleanup_request', lambda *args: app_cleanup.append(args) or (['disable failed'] if cleanup_failed else []))
    monkeypatch.setattr(m.time, 'sleep', lambda *args: None)
    options = ['--bag', str(tmp_path), '--request-id', 'own'] + (['--skip-launch'] if skip_launch else [])
    assert m.main(options) == (1 if cleanup_failed else 0)
    assert len(started) == (1 if skip_launch else 2)
    assert 'drive_frozen_grasp.py' in started[-1]
    if not skip_launch:
        assert 'rviz:=false' in started[0]
    assert cleaned == ([111] if skip_launch else [111, 111])
    assert 999 not in cleaned


def test_foreign_request_never_receives_cancel_or_disable(monkeypatch):
    m = load()
    monkeypatch.setattr(m, 'snapshot', lambda: {
        'startup': {'facts': {'hardware_mode': 'mock', 'ros_domain_id': '81'}},
        'task_executor': {'state': {'run_id': 'foreign', 'action_active': True}}})
    def forbidden():
        raise AssertionError('must not create ROS driver for foreign request')
    monkeypatch.setattr(m, 'RosDriver', forbidden)
    assert m.cleanup_request({'ROS_DOMAIN_ID': '81'}, 'owned') == []


def test_frozen_fixture_preserves_product_cut_travel_range():
    import ast
    tree = ast.parse((SCRIPTS / 'drive_frozen_grasp.py').read_text())
    overrides = [node.args[0].value for node in ast.walk(tree)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                 and node.func.attr == 'set_double' and node.args
                 and isinstance(node.args[0], ast.Constant)]
    assert 'grasp.maximum_travel_m' not in overrides
    assert 'grasp.minimum_travel_m' not in overrides
