"""supervisor/batch 纯核测试：S4 白名单 / S6 preferred 谓词 / S7 回读 / runs-root 对拍."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from peach_common.paths import runs_root
from peach_harvester.supervisor import batch
from peach_harvester.vision.common import runtime


# ---- S4：ledger 白名单（outcome_to_dict）----

def _outcome(target_id='t1', code=0):
    return SimpleNamespace(
        target_id=target_id, outcome=code, reason='ok',
        quality_score=0.5, elapsed=None)


def test_outcome_whitelist_includes_full_extras():
    """W6-B/S4：_cmd_full 写的 extra 键全集进账本（对齐 observability 期望）."""
    extra = {
        'failure_code': 'full_failed',
        'failure_code_n': 12,
        'completion_level': 4,
        'stage_names': ['prepare', 'sleeve'],
        'stage_durations': [0.1, 2.0],
        'build_view_count': 3,
        'build_status': 'READY',
        'build_duration_s': 1.5,
        'timeout_source': 'full_failed',
    }
    data = batch.outcome_to_dict(_outcome(), extra)
    for key in batch._EXTRA_KEYS:
        assert data.get(key) == extra[key], key


def test_outcome_whitelist_excludes_idl_doomed_bools():
    """cut/retreat/harvest_confirmed 三键不补——W7 起顶层镜像已删，证据单源 harvest/verification 块."""
    extra = {
        'cut_confirmed': True,
        'retreat_confirmed': True,
        'harvest_confirmed': True,
    }
    data = batch.outcome_to_dict(_outcome(), extra)
    assert 'cut_confirmed' not in data
    assert 'retreat_confirmed' not in data
    assert 'harvest_confirmed' not in data


# ---- S7：details 随账本回读（load_ledger）----

def test_load_ledger_returns_per_outcome_details(monkeypatch, tmp_path):
    """load_ledger 按 _EXTRA_KEYS 回填 details；未落键不出现."""
    monkeypatch.setattr(
        batch, 'dict_to_outcome',
        lambda item: SimpleNamespace(target_id=item.get('target_id', '')))
    path = tmp_path / 'ledger.json'
    path.write_text(
        '{'
        '"claimed": ["t1"],'
        '"outcomes": [{'
        '"target_id": "t1", "outcome": 1, "reason": "x",'
        '"failure_code": "observe_failed", "completion_level": 2,'
        '"failure_code_n": 7, "cut_confirmed": true}]}',
        encoding='utf-8')
    claimed, outcomes, details = batch.load_ledger(path)
    assert claimed == {'t1'}
    assert len(outcomes) == 1
    assert details == [{
        'failure_code': 'observe_failed',
        'completion_level': 2,
        'failure_code_n': 7,
    }]


def test_load_ledger_missing_file_yields_empty(tmp_path):
    claimed, outcomes, details = batch.load_ledger(
        tmp_path / 'nope.json')
    assert claimed == set() and outcomes == [] and details == []


# ---- S6：preferred 也过资格谓词（next_target）----

def _item(tid, confirmed=True, flags=(), strategy='bag_v1',
          depth=0.8, prio=1, w=10, h=10):
    return SimpleNamespace(
        target_id=tid,
        confirmed=confirmed,
        diagnostic_flags=list(flags),
        candidate=SimpleNamespace(strategy_id=strategy, entry_pose=None),
        candidate_2d=SimpleNamespace(bbox_w=w, bbox_h=h),
        priority=prio,
        camera_distance_m=depth)


def _observations(items, locked=True):
    return SimpleNamespace(target_set_locked=locked, observations=items)


def test_preferred_eligible_passes_through():
    obs = _observations([_item('t1')])
    target_id, filtered = batch.next_target(
        obs, claimed=set(), preferred=['t1'])
    assert target_id == 't1'
    assert filtered == []


def test_preferred_unconfirmed_falls_to_regular_path():
    """S6：未确认目标不可被点名选中；有合格候选则走常规路径."""
    obs = _observations([_item('t1', confirmed=False), _item('t2')])
    target_id, filtered = batch.next_target(
        obs, claimed=set(), preferred=['t1'])
    assert target_id == 't2'


def test_preferred_bare_fruit_and_edge_not_selectable():
    obs = _observations([
        _item('bare', strategy='fruit_v1'),
        _item('edge', flags=['bbox_edge']),
    ])
    target_id, _filtered = batch.next_target(
        obs, claimed=set(), preferred=['bare', 'edge'])
    assert target_id == ''


def test_preferred_claimed_falls_to_regular_path():
    obs = _observations([_item('t1'), _item('t2')])
    target_id, _filtered = batch.next_target(
        obs, claimed={'t1'}, preferred=['t1'])
    assert target_id == 't2'


def test_preferred_without_lock_falls_to_regular_path():
    """锁定集未闭时 preferred 直通同样被拒（谓词含锁定集门）."""
    obs = _observations([_item('t1')], locked=False)
    target_id, _filtered = batch.next_target(
        obs, claimed=set(), preferred=['t1'])
    assert target_id == ''


# ---- W6-B：runs-root 单源对拍（batch / runtime / peach_common 三方一致）----

def _clear_env(monkeypatch):
    monkeypatch.delenv('AUBO_RUNS_DIR', raising=False)
    monkeypatch.delenv('AUBO_HARVEST_DATA_DIR', raising=False)


def test_runs_root_delegation_scenario_default(monkeypatch):
    """场景一：无 env——两旧实现委托 peach_common 后结果一致（工作区 runs）."""
    _clear_env(monkeypatch)
    assert batch.default_runs_root() == runs_root()
    assert runtime.default_runs_root() == runs_root()
    assert batch.resolve_runs_root('') == runs_root('')


def test_runs_root_delegation_scenario_aubo_runs_dir(monkeypatch):
    """场景二：AUBO_RUNS_DIR."""
    monkeypatch.setenv('AUBO_RUNS_DIR', '/tmp/aubo_runs_delegation')
    assert batch.default_runs_root() == Path('/tmp/aubo_runs_delegation')
    assert runtime.resolve_runs_root('') == Path('/tmp/aubo_runs_delegation')


def test_runs_root_delegation_scenario_harvest_data_dir(monkeypatch):
    """场景三：AUBO_HARVEST_DATA_DIR."""
    _clear_env(monkeypatch)
    monkeypatch.setenv('AUBO_HARVEST_DATA_DIR', '/tmp/harvest_data_delegation')
    assert batch.default_runs_root() == Path('/tmp/harvest_data_delegation')
    assert runtime.default_runs_root() == Path('/tmp/harvest_data_delegation')
