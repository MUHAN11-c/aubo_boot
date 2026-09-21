"""safe_component 目录段净化、ensure_within 包含性与 runs_root 归一."""
from pathlib import Path

import pytest

from peach_common.paths import ensure_within, runs_root, safe_component


def test_normal_ids_pass_through():
    assert safe_component('harvest_2026', 'x') == 'harvest_2026'
    assert safe_component('run:42', 'x') == 'run:42'


def test_traversal_and_separators_fall_back():
    for bad in ('..', '../etc', 'a/b', 'a\\b', '', '   ', '.', 'nul\x00'):
        assert safe_component(bad, 'harvest') == 'harvest', bad


def test_fallback_itself_invalid_degrades_to_unknown():
    assert safe_component('..', '../evil') == 'unknown'
    assert safe_component('..', '') == 'unknown'


def test_whitespace_only_stripped_then_fallback():
    assert safe_component('  \t ', 'run') == 'run'


# ---- runs_root 三场景对拍（W6-B 归一：与归一前两实现行为一致）----

def _expected_default_root() -> Path:
    """镜像实现的探测规则（找 src/peach_interfaces 标记 → cwd 兜底）."""
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'peach_interfaces').is_dir():
            return parent / 'runs'
    return Path.cwd() / 'runs'


def _clear_runs_env(monkeypatch):
    monkeypatch.delenv('AUBO_RUNS_DIR', raising=False)
    monkeypatch.delenv('AUBO_HARVEST_DATA_DIR', raising=False)


def test_runs_root_default_without_env(monkeypatch):
    """场景一：无 env——工作区标记探测（src/peach_interfaces）→ runs/."""
    _clear_runs_env(monkeypatch)
    assert runs_root() == _expected_default_root()


def test_runs_root_aubo_runs_dir_env(monkeypatch):
    """场景二：AUBO_RUNS_DIR 直取（且优先于 AUBO_HARVEST_DATA_DIR）."""
    monkeypatch.setenv('AUBO_RUNS_DIR', '/tmp/runs_from_aubo')
    monkeypatch.setenv('AUBO_HARVEST_DATA_DIR', '/tmp/runs_from_harvest')
    assert runs_root() == Path('/tmp/runs_from_aubo')


def test_runs_root_harvest_data_dir_env(monkeypatch):
    """场景三：仅 AUBO_HARVEST_DATA_DIR——同样直取."""
    _clear_runs_env(monkeypatch)
    monkeypatch.setenv('AUBO_HARVEST_DATA_DIR', '/tmp/runs_from_harvest')
    assert runs_root() == Path('/tmp/runs_from_harvest')


def test_runs_root_configured_absolute_wins(monkeypatch):
    _clear_runs_env(monkeypatch)
    assert runs_root('/tmp/configured_root') == Path('/tmp/configured_root')


def test_runs_root_configured_relative_falls_back(monkeypatch):
    """相对路径 configured 不生效（回默认），与归一前语义一致."""
    _clear_runs_env(monkeypatch)
    assert runs_root('relative/root') == _expected_default_root()


# ---- ensure_within 写入包含性（writer 边界兜底；符号链接解出）----

def test_ensure_within_accepts_inside_paths(tmp_path):
    base = tmp_path / 'session'
    base.mkdir()
    deep = base / 'frame_00' / 'meta.yaml'
    assert ensure_within(deep, base) == str(
        Path(__import__('os').path.realpath(deep)))


def test_ensure_within_rejects_escape(tmp_path):
    base = tmp_path / 'session'
    base.mkdir()
    outside = tmp_path / 'elsewhere' / 'x.yaml'
    with pytest.raises(ValueError):
        ensure_within(outside, base)
    with pytest.raises(ValueError):
        ensure_within(base / '..' / 'escape.yaml', base)


def test_ensure_within_resolves_symlink_escape(tmp_path):
    base = tmp_path / 'session'
    base.mkdir()
    link = base / 'link'
    link.symlink_to(tmp_path)
    with pytest.raises(ValueError):
        ensure_within(link / 'x.yaml', base)
