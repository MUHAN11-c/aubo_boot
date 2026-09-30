from pathlib import Path

from peach2_observability.params import load_params, resolve_runs_root
import pytest


def test_shipped_config_loads():
    config = Path(__file__).resolve().parents[1] / 'config' / 'observability.yaml'
    params = load_params(config)
    assert params.host == '127.0.0.1'
    assert params.port == 8091
    assert params.session_bag.enabled is False


def test_resolve_runs_root_explicit(tmp_path):
    assert resolve_runs_root(str(tmp_path)) == tmp_path.resolve()


def test_missing_session_bag_key_rejected(tmp_path, monkeypatch):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('host: 127.0.0.1\nport: 8091\nruns_root: ""\n', encoding='utf-8')
    with pytest.raises(ValueError, match='session_bag'):
        load_params(bad)
