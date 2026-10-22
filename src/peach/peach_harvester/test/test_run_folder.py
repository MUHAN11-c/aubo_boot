"""一次开栈的终局要落进正在录的 session 目录."""
import json
import os
import time
from pathlib import Path

from peach_harvester.supervisor.run_folder import (
    archive_ledger_into,
    live_session_dir,
)


def _ledger(root: Path, request: str, target: str = 'target_0') -> Path:
    path = root / request / 'ledger.json'
    path.parent.mkdir()
    path.write_text(json.dumps({
        'outcomes': [{
            'target_id': target,
            'outcome': 0,
            'elapsed_s': 22,
            'completion_level': 6,
            'stage_names': ['approach_insert', 'retreat'],
            'stage_durations': [12, 10],
        }],
    }), encoding='utf-8')
    return path


def test_archive_lands_in_the_open_session(tmp_path: Path):
    """终局写进当前会话，同一颗重复归档不另开文件."""
    session = tmp_path / 'session_20261022_150712'
    bag = session / 'bag'
    bag.mkdir(parents=True)
    now = time.time()
    marker = bag / 'bag_0.mcap'
    marker.write_bytes(b'x')
    os.utime(marker, (now, now))
    ledger = _ledger(tmp_path, 'grasp_20261022_150934_001')
    written = archive_ledger_into(session, ledger, now=now)
    assert len(written) == 1
    assert written[0].parent == session / 'cycles'
    payload = json.loads(written[0].read_text(encoding='utf-8'))
    assert payload['request_id'] == 'grasp_20261022_150934_001'
    assert payload['target_id'] == 'target_0'
    assert payload['end_unix'] == round(now, 3)
    assert payload['start_unix'] == round(now - 22, 3)
    assert '2026-' in payload['wall_start']
    again = archive_ledger_into(session, ledger, now=now)
    assert again == written
    assert list((session / 'cycles').glob('*.json')) == written
    assert live_session_dir(tmp_path, now=now) == session


def test_stale_session_is_not_the_archive_target(tmp_path: Path):
    """袋文件已经不再更新的会话不接收新周期."""
    bag = tmp_path / 'session_20261022_090000' / 'bag'
    bag.mkdir(parents=True)
    marker = bag / 'bag_0.mcap'
    marker.write_bytes(b'x')
    old = time.time() - 100
    os.utime(marker, (old, old))
    os.utime(bag, (old, old))
    assert live_session_dir(tmp_path) is None
