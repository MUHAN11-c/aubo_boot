import json
from pathlib import Path

from peach2_observability.ledger import read_ledger, sanitize_request_id


def test_sanitize_request_id_rejects_traversal():
    assert sanitize_request_id('../evil') is None
    assert sanitize_request_id('ok_id_2026') == 'ok_id_2026'


def test_read_ledger_parses_outcomes(tmp_path: Path):
    req = 'field_test_20260930'
    root = tmp_path / 'runs'
    ledger_dir = root / req
    ledger_dir.mkdir(parents=True)
    document = {
        'claimed': ['t1'],
        'outcomes': [{
            'target_id': 't1',
            'outcome': 0,
            'stage_names': ['OBSERVE'],
            'stage_durations': [1.5],
            'elapsed_s': 2.0,
        }],
    }
    (ledger_dir / 'ledger.json').write_text(json.dumps(document), encoding='utf-8')
    payload = read_ledger(root, req)
    assert payload['error'] is None
    assert payload['rows'][0]['outcome_name'] == 'SUCCEEDED'
    assert payload['totals']['attempted'] == 1


def test_read_ledger_invalid_id(tmp_path: Path):
    payload = read_ledger(tmp_path, '../../etc/passwd')
    assert 'error' in payload
