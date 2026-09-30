"""Ensure observability never registers service/action clients (read-only contract)."""
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1] / 'peach2_observability'
FORBIDDEN = ('create_client', 'ActionClient')


def test_source_has_no_service_or_action_clients():
    hits = []
    for path in PKG_ROOT.rglob('*.py'):
        text = path.read_text(encoding='utf-8')
        for token in FORBIDDEN:
            if token in text:
                hits.append(f'{path.name}: {token}')
    assert hits == [], 'forbidden ROS client API in observability sources:\n' + '\n'.join(hits)
