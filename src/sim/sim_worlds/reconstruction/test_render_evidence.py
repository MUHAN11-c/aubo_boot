"""Reject mixed or stale render evidence before perception evaluation."""

import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_evidence import record_render, validate_render_set  # noqa: E402


def test_render_evidence_rejects_stale_and_mixed_views(tmp_path):
    """Accept intact evidence and reject three common stale-output cases."""
    manifest = {'source_blend_sha256': 'scene-a', 'lighting': {'preset': 'noon'},
                'render_settings': {'samples': 16}}
    for view in ('reference', 'detail'):
        for suffix in ('.png', '_Depth_0001.exr', '_IndexOB_0001.exr',
                       '_Position_0001.exr'):
            (tmp_path / (view + suffix)).write_bytes(b'original')
        record_render(tmp_path, view, manifest)
    validate_render_set(tmp_path, manifest, ('reference', 'detail'))
    mixed = copy.deepcopy(manifest)
    mixed['lighting']['preset'] = 'overcast'
    with pytest.raises(ValueError, match='context'):
        validate_render_set(tmp_path, mixed, ('reference',))
    with pytest.raises(ValueError, match='missing'):
        validate_render_set(tmp_path, manifest, ('orchard',))
    (tmp_path / 'detail.png').write_bytes(b'changed')
    with pytest.raises(ValueError, match='hash'):
        validate_render_set(tmp_path, manifest, ('detail',))
