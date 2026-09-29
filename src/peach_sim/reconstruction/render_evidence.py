"""Per-view provenance for RGB and Blender ground-truth passes."""

import copy
import hashlib
import json


def _context(manifest):
    return {key: manifest[key] for key in
            ('source_blend_sha256', 'lighting', 'render_settings')}


def record_render(out, view, manifest):
    """Record a complete render only after Blender successfully returns."""
    files = [view + suffix for suffix in
             ('.png', '_Depth_0001.exr', '_IndexOB_0001.exr', '_Position_0001.exr')]
    record = copy.deepcopy(_context(manifest))
    record['files'] = {name: hashlib.sha256((out / name).read_bytes()).hexdigest()
                       for name in files}
    manifest.setdefault('rendered_views', {})[view] = record
    (out / 'scene_manifest.json').write_text(json.dumps(manifest, indent=2))


def validate_render_set(out, manifest, views):
    """Fail closed for missing records, mixed context or altered images."""
    for view in views:
        record = manifest.get('rendered_views', {}).get(view)
        if record is None:
            raise ValueError(f'{view}: missing render provenance; render again')
        if _context(record) != _context(manifest):
            raise ValueError(f'{view}: render context differs; render all views')
        expected = {view + suffix for suffix in
                    ('.png', '_Depth_0001.exr', '_IndexOB_0001.exr',
                     '_Position_0001.exr')}
        if set(record.get('files', {})) != expected:
            raise ValueError(f'{view}: missing pass hashes')
        for name, digest in record['files'].items():
            path = out / name
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise ValueError(f'{name}: render hash differs; render again')
