#!/usr/bin/env python3
"""
双向核对 interface_manifest.yaml 与 peach_* 源码.

正向：manifest 每个 name 必须出现在源码字面量（绝对名或 ~/相对名）。
反向：源码中出现的绝对 /peach 话题字面量必须在 manifest（现行或预留）或
可视化豁免表内；否则说明新增了未登记的跨包话题（漂移）。
"""
from __future__ import annotations

from pathlib import Path
import re
import sys

try:
    import yaml
except ImportError:
    yaml = None

# 可视化/调试话题：仅供 RViz/Web 调试，按 architecture §3 不进清单
EXEMPT_VISUALIZATION = {
    '/peach/perception/axis',
    '/peach/perception/debug_image_raw',
    '/peach/perception/detections',
    '/peach/perception/markers',
    '/peach/perception/masks',
    '/peach/perception/single_cloud',
    '/peach/reconstruction/local_cloud',
    '/peach_manipulation_node/planned_views',
}

_LITERAL_RE = re.compile(r'[\'"](/peach[A-Za-z0-9_/]*)[\'"]')


def _workspace_src(script: Path) -> Path:
    """peach_interfaces/scripts → src/."""
    return script.resolve().parents[2]


def _iter_sources(src_root: Path):
    for package in sorted(src_root.iterdir()):
        if not package.name.startswith('peach_'):
            continue
        for path in package.rglob('*'):
            if path.suffix not in {'.py', '.cpp', '.hpp', '.xml', '.yaml'}:
                continue
            if any(part in {'build', 'install', 'log'} for part in path.parts):
                continue
            yield path


def main() -> int:
    script = Path(__file__)
    manifest_path = script.resolve().parents[1] / 'config' / 'interface_manifest.yaml'
    if yaml is None:
        print('PyYAML missing; skip interface manifest check', file=sys.stderr)
        return 0
    document = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    interfaces = document.get('interfaces', [])
    reserved = document.get('reserved_interfaces', [])
    names = [item['name'] for item in interfaces + reserved]
    known = set(names)
    blob = ''
    src_root = _workspace_src(script)
    for path in _iter_sources(src_root):
        try:
            blob += path.read_text(encoding='utf-8', errors='ignore')
        except OSError:
            continue
    missing = []
    for name in names:
        token = name
        rel = name.rsplit('/', 1)[-1]
        if token not in blob and f"'~/{rel}'" not in blob and f'"~/{rel}"' not in blob:
            missing.append(name)
    if missing:
        print('interface manifest names not found in peach_* sources:')
        for name in missing:
            print(f'  {name}')
        return 1
    # 反向：源码绝对 /peach 字面量必须可归入 manifest / 豁免表 / 是清单名的前缀
    untracked = []
    for literal in sorted(set(_LITERAL_RE.findall(blob))):
        if literal in known or literal in EXEMPT_VISUALIZATION:
            continue
        if any(name.startswith(literal) for name in known):
            continue
        untracked.append(literal)
    if untracked:
        print('source /peach topic literals not tracked in interface manifest:')
        for literal in untracked:
            print(f'  {literal}')
        return 1
    active = len(interfaces)
    print(
        f'interface manifest ok ({active} active + {len(reserved)} reserved names)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
