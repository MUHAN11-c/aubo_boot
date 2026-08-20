#!/usr/bin/env python3
"""核对 interface_manifest.yaml 中的名称是否出现在 peach_* 源码字面量里."""
from __future__ import annotations

import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None


def _workspace_src(script: Path) -> Path:
    """peach_interfaces/scripts → src/."""
    return script.resolve().parents[2]


def _iter_sources(src_root: Path):
    for package in src_root.iterdir():
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
    names = [item['name'] for item in document.get('interfaces', [])]
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
    print(f'interface manifest ok ({len(names)} names)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
