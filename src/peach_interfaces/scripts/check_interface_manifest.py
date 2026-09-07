#!/usr/bin/env python3
"""
双向核对 interface_manifest.yaml 与 peach_* 源码.

正向：manifest 每个 name 必须出现在源码字面量（绝对名或 ~/相对名）。
反向：源码中出现的绝对 /peach 话题字面量必须在 manifest（现行或预留）或
可视化豁免表内；否则说明新增了未登记的跨包话题（漂移）。
弱校验：现行接口的 consumers 须在对应节点源码树里出现该名字
（调度不含 observability；预留区不查消费者）。
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

# consumer 标签 → 源码相对 src/ 的文件或目录（调度节点不含监控）。
_CONSUMER_PATHS = {
    'peach_executor': [
        'peach_executor/peach_executor/executor_node.py',
        'peach_executor/peach_executor/batch.py',
        'peach_executor/peach_executor/select.py',
        'peach_executor/peach_executor/control.py',
        'peach_executor/peach_executor/summary.py',
        'peach_executor/peach_executor/ledger.py',
        'peach_executor/peach_executor/harvest_fsm.py',
        'peach_executor/config/executor_parameters.yaml',
        'peach_executor/config/peach_executor.yaml',
    ],
    'peach_observability': [
        'peach_executor/peach_executor/observability',
        'peach_executor/config/observability_parameters.yaml',
        'peach_executor/config/observability.yaml',
    ],
    'peach_scene_perception': [
        'peach_perception/peach_perception/scene_perception',
        'peach_perception/config/scene_perception_parameters.yaml',
        'peach_perception/config/scene_perception.yaml',
    ],
    'peach_target_reconstruction': [
        'peach_perception/peach_perception/target_reconstruction',
        'peach_perception/config/target_reconstruction_parameters.yaml',
        'peach_perception/config/target_reconstruction.yaml',
    ],
    'peach_manipulation': [
        'peach_manipulation/src',
        'peach_manipulation/include',
        'peach_manipulation/config',
    ],
    'peach_lifecycle_manager': [
        'peach_executor/peach_executor/lifecycle_manager.py',
        'peach_executor/config/lifecycle_manager.yaml',
        'peach_executor/config/lifecycle_manager_parameters.yaml',
    ],
}


def _workspace_src(script: Path) -> Path:
    """peach_interfaces/scripts → src/."""
    return script.resolve().parents[2]


# GPL 生成模块（gitignore）：default_value 会把未进清单的节点内服务名
# 收成 Python 字面量；反向扫描只看手写源码与 yaml。
_GENERATED_PARAM_MODULES = {
    'executor_parameters.py',
    'lifecycle_manager_parameters.py',
    'observability_parameters.py',
    'scene_perception_parameters.py',
    'target_reconstruction_parameters.py',
}


def _iter_sources(src_root: Path):
    for package in sorted(src_root.iterdir()):
        if not package.name.startswith('peach_'):
            continue
        for path in package.rglob('*'):
            if path.suffix not in {'.py', '.cpp', '.hpp', '.xml', '.yaml'}:
                continue
            if path.name in _GENERATED_PARAM_MODULES:
                continue
            if any(part in {'build', 'install', 'log', '__pycache__',
                            '.pytest_cache'} for part in path.parts):
                continue
            yield path


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='ignore')
    except OSError:
        return ''


def _consumer_blob(src_root: Path, consumer: str) -> str:
    """拼接某一 consumer 标签对应的源码文本."""
    pieces = []
    for relative in _CONSUMER_PATHS.get(consumer, ()):
        path = src_root / relative
        if path.is_file():
            pieces.append(_read(path))
            continue
        if path.is_dir():
            for child in path.rglob('*'):
                if child.name in _GENERATED_PARAM_MODULES:
                    continue
                if child.suffix in {'.py', '.cpp', '.hpp', '.yaml'}:
                    pieces.append(_read(child))
    return '\n'.join(pieces)


def _name_in_blob(name: str, blob: str) -> bool:
    """绝对名或 ~/相对名出现即算接线."""
    if name in blob:
        return True
    rel = name.rsplit('/', 1)[-1]
    return f"'~/{rel}'" in blob or f'"~/{rel}"' in blob


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
        blob += _read(path)
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
    # 弱校验：现行 consumers 须在对应节点源码出现该名字（自订阅跳过）
    consumer_miss = []
    for item in interfaces:
        name = item['name']
        producers = set(item.get('producers') or [])
        for consumer in item.get('consumers') or []:
            if consumer in producers:
                continue
            if consumer not in _CONSUMER_PATHS:
                consumer_miss.append(f'{name}: unknown consumer {consumer}')
                continue
            cblob = _consumer_blob(src_root, consumer)
            if not _name_in_blob(name, cblob):
                consumer_miss.append(f'{name} consumer {consumer}')
    if consumer_miss:
        print('interface manifest consumers not found in node sources:')
        for line in consumer_miss:
            print(f'  {line}')
        return 1
    active = len(interfaces)
    print(
        f'interface manifest ok ({active} active + {len(reserved)} reserved names)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
