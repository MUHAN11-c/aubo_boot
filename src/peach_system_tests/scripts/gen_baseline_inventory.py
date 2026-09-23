#!/usr/bin/env python3
"""生成重构基线快照 baseline_inventory.json（FINAL_PLAN §26 缺件补齐）。

快照内容：
  - parameters：全部 peach*/config yaml 参数叶值（file/key/value/provenance 占位）
  - interfaces：interface_manifest.yaml 现行接口名清单
  - functions：非测试 Python def 与 peach_arm/src C++ 外联定义的位置（file/line/name）
只锁 schema 不锁数值（数值随重构演进，schema 漂移即失败）。
用法：python3 gen_baseline_inventory.py [--out test/baseline_inventory.json]
"""
from __future__ import annotations

import argparse
import ast
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCHEMA_VERSION = 1


def collect_parameters() -> list:
    import yaml
    params = []
    for p in sorted((ROOT / 'src').glob('peach*/config/*.yaml')):
        if p.name in {'interface_manifest.yaml'}:
            continue
        try:
            data = yaml.safe_load(p.read_text())
        except Exception:
            continue
        if not isinstance(data, dict):
            continue

        def walk(x, k=''):
            if isinstance(x, dict):
                for a, b in x.items():
                    walk(b, f'{k}.{a}' if k else str(a))
            else:
                params.append({
                    'file': str(p.relative_to(ROOT)),
                    'key': k.split('ros__parameters.', 1)[-1],
                    'value': json.dumps(x, ensure_ascii=False)[:120],
                })
        walk(data)
    return params


def collect_interfaces() -> list:
    import yaml
    m = yaml.safe_load(
        (ROOT / 'src/peach_interfaces/config/interface_manifest.yaml').read_text())
    names = sorted(x['name'] for x in (m.get('interfaces') or []))
    return names


def collect_functions() -> list:
    functions = []
    for p in sorted((ROOT / 'src').glob('peach*/**/*.py')):
        if any(x in p.parts for x in ('test', 'build', 'install', 'log', '_archive')):
            continue
        if p.name == '__init__.py':
            continue
        try:
            tree = ast.parse(p.read_text())
        except Exception:
            continue
        for n in ast.walk(tree):
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
                functions.append({
                    'file': str(p.relative_to(ROOT)), 'line': n.lineno,
                    'name': n.name,
                })
    for p in sorted((ROOT / 'src/peach_arm').glob('src/*.cpp')):
        for no, line in enumerate(p.read_text().splitlines(), 1):
            m = re.match(r'^\s*[\w:<>~*& ,]+\b((?:\w+::)+\w+)\s*\(', line)
            if m and not m.group(1).startswith(
                ('std::', 'rclcpp::', 'Eigen::', 'tf2::')):
                functions.append({
                    'file': str(p.relative_to(ROOT)), 'line': no,
                    'name': m.group(1),
                })
    return functions


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--out', default=str(Path(__file__).resolve().parent.parent /
                             'test/baseline_inventory.json'))
    args = parser.parse_args()
    inventory = {
        'schema_version': SCHEMA_VERSION,
        'generated_at': __import__('datetime').datetime.now().isoformat(
            timespec='seconds'),
        'counts': {},
        'parameters': collect_parameters(),
        'interfaces': collect_interfaces(),
        'functions': collect_functions(),
    }
    inventory['counts'] = {
        'parameters': len(inventory['parameters']),
        'interfaces': len(inventory['interfaces']),
        'functions': len(inventory['functions']),
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(inventory, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f"baseline inventory → {out}")
    print(json.dumps(inventory['counts'], ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
