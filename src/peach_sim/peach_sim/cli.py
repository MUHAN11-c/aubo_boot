"""
``generate_orchard`` 入口：读场景参数 → 写世界 SDF 与目标清单.

文件 I/O 全在这里；布局与几何计算在纯核 ``peach_sim.scene``。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

from .params import check_params, params_from_dict
from .scene import render_scene


def default_share() -> Path:
    """安装后取 share/peach_sim；源码树直接跑则取包根."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('peach_sim'))
    except Exception:  # 未安装/未 source：退回源码树
        return Path(__file__).resolve().parents[1]


def main(argv: list[str] | None = None) -> int:
    share = default_share()
    parser = argparse.ArgumentParser(
        prog='generate_orchard',
        description='由 config/orchard.yaml 确定性生成果园世界 SDF 与目标清单')
    parser.add_argument(
        '--params', type=Path, default=share / 'config' / 'orchard.yaml',
        help='场景参数 yaml（默认 <pkg>/config/orchard.yaml）')
    parser.add_argument(
        '--out-dir', type=Path, default=share / 'worlds',
        help='输出目录（默认 <pkg>/worlds）')
    parser.add_argument(
        '--check', action='store_true',
        help='只校验参数与布局，不写文件')
    args = parser.parse_args(argv)

    raw = yaml.safe_load(args.params.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw if isinstance(raw, dict) else {})
    problems += check_params(params)
    if problems:
        print(f'{args.params}: 场景参数不合法', file=sys.stderr)
        for problem in problems:
            print(f'- {problem}', file=sys.stderr)
        return 2

    scene = render_scene(params)
    counts = scene.manifest['counts']
    print(
        f'布局：{counts["rows"]} 行 / {counts["trees"]} 株 / '
        f'{counts["targets"]} 颗套袋桃（作业位可达 {counts["reachable"]}，'
        f'作业区树 {counts["work_zone_trees"]} 株）')
    if args.check:
        print('校验通过（--check，未写文件）')
        return 0

    args.out_dir.mkdir(parents=True, exist_ok=True)
    world_path = args.out_dir / 'peach_orchard.sdf'
    manifest_path = args.out_dir / 'peach_orchard.manifest.yaml'
    world_path.write_text(scene.world_sdf, encoding='utf-8')
    manifest_path.write_text(
        yaml.safe_dump(
            scene.manifest, allow_unicode=True, sort_keys=False,
            default_flow_style=False),
        encoding='utf-8')
    print(f'已写 {world_path}')
    print(f'已写 {manifest_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
