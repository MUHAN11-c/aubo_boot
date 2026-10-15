"""
重建果园 GT 纯核（零 ROS）：scene_manifest → 停靠位 / 导航地图 / 作业位.

数据源 ``reconstruction/output/field_no_grass_20261009/scene_manifest.json``
（v11 建模轮，24 树 / 395 袋全 3D GT）。布局实测：8 行（y≈1.5..22.5，行距
3 m）× 3 列（x≈-5.5/-0.5/4.5，列距 ~5 m）。停靠位取作业道侧树干外
``row_gap``（默认 0.9 m，臂包络/深度窗标定位；列距 5 m 侧向余量充足）。
地面/网格仅用树干致命盘 + 外沿致命环（同 nav_map 口径，无棚架柱数据）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
import sys

from .nav_map import (
    BORDER_M,
    FREE,
    map_yaml_text,
    NavMap,
    OCCUPIED,
    RESOLUTION,
    write_pgm,
)

_TREE_RE = re.compile(r'Tree(\d+)/')


@dataclass(frozen=True)
class FieldStation:
    """重建果园一个停靠位（世界系 = GLB Z-up 米）."""

    tree_id: str
    index: int
    x: float
    y: float
    yaw: float
    bag_count: int


@dataclass
class FieldGT:
    """重建场景结构化 GT（树干位置 / 每树袋颈位置）."""

    trunks: dict[int, tuple[float, float]] = field(default_factory=dict)
    bag_necks: dict[int, list[tuple[float, float, float]]] = field(
        default_factory=dict)
    bag_total: int = 0


def parse_field_manifest(path: Path) -> FieldGT:
    """解析 scene_manifest：scaffold 根点聚树干，bag neck 进每树列表."""
    data = json.loads(path.read_text(encoding='utf-8'))
    gt = FieldGT()
    roots: dict[int, list[tuple[float, float]]] = {}
    for conn in data.get('connections', []):
        match = _TREE_RE.match(conn['name'])
        if match and 'scaffold' in conn['name']:
            roots.setdefault(int(match.group(1)), []).append(
                (conn['root'][0], conn['root'][1]))
    for tree, pts in roots.items():
        gt.trunks[tree] = (sum(p[0] for p in pts) / len(pts),
                           sum(p[1] for p in pts) / len(pts))
    for target in data.get('targets', []):
        match = _TREE_RE.match(target['name'])
        if not match:
            continue
        neck = target['neck']
        gt.bag_necks.setdefault(int(match.group(1)), []).append(
            (neck[0], neck[1], neck[2]))
        gt.bag_total += 1
    return gt


def _rows(trunks: dict[int, tuple[float, float]],
          cluster_sep: float = 1.0) -> list[list[int]]:
    """按 y 聚行（行沿 X；返回每行的树号列表，行内按 x 排序）."""
    order = sorted(trunks, key=lambda t: trunks[t][1])
    rows: list[list[int]] = []
    prev_y = None
    for tree in order:
        if prev_y is None or trunks[tree][1] - prev_y > cluster_sep:
            rows.append([])
        rows[-1].append(tree)
        prev_y = trunks[tree][1]
    for row in rows:
        row.sort(key=lambda t: trunks[t][0])
    return rows


def field_stations(gt: FieldGT, row_gap: float = 0.9,
                   side: int = 1) -> list[FieldStation]:
    """
    每树一个停靠位（作业道侧 row_gap，蛇形行序，车头沿行进方向）.

    side=+1 站树 +x 侧、-1 站 -x 侧；行末不插换行航点（由 conductor 的
    Nav2 全局规划走 headland 自由区，重建世界无棚架端柱约束）。
    """
    stations: list[FieldStation] = []
    for row_number, row in enumerate(_rows(gt.trunks)):
        ascending = row_number % 2 == 0
        trees = row if ascending else list(reversed(row))
        yaw = 0.0 if ascending else math.pi
        for index, tree in enumerate(trees):
            trunk_x, trunk_y = gt.trunks[tree]
            stations.append(FieldStation(
                tree_id=f'field_tree_{tree}', index=index,
                x=round(trunk_x + side * row_gap, 3),
                y=round(trunk_y, 3), yaw=round(yaw, 6),
                bag_count=len(gt.bag_necks.get(tree, []))))
    return stations


def build_field_nav_map(gt: FieldGT, margin: float = 4.0,
                        trunk_radius: float = 0.25) -> NavMap:
    """
    树干致命盘 + 外沿致命环栅格（NavMap 口径：行 0 = 最高 y）.

    margin = 树包络外的地面余量（行驶 headland）；origin = 左下角世界坐标。
    """
    xs = [p[0] for p in gt.trunks.values()]
    ys = [p[1] for p in gt.trunks.values()]
    x0, y0 = min(xs) - margin, min(ys) - margin
    x1, y1 = max(xs) + margin, max(ys) + margin
    width = int(math.ceil((x1 - x0) / RESOLUTION))
    height = int(math.ceil((y1 - y0) / RESOLUTION))
    grid = [[FREE] * width for _ in range(height)]

    def stamp(cx: float, cy: float, radius: float, value: int) -> None:
        col0 = int(math.floor((cx - radius - x0) / RESOLUTION))
        col1 = int(math.ceil((cx + radius - x0) / RESOLUTION))
        row_b0 = int(math.floor((cy - radius - y0) / RESOLUTION))
        row_b1 = int(math.ceil((cy + radius - y0) / RESOLUTION))
        for row_b in range(max(0, row_b0), min(height, row_b1)):
            row = height - 1 - row_b  # PGM 顶行 = 最高 y
            for col in range(max(0, col0), min(width, col1)):
                wx = x0 + (col + 0.5) * RESOLUTION
                wy = y0 + (row_b + 0.5) * RESOLUTION
                if (wx - cx) ** 2 + (wy - cy) ** 2 <= radius ** 2:
                    grid[row][col] = value

    for trunk_x, trunk_y in gt.trunks.values():
        stamp(trunk_x, trunk_y, trunk_radius, OCCUPIED)
    border = max(1, int(BORDER_M / RESOLUTION))
    for row in range(height):
        if row < border or row >= height - border:
            grid[row] = [OCCUPIED] * width
        else:
            for col in list(range(width))[:border] + \
                    list(range(width))[-border:]:
                grid[row][col] = OCCUPIED
    return NavMap(width=width, height=height, resolution=RESOLUTION,
                  origin=(x0, y0), cells=tuple(tuple(r) for r in grid))


def main(argv: list[str] | None = None) -> int:
    """``generate_field`` CLI：GT → maps/field_map.{pgm,yaml} + stations yaml."""
    import argparse

    parser = argparse.ArgumentParser(
        prog='generate_field',
        description='由重建 scene_manifest 生成 field 世界的导航地图与停靠位')
    parser.add_argument(
        '--manifest', type=Path, required=True,
        help='reconstruction scene_manifest.json')
    parser.add_argument('--out-dir', type=Path, default=Path('maps'))
    parser.add_argument('--row-gap', type=float, default=0.9)
    parser.add_argument('--side', type=int, default=1, choices=(1, -1))
    args = parser.parse_args(argv)

    gt = parse_field_manifest(args.manifest)
    if not gt.trunks:
        print(f'{args.manifest}: 未解析到树 GT', file=sys.stderr)
        return 2
    stations = field_stations(gt, row_gap=args.row_gap, side=args.side)
    nav_map = build_field_nav_map(gt)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    pgm = args.out_dir / 'field_map.pgm'
    write_pgm(nav_map, pgm)
    yaml_path = args.out_dir / 'field_map.yaml'
    yaml_path.write_text(map_yaml_text(nav_map, image_name=pgm.name), 'utf-8')

    stations_path = args.out_dir / 'field_stations.yaml'
    fruitful = [s for s in stations if s.bag_count > 0]
    first = fruitful[0] if fruitful else stations[0]
    # 走廊起步位：首行相邻两树干中点（不是首站——站位是作业位，距树仅
    # row_gap，机器人出生即压在树干膨胀圈上，MPPI 无法起步
    # "Failed to make progress"，2026-10-15 实测）。列中点距两侧树干
    # 各 ≈ 列距/2（≈2.5m），代价图干净。
    first_row = _rows(gt.trunks)[0]
    row_xs = sorted(gt.trunks[t][0] for t in first_row)
    if len(row_xs) >= 2:
        spawn_x = round((row_xs[0] + row_xs[1]) / 2.0, 3)
    else:
        spawn_x = round(row_xs[0] + 2.5, 3)
    lines = [
        '# 重建果园停靠位（generate_field 产物；世界系 = GLB Z-up 米）',
        f'manifest: {args.manifest}',
        f'bag_total: {gt.bag_total}',
        'spawn:  # AMCL 初始位姿 = 首行走廊中点（干净起步；站位是作业位贴树，'
        '不作 spawn）',
        f'  x: {spawn_x}',
        f'  y: {first.y}',
        '  yaw: 0.0',
        'stations:',
    ]
    for s in stations:
        lines.append(
            f'  - {{tree_id: {s.tree_id}, index: {s.index}, x: {s.x}, '
            f'y: {s.y}, yaw: {s.yaw}, bag_count: {s.bag_count}}}')
    stations_path.write_text('\n'.join(lines) + '\n', 'utf-8')
    print(f'已写 {pgm} / {yaml_path} / {stations_path}')
    print(f'布局：{len(gt.trunks)} 树 / {gt.bag_total} 袋 / '
          f'{len(stations)} 站（有袋 {len(fruitful)}）')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
