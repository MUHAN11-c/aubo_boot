"""评审拼板：从矩阵帧中拼两张对比图供人工查看效果.

- lighting board：同一停靠视角在全部光照预设下的对比（2 列网格）。
  光照列表从 trajectory.json 单源读取（新增预设自动入板，2026-09-29
  起含 backlit 逆光困难组）。
- occlusion board：none / light / heavy / reference 各取一 primary 近视
  （noon 渲染）。
输出 output/matrix_lighting_board.jpg 与 output/matrix_occlusion_board.jpg。
"""

import json
from pathlib import Path

from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
OUT = HERE / 'output'
THUMB = (640, 360)


def _tile(path, label):
    img = Image.open(path).convert('RGB').resize(THUMB)
    tile = Image.new('RGB', (THUMB[0], THUMB[1] + 26), (24, 24, 24))
    tile.paste(img, (0, 26))
    ImageDraw.Draw(tile).text((8, 6), label, fill=(235, 235, 235))
    return tile


def main():
    traj = json.loads((OUT / 'matrix/trajectory.json').read_text())
    registry = {int(k): v for k, v in traj['target_registry'].items()}
    views = traj['views']
    lightings = sorted(traj['lighting_applied'])

    # ---- lighting board: prefer a stop view (sees the whole row) ----
    stop_id = next(v['view_id'] for v in views if v['kind'] == 'alley_stop'
                   and v['stop_id'] == 2)
    cols = 2
    rows = (len(lightings) + cols - 1) // cols
    board = Image.new('RGB',
                      (THUMB[0] * cols + 30, (THUMB[1] + 26) * rows + 40),
                      (16, 16, 16))
    for i, lname in enumerate(lightings):
        path = OUT / 'matrix' / lname / stop_id / 'rgb.png'
        if not path.exists():
            continue
        board.paste(_tile(path, f'{stop_id}  lighting={lname}'),
                    (10 + (i % cols) * (THUMB[0] + 10),
                     10 + (i // cols) * (THUMB[1] + 26 + 10)))
    board.save(OUT / 'matrix_lighting_board.jpg', quality=92)

    # ---- occlusion board: one primary view per level ----
    pick = {}
    for v in views:
        if v['kind'] != 'primary':
            continue
        tid = v['target_ids'][0] if v.get('target_ids') else None
        level = registry[tid]['occlusion'] if tid in registry else None
        if level not in pick:
            pick[level] = v['view_id']
    order = [lv for lv in ('none', 'light', 'heavy', 'reference') if lv in pick]
    board2 = Image.new(
        'RGB', (THUMB[0] * len(order) + 10 * (len(order) + 1),
                THUMB[1] + 26 + 20), (16, 16, 16))
    for i, lv in enumerate(order):
        path = OUT / 'matrix/noon' / pick[lv] / 'rgb.png'
        board2.paste(_tile(path, f'{pick[lv]}  level={lv} (noon)'),
                     (10 + i * (THUMB[0] + 10), 10))
    board2.save(OUT / 'matrix_occlusion_board.jpg', quality=92)
    print('boards:', OUT / 'matrix_lighting_board.jpg',
          OUT / 'matrix_occlusion_board.jpg', flush=True)
    print('picked primary views:', pick, flush=True)


if __name__ == '__main__':
    main()
