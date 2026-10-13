"""外观对照板：真实数据集抽样 vs 本管线渲染，供人工观感 QA.

上行 = PeachDataSet/Peach_bag 确定性抽样 4 帧（真实参考，只做对照不
做贴图）；下行 = 本管线代表渲染（reference/detail/orchard + 同版整园
总览）。输出 output/appearance_board.jpg。定性工具，不产生门。
"""

from pathlib import Path
import random
import json

from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
DATASET = Path('/home/mu/Downloads/PeachDataSet/Peach_bag/RGB')
REAL_IDS = ('0120', '0800', '1200', '1600')
RENDER_ROWS = [
    ('reference.png', 'reference (dataset-1200 match view)'),
    ('detail.png', 'detail oblique'),
    ('orchard.png', 'orchard overview'),
]
THUMB = (640, 360)


def _load(path, label):
    img = Image.open(path).convert('RGB').resize(THUMB)
    tile = Image.new('RGB', (THUMB[0], THUMB[1] + 26), (24, 24, 24))
    tile.paste(img, (0, 26))
    ImageDraw.Draw(tile).text((8, 6), label, fill=(230, 230, 230))
    return tile


def main():
    rng = random.Random(240928)
    real_paths = []
    names = sorted(p.name for p in DATASET.glob('*.png'))
    real_names = [n for n in names if n[:-4] in REAL_IDS]
    while len(real_names) < 4:
        real_names.append(rng.choice(names))
    real_paths = [DATASET / n for n in real_names[:4]]

    rows = [[(p, f'real Peach_bag {p.stem}') for p in real_paths]]
    render_row = []
    for filename, label in RENDER_ROWS:
        path = HERE / 'output' / filename
        if path.exists():
            render_row.append((path, f'render {label}'))
    # Never silently mix a previous matrix/field geometry into this review.
    scene = json.loads((HERE / 'output/scene_manifest.json').read_text())
    field_dir = HERE / 'output/field_anchor'
    field_manifest = field_dir / 'scene_manifest.json'
    if field_manifest.exists():
        field = json.loads(field_manifest.read_text())
        if scene.get('source_sha256') and scene['source_sha256'] == field.get('source_sha256'):
            render_row.append((field_dir / 'orchard.png', 'render full orchard (same revision)'))
    rows.append(render_row[:4])

    width = THUMB[0] * 4 + 50
    height = sum(THUMB[1] + 26 + 12 for _ in rows) + 20
    board = Image.new('RGB', (width, height), (16, 16, 16))
    y = 10
    for row in rows:
        x = 10
        for path, label in row:
            if path.exists():
                board.paste(_load(path, label), (x, y))
            x += THUMB[0] + 10
        y += THUMB[1] + 26 + 12
    out = HERE / 'output/appearance_board.jpg'
    board.save(out, quality=90)
    print('appearance board:', out)


if __name__ == '__main__':
    main()
