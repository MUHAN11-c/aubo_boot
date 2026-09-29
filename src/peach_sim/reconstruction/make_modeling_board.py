"""Build visual review sheets without mixing historical matrix renders."""

import json
from pathlib import Path

from PIL import Image, ImageDraw

from render_evidence import validate_render_set

HERE = Path(__file__).resolve().parent
OUT = HERE / 'output'
REVIEW = OUT / 'modeling_20260929'


def sheet(rows, destination):
    """Arrange labeled images with a fixed aspect ratio."""
    width, height = 640, 360
    board = Image.new('RGB', (len(rows[0]) * width, len(rows) * (height + 30)), '#171b20')
    draw = ImageDraw.Draw(board)
    for row, cells in enumerate(rows):
        for column, (path, label) in enumerate(cells):
            x, y = column * width, row * (height + 30)
            draw.text((x + 10, y + 8), label, fill='white')
            with Image.open(path) as image:
                board.paste(image.convert('RGB').resize((width, height)), (x, y + 30))
    board.save(destination, quality=93)


def main():
    """Compare baseline, current model and five current lighting conditions."""
    manifest = json.loads((OUT / 'scene_manifest.json').read_text())
    validate_render_set(OUT, manifest, ('reference', 'detail', 'orchard'))
    rows = []
    for view in ('reference', 'detail', 'orchard'):
        rows.append([(REVIEW / 'before' / f'{view}.png', f'Before / {view}'),
                     (OUT / f'{view}.png', f'Current / {view}')])
    sheet(rows, REVIEW / 'before_after.jpg')
    frames = [(Path('/home/mu/Downloads/PeachDataSet/Peach_bag/RGB/1200.png'),
               'Real RGB-D reference 1200 / watermarked source'),
              (OUT / 'reference.png', 'Current / noon')]
    for light in ('morning', 'late_afternoon', 'backlit', 'overcast'):
        folder = REVIEW / light
        other = json.loads((folder / 'scene_manifest.json').read_text())
        if other['source_blend_sha256'] != manifest['source_blend_sha256']:
            raise ValueError(f'{light}: different blend revision')
        validate_render_set(folder, other, ('reference', 'detail', 'orchard'))
        frames.append((folder / 'reference.png', f'Current / {light}'))
    sheet([frames[:2], frames[2:4], frames[4:]], REVIEW / 'lighting_comparison.jpg')
    print(REVIEW / 'before_after.jpg')
    print(REVIEW / 'lighting_comparison.jpg')


if __name__ == '__main__':
    main()
