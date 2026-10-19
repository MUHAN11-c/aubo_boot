"""
程序化贴图生成（PIL + 标准库；同 seed 必同图）.

真实感来自材质而不是堆多边形：纸袋褶皱与袋底折痕、树皮纵裂、桃果皮红晕与缝合线、
土壤团粒、草带条纹，配合 SDF ``<material><pbr><metal>`` 的 albedo/normal/roughness。
高度图→法线图用有限差分（切线空间，OpenGL 约定 +Y 上）。

产物（由 ``cli`` 生成到 ``worlds/textures/``，SDF 以相对 URI 引用）：
``paper_bag``（含袋底折边区/透孔/针孔）、``bark``、``leaf``、``soil``、``grass``、
``fruit_skin`` 各出 ``*_albedo.png`` + ``*_normal.png``。
"""

from __future__ import annotations

import math
from pathlib import Path
import random

from PIL import Image, ImageDraw, ImageFilter

SIZE = 512

# 材质基调（与 scene.py 的低模颜色同一口径；贴图在其上做明暗变化）
PAPER = (208, 96, 102)       # 纸袋玫红（实测渲染目标 (139,68,71)、饱和 77）
PAPER_DARK = (140, 58, 62)   # 折痕阴影
INNER_PAPER = (46, 38, 34)         # 内衬遮光纸层
_BARK = (86, 66, 48)
_BARK_DARK = (48, 36, 26)
_LEAF = (58, 88, 40)
_LEAF_LIGHT = (96, 128, 62)
_SOIL = (92, 72, 52)
_SOIL_LIGHT = (126, 104, 78)
_GRASS = (72, 96, 46)
_GRASS_DRY = (132, 138, 78)
_FRUIT_GROUND = (216, 198, 132)  # 套袋桃底色（淡黄乳白，实测 102,92,50 提亮）
_FRUIT_BLUSH = (188, 118, 88)    # 少量红晕（袋内受光弱）


TexturePair = tuple[Image.Image, Image.Image]


def _rng(seed: int, *parts: str) -> random.Random:
    return random.Random(f'{seed}/{"|".join(parts)}')


def _noise(size: int, rng: random.Random, cells: int,
           blur: float) -> Image.Image:
    """低频值噪声：小图放大 + 模糊，代替逐像素随机（更像自然纹理）."""
    small = Image.new('L', (cells, cells))
    small.putdata([rng.randint(0, 255) for _ in range(cells * cells)])
    up = small.resize((size, size), Image.BICUBIC)
    return up.filter(ImageFilter.GaussianBlur(blur))


def height_to_normal(height: Image.Image, strength: float = 2.0) -> Image.Image:
    """高度图 → 切线空间法线图（有限差分；+Y 上、Z 朝外）."""
    width, height_px = height.size
    pixels = height.load()
    out = Image.new('RGB', (width, height_px))
    target = out.load()
    for y in range(height_px):
        for x in range(width):
            left = pixels[(x - 1) % width, y]
            right = pixels[(x + 1) % width, y]
            up = pixels[x, (y - 1) % height_px]
            down = pixels[x, (y + 1) % height_px]
            dx = (int(left) - int(right)) / 255.0 * strength
            dy = (int(down) - int(up)) / 255.0 * strength
            norm = math.sqrt(dx * dx + dy * dy + 1.0)
            target[x, y] = (
                int(round((dx / norm * 0.5 + 0.5) * 255)),
                int(round((dy / norm * 0.5 + 0.5) * 255)),
                int(round((1.0 / norm * 0.5 + 0.5) * 255)),
            )
    return out


def _shade(base: tuple[int, int, int], factor: float) -> tuple[int, int, int]:
    return tuple(max(0, min(255, int(round(channel * factor))))
                 for channel in base)


def _blend(a: tuple[int, int, int], b: tuple[int, int, int],
           weight: float) -> tuple[int, int, int]:
    return tuple(int(round(a[i] * (1.0 - weight) + b[i] * weight))
                 for i in range(3))


def paper_bag(seed: int) -> TexturePair:
    """
    套袋纸袋：纵向折痕 + 袋底折边压痕 + 透孔/针孔（专利 CN201025802Y 构造）.

    UV 约定（球/椭球原语）：u 绕袋周、v 沿袋轴（v=0 袋底 → v=1 袋口）。
    """
    rng = _rng(seed, 'paper_bag')
    size = SIZE
    albedo = Image.new('RGB', (size, size))
    height = Image.new('L', (size, size), 128)
    draw_a = ImageDraw.Draw(albedo)
    draw_h = ImageDraw.Draw(height)
    mottle = _noise(size, rng, 8, 3.0)

    # 纸底：粉纸底 + 红通道色斑（实测真袋色度散布 +13.8 在 R 通道）
    tint = _noise(size, rng, 24, 1.2)   # 细粒纹（补高频能量 7.4→11.9）
    for y in range(size):
        for x in range(size):
            factor = 0.72 + 0.52 * mottle.getpixel((x, y)) / 255.0
            grain = 0.88 + 0.24 * tint.getpixel((x, y)) / 255.0
            base = _shade(PAPER, factor * grain)
            swing = int((mottle.getpixel((x, y)) - 128) * 0.12)   # R 通道色斑
            draw_a.point((x, y), fill=(
                max(0, min(255, base[0] + swing)),
                max(0, min(255, base[1] + swing // 3)),
                max(0, min(255, base[2] + swing // 4))))

    # 纵向折痕（绕袋周等距 + 抖动）：袋体被果实撑起前的自然皱褶
    creases = 11
    for index in range(creases):
        center = int((index + rng.random() * 0.6) * size / creases)
        width = rng.randint(3, 8)
        for y in range(size):
            wobble = int(3.0 * math.sin(y / size * math.pi * 2 + index))
            x0 = (center + wobble) % size
            for step in range(-width, width + 1):
                x = (x0 + step) % size
                weight = 1.0 - abs(step) / (width + 1.0)
                draw_a.point((x, y), fill=_blend(
                    _shade(PAPER, 0.82 + 0.3 * mottle.getpixel((x, y)) / 255.0),
                    PAPER_DARK, 0.55 * weight))
                if weight > 0.25:
                    draw_h.point((x, y), fill=int(128 - 78 * weight))

    # 袋底折边（立体折边压痕，v<0.22 一带两条横痕）
    for band in (0.12, 0.20):
        y0 = int(band * size)
        for y in range(y0, y0 + 3):
            for x in range(size):
                draw_a.point((x, y), fill=_blend(
                    albedo.getpixel((x, y)), PAPER_DARK, 0.45))
                draw_h.point((x, y), fill=70)

    # 透孔（袋体上方数个）与针孔（袋体两侧下方）：画成深色小孔
    for _ in range(9):
        cx, cy = rng.randint(0, size - 1), rng.randint(int(size * 0.55),
                                                       int(size * 0.8))
        draw_a.ellipse([cx - 1, cy - 1, cx + 1, cy + 1], fill=INNER_PAPER)
        draw_h.ellipse([cx - 1, cy - 1, cx + 1, cy + 1], fill=40)
    for _ in range(14):
        cx, cy = rng.randint(0, size - 1), rng.randint(int(size * 0.05),
                                                       int(size * 0.3))
        draw_a.point((cx, cy), fill=INNER_PAPER)
        draw_h.point((cx, cy), fill=60)

    # 袋面印花/戳记（真实高分裁剪可见的淡色印记）
    for _ in range(3):
        cx = rng.randint(int(size * 0.15), int(size * 0.85))
        cy = rng.randint(int(size * 0.3), int(size * 0.75))
        rx, ry = rng.randint(14, 30), rng.randint(8, 18)
        tone = _blend(PAPER_DARK, (150, 110, 120), 0.5)
        draw_a.ellipse([cx - rx, cy - ry, cx + rx, cy + ry],
                       outline=tone, width=3)
        draw_a.ellipse([cx - rx // 2, cy - ry // 2, cx + rx // 2,
                        cy + ry // 2], outline=tone, width=2)

    # 袋口一带压暗（束口聚集处纸层叠起）
    for y in range(int(size * 0.88), size):
        for x in range(size):
            weight = (y - size * 0.88) / (size * 0.12)
            draw_a.point((x, y), fill=_blend(albedo.getpixel((x, y)),
                                             PAPER_DARK, 0.5 * weight))
            draw_h.point((x, y), fill=int(128 + 50 * weight))

    albedo = albedo  # 不再模糊：保留高频细节（实测高频 9.7 vs 3.1）
    return (albedo, height_to_normal(height, 4.2))


def bark(seed: int) -> TexturePair:
    """树皮：纵裂纹 + 粗糙横纹."""
    rng = _rng(seed, 'bark')
    size = SIZE
    albedo = Image.new('RGB', (size, size))
    height = Image.new('L', (size, size), 128)
    draw_a = ImageDraw.Draw(albedo)
    draw_h = ImageDraw.Draw(height)
    mottle = _noise(size, rng, 6, 2.0)
    for y in range(size):
        for x in range(size):
            draw_a.point((x, y), fill=_shade(
                _BARK, 0.8 + 0.4 * mottle.getpixel((x, y)) / 255.0))
    for _ in range(26):
        x0 = rng.randint(0, size - 1)
        depth = rng.random()
        y0 = rng.randint(-20, size)
        length = rng.randint(size // 3, size)
        for step in range(length):
            y = (y0 + step) % size
            drift = int(4.0 * math.sin(step / 12.0 + depth * 6))
            x = (x0 + drift) % size
            draw_a.line([(x, y), (x, y)], fill=_BARK_DARK, width=2)
            draw_h.point((x, y), fill=40)
    return (albedo, height_to_normal(height, 3.0))


def leaf(seed: int) -> TexturePair:
    """叶幕：深浅叶簇斑驳（冠层球贴图）."""
    rng = _rng(seed, 'leaf')
    size = SIZE
    albedo = Image.new('RGB', (size, size))
    height = Image.new('L', (size, size), 128)
    draw_a = ImageDraw.Draw(albedo)
    draw_h = ImageDraw.Draw(height)
    base = _noise(size, rng, 10, 2.5)
    for y in range(size):
        for x in range(size):
            draw_a.point((x, y), fill=_blend(
                _LEAF, _LEAF_LIGHT, base.getpixel((x, y)) / 255.0))
    for _ in range(1500):  # 单叶小斑（细碎叶簇；冠面纹理是叶幕对比度主源）
        cx, cy = rng.randint(0, size - 1), rng.randint(0, size - 1)
        radius = rng.randint(2, 5)
        tone = rng.random() ** 0.55
        shade = 0.55 + 0.85 * rng.random()
        base = _blend(_LEAF, _LEAF_LIGHT, tone)
        draw_a.ellipse([cx - radius, cy - radius // 2, cx + radius,
                        cy + radius // 2],
                       fill=tuple(max(0, min(255, int(c * shade))) for c in base),
                       outline=_shade(_LEAF, 0.55))
        draw_h.ellipse([cx - radius, cy - radius // 2, cx + radius,
                        cy + radius // 2], fill=int(128 + 60 * (tone - 0.5)))
    return (albedo, height_to_normal(height, 1.6))


def soil(seed: int) -> TexturePair:
    """土壤：团粒 + 碎石斑."""
    rng = _rng(seed, 'soil')
    size = SIZE
    albedo = Image.new('RGB', (size, size))
    height = Image.new('L', (size, size), 128)
    draw_a = ImageDraw.Draw(albedo)
    draw_h = ImageDraw.Draw(height)
    base = _noise(size, rng, 12, 2.0)
    for y in range(size):
        for x in range(size):
            draw_a.point((x, y), fill=_blend(
                _SOIL, _SOIL_LIGHT, base.getpixel((x, y)) / 255.0))
    for _ in range(160):
        cx, cy = rng.randint(0, size - 1), rng.randint(0, size - 1)
        radius = rng.randint(1, 4)
        tone = rng.random()
        draw_a.ellipse([cx - radius, cy - radius, cx + radius, cy + radius],
                       fill=_shade(_SOIL_LIGHT, 0.8 + 0.5 * tone))
        draw_h.ellipse([cx - radius, cy - radius, cx + radius, cy + radius],
                       fill=int(128 + 45 * (tone - 0.4)))
    return (albedo, height_to_normal(height, 2.2))


def grass(seed: int) -> TexturePair:
    """草带：草叶条纹 + 枯草斑."""
    rng = _rng(seed, 'grass')
    size = SIZE
    albedo = Image.new('RGB', (size, size))
    height = Image.new('L', (size, size), 128)
    draw_a = ImageDraw.Draw(albedo)
    draw_h = ImageDraw.Draw(height)
    base = _noise(size, rng, 8, 3.0)
    for y in range(size):
        for x in range(size):
            draw_a.point((x, y), fill=_blend(
                _GRASS, _GRASS_DRY, 0.35 * base.getpixel((x, y)) / 255.0))
    for _ in range(420):
        x0, y0 = rng.randint(0, size - 1), rng.randint(0, size - 1)
        length = rng.randint(6, 18)
        lean = rng.randint(-4, 4)
        color = _blend(_GRASS, _GRASS_DRY, rng.random() * 0.6)
        for step in range(length):
            x = (x0 + lean * step // max(length, 1)) % size
            y = (y0 - step) % size
            draw_a.point((x, y), fill=_shade(color, 0.85 + 0.3 * rng.random()))
            draw_h.point((x, y), fill=170)
    return (albedo, height_to_normal(height, 1.4))


def fruit_skin(seed: int) -> TexturePair:
    """桃果皮：黄底红晕 + 缝合线 + 绒毛感细斑."""
    rng = _rng(seed, 'fruit')
    size = SIZE
    albedo = Image.new('RGB', (size, size))
    height = Image.new('L', (size, size), 128)
    draw_a = ImageDraw.Draw(albedo)
    draw_h = ImageDraw.Draw(height)
    blush = _noise(size, rng, 5, 4.0)
    for y in range(size):
        for x in range(size):
            weight = max(0.0, (x / size - 0.35)) * (0.6 + 0.8 * blush.getpixel(
                (x, y)) / 255.0)
            draw_a.point((x, y), fill=_blend(_FRUIT_GROUND, _FRUIT_BLUSH,
                                             min(1.0, weight)))
    for x in range(size):  # 缝合线
        y = int(size * 0.5 + 4.0 * math.sin(x / size * math.pi))
        for offset in range(-1, 2):
            draw_a.point((x, (y + offset) % size),
                         fill=_blend(_FRUIT_BLUSH, (120, 50, 40), 0.5))
            draw_h.point((x, (y + offset) % size), fill=95)
    for _ in range(260):  # 果点
        cx, cy = rng.randint(0, size - 1), rng.randint(0, size - 1)
        draw_a.point((cx, cy), fill=_shade(_FRUIT_GROUND, 1.1))
    return (albedo, height_to_normal(height, 1.0))


# 曝光对齐（渲染↔数据集实测闭环）：纸袋渲染中位 65,46,33 vs 实测 83,54,47（×1.25）、
# 叶幕 46,53,25 vs 92,104,80（×1.45）；其余材质取同量级 1.3。
EXPOSURE_GAIN = {
    'paper_bag': 1.0,
    'leaf': 2.10,
    'bark': 1.30,
    'soil': 1.30,
    'grass': 1.35,
    'fruit_skin': 1.20,
}


def leaf_card(seed: int):
    """
    叶簇卡（RGBA 带透明缝隙）：真叶是离散叶片、枝间有透光缝隙.

    用于叶幕叶卡（alpha clip），制造叶级对比度（实测明暗起伏 38 vs 连续面 24）。
    """
    rng = _rng(seed, 'leaf_card')
    size = SIZE
    image = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    for _ in range(90):   # 离散叶片
        cx, cy = rng.randint(4, size - 4), rng.randint(4, size - 4)
        length = rng.randint(14, 30)
        width = rng.randint(7, 14)
        angle = rng.uniform(0.0, 3.14)
        # 叶片明暗反差拉大（阳面叶亮/阴面叶暗）：实测叶幕明暗起伏 38 vs 连续面 23
        tone = rng.random() ** 0.6
        base = _blend(_LEAF, _LEAF_LIGHT, tone)
        shade = 0.62 + 0.75 * rng.random()
        color = tuple(max(0, min(255, int(c * shade))) for c in base) + (255,)
        # 叶片=旋转椭圆（用两点线段+宽度近似）
        dx = math.cos(angle) * length / 2.0
        dy = math.sin(angle) * length / 2.0
        draw.line([(cx - dx, cy - dy), (cx + dx, cy + dy)],
                  fill=color, width=width)
    return (image, None)


_GENERATORS = {
    'leaf_card': leaf_card,
    'paper_bag': paper_bag,
    'bark': bark,
    'leaf': leaf,
    'soil': soil,
    'grass': grass,
    'fruit_skin': fruit_skin,
}


def generate_all(out_dir: Path, seed: int) -> dict[str, tuple[Path, Path]]:
    """按 seed 生成全套贴图到 ``out_dir``，返回名称→(albedo, normal) 路径."""
    out_dir.mkdir(parents=True, exist_ok=True)
    written: dict[str, tuple[Path, Path]] = {}
    for name, generator in sorted(_GENERATORS.items()):
        pair = generator(seed)
        if pair[1] is None:   # 仅 albedo（RGBA 叶卡）
            albedo_path = out_dir / f'{name}_albedo.png'
            pair[0].save(albedo_path)
            written[name] = (albedo_path, albedo_path)
            continue
        albedo, normal = pair
        # 细粒颗粒：补像素级高频（实测真实高频能量 9.7 vs 合成 3.1）
        grain_rng = random.Random(f'{seed}/grain/{name}')
        pixels = albedo.load()
        for y in range(albedo.size[1]):
            for x in range(albedo.size[0]):
                pixel = pixels[x, y]
                if len(pixel) == 4 and pixel[3] == 0:
                    continue
                jitter = grain_rng.randint(-12, 12)
                pixels[x, y] = tuple(
                    max(0, min(255, c + jitter)) for c in pixel[:3]) + pixel[3:]
        gain = EXPOSURE_GAIN.get(name, 1.0)
        if gain != 1.0:
            albedo = albedo.point(
                lambda value: min(255, int(round(value * gain))))
        albedo_path = out_dir / f'{name}_albedo.png'
        normal_path = out_dir / f'{name}_normal.png'
        albedo.save(albedo_path)
        normal.save(normal_path)
        written[name] = (albedo_path, normal_path)
    return written
