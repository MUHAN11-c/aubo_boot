"""程序化桃叶 / 纸袋 / 树皮贴图。颜色锚 Peach_bag 框内中位 (101, 60, 55)。"""

from __future__ import annotations

import os

import numpy as np
from PIL import Image

OUT = os.path.join(os.path.dirname(__file__), '..', 'textures')


def _leaf_surface(path: str) -> None:
    """不透明叶面：中脉 + 侧脉，铺在披针网格上。"""
    rng = np.random.default_rng(5)
    height, width = 256, 128
    yy = np.linspace(0, 1, height, dtype=np.float32)[:, None]
    xx = np.linspace(-1, 1, width, dtype=np.float32)[None, :]
    green = np.zeros((height, width, 3), dtype=np.float32)
    green[..., 0] = 48
    green[..., 1] = 112
    green[..., 2] = 36
    green += rng.normal(0, 5, green.shape)
    mid = np.exp(-((xx / 0.07) ** 2))
    green[..., 1] -= 30 * mid
    for along, slant in ((0.28, 0.4), (0.48, 0.4), (0.68, 0.4)):
        vein = np.exp(-(((yy - along) - slant * np.abs(xx)) ** 2) / 0.0006)
        green[..., 1] -= 16 * vein
    Image.fromarray(np.clip(green, 0, 255).astype(np.uint8), 'RGB').save(path)


def _paper(path: str) -> None:
    rng = np.random.default_rng(7)
    height, width = 512, 512
    base = np.array([168, 52, 36], dtype=np.float32)
    noise = rng.normal(0, 5, (height, width, 1)).astype(np.float32)
    x = np.linspace(0, 1, width, dtype=np.float32)
    y = np.linspace(0, 1, height, dtype=np.float32)[:, None]
    x = x[None, :]
    folds = np.zeros((height, width), dtype=np.float32)
    for center in (0.18, 0.25, 0.32):
        folds += np.exp(-((x - center) / 0.028) ** 2)
    for slope, offset in ((1.6, 0.15), (-0.9, 0.62), (2.4, -0.2), (0.4, 0.78)):
        dist = np.abs((y - offset) - slope * (x - 0.5))
        folds += 0.7 * np.exp(-((dist / 0.012) ** 2))
    yy = np.linspace(0, 1, height, dtype=np.float32)[:, None]
    xx = np.linspace(0, 1, width, dtype=np.float32)[None, :]
    for _ in range(36):
        cx, cy = rng.random(), rng.random()
        angle = rng.uniform(0, np.pi)
        dx, dy = np.cos(angle), np.sin(angle)
        along = (xx - cx) * dx + (yy - cy) * dy
        across = (xx - cx) * -dy + (yy - cy) * dx
        length = rng.uniform(0.04, 0.16)
        folds += rng.uniform(0.35, 0.8) * np.exp(
            -((across / 0.008) ** 2) - ((along / length) ** 2))
    image = np.clip(base + noise + (-50 * folds)[..., None], 0, 255).astype(np.uint8)
    Image.fromarray(image, 'RGB').save(path)


def _bark(path: str) -> None:
    rng = np.random.default_rng(3)
    height, width = 512, 256
    y = np.linspace(0, 1, height, dtype=np.float32)[:, None, None]
    base = np.array([92, 64, 42], dtype=np.float32) * (0.75 + 0.35 * y)
    grain = rng.normal(0, 10, (height, width, 1)).astype(np.float32)
    ridges = (14 * np.sin(np.linspace(0, 40 * np.pi, height, dtype=np.float32)))
    image = np.clip(base + grain + ridges[:, None, None], 0, 255).astype(np.uint8)
    Image.fromarray(image, 'RGB').save(path)


def _grass(path: str) -> None:
    rng = np.random.default_rng(9)
    height, width = 512, 512
    yy = np.linspace(0, 1, height, dtype=np.float32)[:, None]
    xx = np.linspace(0, 1, width, dtype=np.float32)[None, :]
    image = np.zeros((height, width, 3), dtype=np.float32)
    image[..., 0] = 52
    image[..., 1] = 98
    image[..., 2] = 38
    image += rng.normal(0, 7, image.shape)
    blotch = (
        np.sin(xx * 18 + yy * 7) * np.sin(yy * 23)
        + 0.6 * np.sin(xx * 41) * np.cos(yy * 29))
    image[..., 1] += 14 * blotch
    image[..., 0] += 6 * np.sin(xx * 11 + 1.7)
    Image.fromarray(np.clip(image, 0, 255).astype(np.uint8), 'RGB').save(path)


def _soil(path: str) -> None:
    rng = np.random.default_rng(4)
    height, width = 256, 256
    image = np.zeros((height, width, 3), dtype=np.float32)
    image[..., 0] = 96
    image[..., 1] = 72
    image[..., 2] = 46
    image += rng.normal(0, 8, image.shape)
    blotch = rng.normal(0, 1, (32, 32)).astype(np.float32)
    blotch = np.repeat(np.repeat(blotch, 8, axis=0), 8, axis=1)
    image += 10 * blotch[..., None]
    Image.fromarray(np.clip(image, 0, 255).astype(np.uint8), 'RGB').save(path)


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    _leaf_surface(os.path.join(OUT, 'leaf_surface.png'))
    _paper(os.path.join(OUT, 'paper.png'))
    _bark(os.path.join(OUT, 'bark.png'))
    _grass(os.path.join(OUT, 'grass.png'))
    _soil(os.path.join(OUT, 'soil.png'))


if __name__ == '__main__':
    main()
