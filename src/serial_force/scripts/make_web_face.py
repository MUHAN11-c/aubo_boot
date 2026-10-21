#!/usr/bin/env python3
"""
给 force_web.html 生成两张 +Z 侧渲染（均 1:1，盘心居中）.

  src/serial_force/scripts/face_fruit.png   72 mm 视场（盘面视图）
  src/serial_force/scripts/face_full.png   220 mm 视场（整机视图）

网页按 R=16.35 mm 在同一 tcp 坐标上叠加五点；果实侧是同一渲染的 CSS 镜像
（与 render_force_pads.py 的 ③ 同向），臂侧是直接视图。
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[3]
STL = ROOT / 'src/aubo_description/meshes/visual/tools/bite_shear_v1.stl'
OUT_DIR = ROOT / 'src/serial_force/scripts'
Z0 = 0.1655
SIZE = 900


def load_stl(path: Path) -> np.ndarray:
    with open(path, 'rb') as fh:
        fh.read(80)
        n = struct.unpack('<I', fh.read(4))[0]
        raw = fh.read()
    rec = np.frombuffer(raw, dtype=np.uint8)[: n * 50].reshape(n, 50)
    return rec[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)


def render(tris, right, up, fwd, light, half, size=SIZE, ss=2, bg=(0.047, 0.063, 0.078)):
    W = H = size * ss
    zbuf = np.full((H, W), -1e9, np.float32)
    img = np.empty((H, W, 3), np.float32)
    img[:] = bg
    sx0, sx1, sy0, sy1 = -half, half, -Z0 - half, -Z0 + half

    basis = np.stack([right, up, fwd])
    p = (tris.reshape(-1, 3) @ basis.T).reshape(-1, 3, 3)
    sx, sy, dep = p[..., 0], p[..., 1], -p[..., 2]   # dep>0 = 离相机近
    px = (sx - sx0) / (sx1 - sx0) * (W - 1)
    py = (sy - sy0) / (sy1 - sy0) * (H - 1)

    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    front = (n @ fwd) < 0
    n[~front] *= -1
    light = np.asarray(light, float)
    light /= np.linalg.norm(light)
    shade = 0.34 + 0.66 * np.clip(n @ light, 0.0, 1.0)
    base = np.array([0.40, 0.44, 0.49])
    colors = np.clip(base[None, :] * shade[:, None], 0, 1)

    for i in range(len(tris)):
        if not front[i]:
            continue
        ix0 = max(int(np.floor(px[i].min())), 0)
        ix1 = min(int(np.ceil(px[i].max())), W - 1)
        iy0 = max(int(np.floor(py[i].min())), 0)
        iy1 = min(int(np.ceil(py[i].max())), H - 1)
        if ix1 < ix0 or iy1 < iy0:
            continue
        gx, gy = np.meshgrid(np.arange(ix0, ix1 + 1) + 0.5,
                             np.arange(iy0, iy1 + 1) + 0.5)
        ax, ay = px[i, 0], py[i, 0]
        bx, by = px[i, 1], py[i, 1]
        cx, cy = px[i, 2], py[i, 2]
        d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(d) < 1e-9:
            continue
        w0 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / d
        w1 = ((cy - ay) * (gx - cx) + (ax - cx) * (gy - cy)) / d
        w2 = 1.0 - w0 - w1
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not inside.any():
            continue
        z = w0 * dep[i, 0] + w1 * dep[i, 1] + w2 * dep[i, 2]
        sub = zbuf[iy0:iy1 + 1, ix0:ix1 + 1]
        upd = inside & (z > sub)
        if not upd.any():
            continue
        sub[upd] = z[upd]
        img[iy0:iy1 + 1, ix0:ix1 + 1][upd] = colors[i]

    img = img.reshape(H // ss, ss, W // ss, ss, 3).mean(axis=(1, 3))
    # 径向压暗，融进页面背景
    yy, xx = np.mgrid[0:size, 0:size]
    r = np.hypot(xx - (size - 1) / 2, yy - (size - 1) / 2) / (size / 2)
    img *= (1.0 - 0.30 * np.clip(r, 0, 1) ** 2)[..., None]
    return img[::-1]          # PNG row0 = 上


tris = load_stl(STL)
R_F = np.array([1.0, 0.0, 0.0])
U = np.array([0.0, 0.0, -1.0])
F_F = np.array([0.0, -1.0, 0.0])
F_A = np.array([0.0, 1.0, 0.0])
R_A = np.array([-1.0, 0.0, 0.0])

for name, half in (('face_fruit.png', 0.036), ('face_full.png', 0.110)):
    img = render(tris, R_F, U, F_F, (-0.40, 0.86, -0.32), half)
    arr = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    arr.quantize(colors=160, method=Image.MEDIANCUT, dither=Image.NONE).save(
        OUT_DIR / name, optimize=True)
    print(name, (OUT_DIR / name).stat().st_size)
