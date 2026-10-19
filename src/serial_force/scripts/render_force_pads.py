#!/usr/bin/env python3
"""
5 点力膜片在咬合末端上的位置核对图（静态 PNG）.

数据源：
  * 膜片 5 点 = CAD 总装提取报告 _tools/shear_pipeline/bite2/extract_report.json
    里的 5 个 "电阻式薄膜压力传感器 力敏电阻[000..004]" 零件 bbox 中心；
  * 渲染网格 = URDF 同一份 visual STL（sha256 与 CAD 提取的 visual.stl 相同）；
  * tool_body_link 与 wrist3_Link 是 identity，故 CAD 总装坐标即 URDF 坐标。

坐标：tcp 系，+Z = 开口方向，+Y = 盘面“上”，+X = 盘面“右”。
"""
from __future__ import annotations

import struct
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle

ROOT = Path(__file__).resolve().parents[3]
STL = ROOT / 'src/aubo_description/meshes/visual/tools/bite_shear_v1.stl'
OUT = ROOT / 'src/serial_force/doc' / 'force_pads_on_tool.png'

# ── 字体 ────────────────────────────────────────────────────────────────
for _f in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
           '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'):
    if Path(_f).exists():
        fm.fontManager.addfont(_f)
plt.rcParams['font.family'] = 'Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus'] = False

# ── 几何 ────────────────────────────────────────────────────────────────
Z0 = 0.1655                                          # 盘心 z（wrist3）
PLATE_HALF = 0.0275                                  # 盖板 55 × 55
PAD_Z = -0.07229                                     # 膜片平面（tcp z）

# CAD 件号 -> (tcp 系 xyz / m, 自 +Y 顺时针角 / deg, 推定通道)
PADS = {
    '000': ((0.0, 0.01645, PAD_Z), 0.0, 5),
    '001': ((0.01575, 0.00430, PAD_Z), 74.7, 7),
    '002': ((0.00945, -0.01330, PAD_Z), 144.6, 3),
    '004': ((-0.00965, -0.01315, PAD_Z), 216.3, 2),
    '003': ((-0.01570, 0.00455, PAD_Z), 286.2, 1),
}
ORDER = ['000', '001', '002', '004', '003']           # 顺时针


def load_stl(path: Path) -> np.ndarray:
    with open(path, 'rb') as fh:
        fh.read(80)
        n = struct.unpack('<I', fh.read(4))[0]
        raw = fh.read()
    rec = np.frombuffer(raw, dtype=np.uint8)[: n * 50].reshape(n, 50)
    return rec[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)


def render(tris, right, up, fwd, win, size=1200, ss=2, bg=(0.050, 0.068, 0.082)):
    """
    正交 z-buffer 渲染.

    win=(sx0,sx1,sy0,sy1)，row0 = sy0。
    """
    sx0, sx1, sy0, sy1 = win
    W = H = size * ss
    zbuf = np.full((H, W), -1e9, np.float32)
    img = np.empty((H, W, 3), np.float32)
    img[:] = bg

    basis = np.stack([right, up, fwd])
    p = (tris.reshape(-1, 3) @ basis.T).reshape(-1, 3, 3)
    sx, sy, dep = p[..., 0], p[..., 1], -p[..., 2]   # dep>0 = 离相机近
    px = (sx - sx0) / (sx1 - sx0) * (W - 1)
    py = (sy - sy0) / (sy1 - sy0) * (H - 1)

    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    front = (n @ fwd) < 0
    n[~front] *= -1

    light = np.array([-0.45, 0.82, -0.36])
    light /= np.linalg.norm(light)
    shade = 0.40 + 0.60 * np.clip(n @ light, 0.0, 1.0)
    base = np.array([0.52, 0.57, 0.62])
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

    return img.reshape(H // ss, ss, W // ss, ss, 3).mean(axis=(1, 3))


# 果实侧：相机在 +Z_tcp 看 -Z_tcp；右=+X_tcp、上=+Y_tcp
R_F = np.array([1.0, 0.0, 0.0])
U_F = np.array([0.0, 0.0, -1.0])
F_F = np.array([0.0, -1.0, 0.0])
R_A = np.array([-1.0, 0.0, 0.0])                      # 臂侧：右=-X_tcp

FULL = (-0.105, 0.105, -Z0 - 0.105, -Z0 + 0.105)
ZOOM = (-0.036, 0.036, -Z0 - 0.036, -Z0 + 0.036)

tris = load_stl(STL)
full = render(tris, R_F, U_F, F_F, FULL, size=1100)
zoom = render(tris, R_F, U_F, F_F, ZOOM, size=1200)
mirror = render(tris, R_A, U_F, F_F, ZOOM, size=1200)

# ── 画布 ────────────────────────────────────────────────────────────────
BG = '#0c1014'
FG = '#e6edf3'
DIM = '#8b98a3'
ACC = '#ffb648'
RED = '#ff5a5a'

fig = plt.figure(figsize=(15.2, 14.0), facecolor=BG)
gs = fig.add_gridspec(2, 2, left=0.025, right=0.975, top=0.918, bottom=0.060,
                      wspace=0.055, hspace=0.095)


def show(ax, img, win, title, sub=None, sub_y=0.975):
    sx0, sx1, sy0, sy1 = win
    ax.set_facecolor(BG)
    ax.imshow(img, extent=(sx0, sx1, sy0, sy1), origin='lower', interpolation='bilinear')
    ax.set_xlim(sx0, sx1)
    ax.set_ylim(sy0, sy1)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color('#2b3742')
    ax.set_title(title, color=FG, fontsize=15.5, pad=(26 if sub else 9), loc='left')
    if sub:
        ax.text(0.0, 1.006, sub, transform=ax.transAxes, color=DIM, fontsize=10.2,
                va='bottom', ha='left', linespacing=1.45)


def pad_overlay(ax, win, mirror=False):
    sx0, sx1, sy0, sy1 = win
    sg = -1.0 if mirror else 1.0
    # 55×55 盖板
    ax.add_patch(Rectangle((sg * -PLATE_HALF if sg > 0 else -PLATE_HALF, -Z0 - PLATE_HALF),
                           2 * PLATE_HALF, 2 * PLATE_HALF, fill=False,
                           ec='#e8eef4', lw=1.1, ls=(0, (4, 3)), alpha=0.55))
    # R = 16.35 参考圆
    ang = np.linspace(0, 2 * np.pi, 400)
    ax.plot(sg * 0.01635 * np.sin(ang), -Z0 + 0.01635 * np.cos(ang),
            color=ACC, lw=0.9, ls=(0, (2, 4)), alpha=0.55)
    # 五边形
    px = [sg * PADS[k][0][0] for k in ORDER]
    py = [-Z0 + PADS[k][0][1] for k in ORDER]
    ax.plot(px + [px[0]], py + [py[0]], color=FG, lw=0.9, alpha=0.40)
    ax.plot(0, -Z0, marker='+', ms=12, mew=1.5, color=FG, zorder=4)
    for key in ORDER:
        xyz, angle, ch = PADS[key]
        x, y = sg * xyz[0], -Z0 + xyz[1]
        ax.add_patch(Circle((x, y), 0.00275, fc=RED, ec='white', lw=1.1, zorder=5))
        ax.text(x, y - 0.00015, str(ch), color='white', fontsize=10, fontweight='bold',
                ha='center', va='center', zorder=6)
        dx, dy = x, y + Z0
        rr = np.hypot(dx, dy) or 1.0
        off = 0.0125 if key == '000' else 0.0082
        tx, ty = x + dx / rr * off, y + dy / rr * off
        ha = 'center' if abs(tx) < 0.0035 else ('left' if tx > 0 else 'right')
        va = 'bottom' if ty > -Z0 + 0.004 else ('top' if ty < -Z0 - 0.004 else 'center')
        ax.text(tx, ty, f'CAD {key}\n{angle:.1f}°', color='#c9d4dd', fontsize=8.8,
                ha=ha, va=va, linespacing=1.25)
    # 10 mm 比例尺
    x0, y0 = sx0 + 0.006, sy0 + 0.005
    ax.add_patch(Rectangle((x0 - 0.0018, y0 - 0.0012), 0.0142, 0.0068,
                           fc='#0c1014', ec='none', alpha=0.62, zorder=3))
    ax.plot([x0, x0 + 0.010], [y0, y0], color=FG, lw=2.4, solid_capstyle='butt', zorder=4)
    ax.text(x0 + 0.005, y0 + 0.0012, '10 mm', color=FG, fontsize=8.5, ha='center',
            va='bottom', zorder=4)


# ① 整机正面
ax = fig.add_subplot(gs[0, 0])
show(ax, full, FULL, '① 咬合末端正面（沿 −Z_tcp 看）')
ax.add_patch(Rectangle((-PLATE_HALF, -Z0 - PLATE_HALF), 2 * PLATE_HALF, 2 * PLATE_HALF,
                       fill=False, ec=ACC, lw=1.8, ls=(0, (4, 3))))
# 把五点直接叠到整机视图上（同一 tcp 坐标，换到 wrist3 的 x/z）
ang0 = np.linspace(0, 2 * np.pi, 360)
ax.plot(0.01635 * np.sin(ang0), -Z0 + 0.01635 * np.cos(ang0),
        color=ACC, lw=0.8, ls=(0, (2, 4)), alpha=0.75)
for _k in ORDER:
    _xyz, _ang, _ch = PADS[_k]
    _x, _y = _xyz[0], -Z0 + _xyz[1]
    ax.plot(_x, _y, 'o', ms=9.0, mfc='#ff5a5a', mec='white', mew=1.0, zorder=6)
    ax.text(_x, _y, str(_ch), color='white', fontsize=7.2, fontweight='bold',
            ha='center', va='center', zorder=7)
ax.annotate('5 点力膜片盘 55 × 55\n（盖板内侧，膜片不可见）',
            xy=(-0.020, -Z0 - 0.020), xytext=(-0.098, -0.256),
            color=ACC, fontsize=11.5, va='center', linespacing=1.5,
            arrowprops=dict(arrowstyle='-', color=ACC, lw=1.1, alpha=0.85,
                            connectionstyle='arc3,rad=-0.15'))
ax.annotate('', xy=(0.086, -0.256), xytext=(0.054, -0.256),
            arrowprops=dict(arrowstyle='-|>', color=FG, lw=1.0))
ax.text(0.089, -0.256, '+X_tcp', color=FG, fontsize=9.5, va='center')
ax.annotate('', xy=(-0.096, -0.222), xytext=(-0.096, -0.252),
            arrowprops=dict(arrowstyle='-|>', color=FG, lw=1.0))
ax.text(-0.093, -0.216, '+Y_tcp', color=FG, fontsize=9.5, va='bottom')
ax.text(-0.103, -0.069, '视线沿 −Z_tcp（从果实侧）', color=DIM, fontsize=9.2, va='top')

# ② 盘面放大（果实侧）
ax = fig.add_subplot(gs[0, 1])
show(ax, zoom, ZOOM, '② 盘面放大 · 从果实侧看（与网页同向）')
pad_overlay(ax, ZOOM)

# ③ 臂侧镜像
ax = fig.add_subplot(gs[1, 0])
show(ax, mirror, ZOOM, '③ 同一盘 · 从臂侧看（左右镜像）')
pad_overlay(ax, ZOOM, mirror=True)

# ④ 轴向关系
ax = fig.add_subplot(gs[1, 1])
ax.set_facecolor(BG)
ax.set_xlim(-82, 44)
ax.set_ylim(-1.9, 1.7)
ax.set_yticks([])
ax.set_xticks([-80, -60, -40, -20, 0, 20, 40])
ax.tick_params(colors=DIM, labelsize=9)
ax.grid(axis='x', color='#1c2530', lw=0.8)
for s in ax.spines.values():
    s.set_color('#2b3742')
ax.set_title('④ 膜片平面 vs TCP（wrist3 y / mm）', color=FG, fontsize=15.5, pad=9, loc='left')
ax.add_patch(Rectangle((-82, -0.42), 82 + 31.3, 0.84, fc='#2b3742', ec='none'))
ax.annotate('', xy=(-81, 0.0), xytext=(-70, 0.0),
            arrowprops=dict(arrowstyle='-|>', color=DIM, lw=1.0))
ax.text(-80, 0.52, '喉道继续至 −229.5', color=DIM, fontsize=9.5, va='bottom')
ax.add_patch(Rectangle((-46.0, -0.80), 5.0, 1.60, fc=ACC, ec='none'))
ax.plot([-42.05, -42.05], [-1.02, 1.02], color=RED, lw=2.4)
ax.plot([30.24, 30.24], [-1.02, 1.02], color='white', lw=2.4)
ax.annotate('', xy=(-42.05, 1.18), xytext=(30.24, 1.18),
            arrowprops=dict(arrowstyle='<|-|>', color=FG, lw=1.3))
ax.text(-5.9, 1.28, '72.29 mm', color=FG, fontsize=12.5, ha='center', va='bottom')
ax.text(-42.05, -1.10, '力敏膜平面\ny = −42.05', color='#ff8b8b', fontsize=10.2,
        ha='center', va='top', linespacing=1.4)
ax.text(30.24, -1.10, 'TCP\ny = +30.24', color='white', fontsize=10.2, ha='center',
        va='top', linespacing=1.4)
ax.text(0.0, -0.62, 'TCP 在膜片前方（果实侧）72.29 mm', color=FG, fontsize=10.2,
        ha='center', va='center')
ax.text(41, 1.35, '果实侧 →', color=DIM, fontsize=9.5, ha='right', va='bottom')

fig.text(0.5, 0.963, '咬合末端 5 点力：膜片位置 × 网页点位核对', color=FG, fontsize=20,
         ha='center', va='center', fontweight='bold')
fig.text(0.5, 0.033,
         'tcp 系：+Z = 开口，+Y = 盘面“上”，+X = 盘面“右”。② 从果实侧看（与 force_web.html 同向）；'
         '③ 从臂侧看：上下不变，1 ↔ 7、2 ↔ 3 左右互换。',
         color=DIM, fontsize=9.6, ha='center', va='center')
fig.text(0.5, 0.014,
         '五点半径 16.31–16.45 mm，角度 0 / 74.7 / 144.6 / 216.3 / 286.2°（自 +Y 顺时针）；'
         '膜片平面在 tcp z = −72.29 mm；通道号按 2026-10-16“5 = 顶点”推定，尚待现场按压确认。',
         color=DIM, fontsize=9.6, ha='center', va='center')

fig.savefig(OUT, dpi=150, facecolor=BG)
print('saved', OUT, OUT.stat().st_size)
