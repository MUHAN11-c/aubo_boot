#!/usr/bin/env python3
"""深度对比分析（定版）：感知检测/分割统计 + 圆柱拟合可视化 + 点云质量.

只读本次会话数据（jsonl 帧号上限内的最新 npz，防历史残留）。
输出 /tmp/e2e_live/report/：fig_cyl.png（圆柱拟合可视化）、fig_pcq.png（点云质量）
运行：python3 deep_analysis.py（ROOT 可用 E2E_ROOT 覆盖）
"""
import json
import os
import sys

import cv2
import numpy as np

ROOT = os.environ.get('E2E_ROOT', '/tmp/e2e_live')
REPORT = f'{ROOT}/report'
FX, FY, CX, CY = 466.17, 465.56, 326.07, 244.79
TAGS = [('hh4', 'peach_stereo hh4'), ('percipio2', 'percipio driver')]


def jsonl(tag):
    p = (f'{ROOT}/percipio2.jsonl' if tag == 'percipio2'
         else f'{ROOT}/{tag}/frames.jsonl')
    rows = []
    for l in open(p):
        l = l.strip()
        if l.startswith('{'):
            try:
                rows.append(json.loads(l))
            except json.JSONDecodeError:
                pass
    return rows


def latest_npz(tag):
    """本次会话（jsonl 最大帧号）以内最新的 npz 帧号。"""
    cap = max((r['n_proc'] for r in jsonl(tag)), default=0)
    fs = sorted(int(f.split('_')[1].split('.')[0])
                for f in os.listdir(f'{ROOT}/{tag}/npz') if f.endswith('.npz'))
    ok = [f for f in fs if f <= cap]
    return ok[-1] if ok else 0


FRAME = {t: latest_npz(t) for t, _ in TAGS}


def load_npz(tag):
    d = np.load(f'{ROOT}/{tag}/npz/frame_{FRAME[tag]:04d}.npz')
    return d['depth'].astype(np.float32), d['mask']


def img_of(tag):
    fs = sorted(os.listdir(f'{ROOT}/{tag}/img'))
    pref = f'debug_{FRAME[tag]:04d}.png'
    p = f'{ROOT}/{tag}/img/{pref if pref in fs else fs[-1]}'
    return cv2.imread(p)


def project(p3):
    return np.array([FX * p3[0] / p3[2] + CX, FY * p3[1] / p3[2] + CY])


def best_mask(mask, depth):
    """选窗内深度点最多的掩膜实例（=分析目标袋）。最大掩膜可能是左盲带区
    （hh4 主机 SGBM 结构性盲区，深度全 0），故按窗内点数而非面积选。"""
    best, best_n = None, -1
    for lab in np.unique(mask):
        if lab == 0:
            continue
        m = mask == lab
        n = int(((depth >= 300) & (depth <= 1500) & m).sum())
        if n > best_n:
            best, best_n = m, n
    return best


def blind_note(mask, depth):
    """覆盖缺口量化：全图掩膜像素中无窗内深度的占比（左盲带+窗外混合）。"""
    mm = mask > 0
    zero = int((mm & ~((depth >= 300) & (depth <= 1500))).sum())
    return zero, zero / max(int(mm.sum()), 1)


def cyl_panel(tag, name):
    """圆柱拟合可视化：掩膜点反投影→PCA 轴→底/顶圆+轴投影叠加到帧。"""
    depth, mask = load_npz(tag)
    img = img_of(tag).copy()
    m = best_mask(mask, depth)
    ys, xs = np.where(m & (depth >= 300) & (depth <= 1500))
    if xs.size > 50:
        z = depth[ys, xs] / 1000.0
        pts = np.stack([(xs - CX) * z / FX, (ys - CY) * z / FY, z], 1)
        cen = pts.mean(0)
        _, _, vt = np.linalg.svd(pts - cen, full_matrices=False)
        axis = vt[0]
        if axis[2] < 0:
            axis = -axis
        r = float(np.median(np.linalg.norm(
            (pts - cen) - np.outer((pts - cen) @ axis, axis), axis=1)))
        ext = float(np.abs((pts - cen) @ axis).max())
        e1 = np.cross(axis, [0, 0, 1.0])
        e1 /= (np.linalg.norm(e1) + 1e-9)
        e2 = np.cross(axis, e1)
        for sgn, colr in ((-1, (0, 255, 255)), (+1, (0, 180, 255))):
            c = cen + axis * (sgn * ext)
            circ = np.array([project(c + r * (np.cos(t) * e1 + np.sin(t) * e2))
                             for t in np.linspace(0, 2 * np.pi, 36)])
            cv2.polylines(img, [circ.astype(np.int32)], True, colr, 2, cv2.LINE_AA)
        p0 = project(cen - axis * (ext + 0.05))
        p1 = project(cen + axis * (ext + 0.05))
        cv2.arrowedLine(img, tuple(p0.astype(int)), tuple(p1.astype(int)),
                        (255, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f'PCA axis r={r*1000:.0f}mm h={2*ext*1000:.0f}mm '
                    f'n={xs.size}pts', (8, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
    cv2.rectangle(img, (0, 0), (640, 22), (0, 0, 0), -1)
    cv2.putText(img, f'{name}  cylinder fit (mask pts, frame {FRAME[tag]})',
                (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                cv2.LINE_AA)
    return img


def rough_med(z, mask):
    H, W = z.shape
    xs = (np.arange(W, dtype=np.float32)[None, :] - W / 2) * np.ones((H, W), np.float32)
    ys = (np.arange(H, dtype=np.float32)[:, None] - H / 2) * np.ones((1, W), np.float32)
    basis = [np.ones((H, W), np.float32), xs, ys]
    Bm = np.zeros((H, W, 3, 3), np.float32)
    rhs = np.zeros((H, W, 3), np.float32)
    cnt = np.zeros((H, W), np.int32)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            zs = np.zeros((H, W), np.float32)
            bs = [np.zeros((H, W), np.float32) for _ in range(3)]
            y0, y1 = max(0, dy), H + min(0, dy)
            x0, x1 = max(0, dx), W + min(0, dx)
            zs[y0:y1, x0:x1] = z[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
            for j, bsrc in enumerate(basis):
                bs[j][y0:y1, x0:x1] = bsrc[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
            valid = zs > 0
            phi = np.stack(bs, -1)
            wgt = valid.astype(np.float32)
            Bm += wgt[..., None, None] * phi[..., :, None] * phi[..., None, :]
            rhs += np.where(valid[..., None], wgt[..., None] * phi * zs[..., None], 0)
            cnt += valid
    ok = (z > 0) & (z >= 300) & (z <= 1500) & (cnt >= 6) & (mask > 0)
    if ok.sum() < 10:
        return float('nan'), 0
    coef = np.linalg.solve(Bm[ok], rhs[ok][..., None])[..., 0]
    pred = coef[:, 0] + coef[:, 1] * np.broadcast_to(xs, (H, W))[ok] \
        + coef[:, 2] * np.broadcast_to(ys, (H, W))[ok]
    return float(np.median(np.abs(z[ok] - pred))), int(ok.sum())


def pcq_panel(tag, name):
    depth, mask = load_npz(tag)
    m = best_mask(mask, depth)
    inwin = (depth >= 300) & (depth <= 1500)
    nrm = np.clip((depth - 300) / 1200.0, 0, 1)
    vis = cv2.applyColorMap((nrm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    vis[~inwin] = (40, 40, 40)
    rough, _ = rough_med(depth, m)
    nin = int((m & inwin).sum())
    dens = nin / max(int(m.sum()), 1)
    cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, cnts, -1, (0, 255, 0), 2)
    cv2.rectangle(vis, (0, 0), (640, 22), (0, 0, 0), -1)
    cv2.putText(vis, f'{name}  depth JET + mask', (8, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.rectangle(vis, (0, 458), (640, 480), (0, 0, 0), -1)
    cv2.putText(vis, f'mask pts={nin} dens={dens:.2f} rough={rough:.2f}mm',
                (6, 474), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA)
    return vis, dict(nin=nin, dens=dens, rough=rough)


def main():
    os.makedirs(REPORT, exist_ok=True)
    for tag, name in TAGS:
        rows = jsonl(tag)[8:]
        T = [next(iter(r['targets'].values())) for r in rows if r['targets']]
        ent = np.array([t['entry'] for t in T])
        print(f'== {name} (frame cap {FRAME[tag]}) ==')
        print(f'  检测 {rows[-1]["n_kept"]} 确认 {rows[-1]["n_conf"]} '
              f'conf={np.mean([t["conf"] for t in T]):.3f} '
              f'mdr={np.mean([t["mask_depth_ratio"] for t in T]):.4f}')
        print(f'  entry std[mm]={(ent.std(0)*1000).round(3)} '
              f'jump[mm]={(np.abs(np.diff(ent,axis=0)).mean(0)*1000).round(2)}')
        rad = [t.get('radius_mm', np.nan) for t in T]
        print(f'  半径 {np.nanmean(rad):.1f}±{np.nanstd(rad):.2f}mm')
        med = [t['med'] for t in T if 'med' in t]
        if med:
            print(f'  掩膜中位 {np.mean(med):.1f}±{np.std(med):.2f}mm')
        print(f'  det/seg/geom ms = {np.mean([r["det_ms"] for r in rows]):.0f}/'
              f'{np.mean([r["seg_ms"] for r in rows]):.0f}/'
              f'{np.mean([r["geom_ms"] for r in rows]):.0f}')
        _, q = pcq_panel(tag, name)
        bdepth, bmask = load_npz(tag)
        bz, br = blind_note(bmask, bdepth)
        print(f'  点云质量: 掩膜内点={q["nin"]} 密度={q["dens"]:.3f} '
              f'粗糙度={q["rough"]:.2f}mm')
        print(f'  覆盖缺口: 全图掩膜中无窗内深度像素={bz}（{br:.1%}，'
              f'含左盲带结构性缺口）')
    cv2.imwrite(f'{REPORT}/fig_cyl.png',
                np.hstack([cyl_panel(t, n) for t, n in TAGS]))
    cv2.imwrite(f'{REPORT}/fig_pcq.png',
                np.hstack([pcq_panel(t, n)[0] for t, n in TAGS]))
    print('figs: fig_cyl.png fig_pcq.png')
    return 0


if __name__ == '__main__':
    sys.exit(main())
