#!/usr/bin/env python3
"""A/B live 数据分析：hh4 vs 对比档 完整感知链对比（读 /tmp/e2e_live 数据）.

输出（cv2.imwrite 到 /tmp/e2e_live/report/，统计打印到 stdout）：
  fig_a_debug.png   两档 debug 叠加图并排（由 make_fig_annot.py 重建版接管）
  fig_b_traces.png  entry x/y/z + 掩膜深度中位 时间轨迹对比（4 面板）
  fig_c_rough.png   袋 ROI 3x3 局部平面粗糙度逐帧轨迹 + 样本 ROI 深度图并排
  fig_d_metrics.png 稳定性指标条形图
  fig_e_rviz.png    rviz 截图（由 make_fig_e_final.py 接管；本文件内为半屏自选版）
本轮最终对比档：hh4 vs percipio2（percipio 原驱动健康档）。
"""
import json
import os
import sys

import cv2
import numpy as np

ROOT = '/tmp/e2e_live'
REPORT = f'{ROOT}/report'
TAGS = [('hh4', 'peach_stereo hh4'), ('percipio2', 'percipio driver (device)')]
WARMUP = 8  # 丢弃确认预热帧
FX, FY, CX, CY = 466.17, 465.56, 326.07, 244.79


def load(tag):
    p = (f'{ROOT}/percipio2.jsonl' if tag == 'percipio2'
         else f'{ROOT}/{tag}/frames.jsonl')
    rows = [json.loads(l) for l in open(p)]
    return rows[WARMUP:]


def tvec(rows):
    """主 target 的 entry/bottom/掩膜统计序列（缺键帧跳过）。"""
    out = dict(t=[], entry=[], bottom=[], med=[], mad=[], n=[], mdr=[],
               radius=[], valid=[], scene_med=[], det=[], seg=[], geom=[])
    for r in rows:
        ts = r.get('targets', {})
        if not ts:
            continue
        t = next(iter(ts.values()))
        out['t'].append(r['t'])
        out['entry'].append(t['entry'])
        out['bottom'].append(t['bottom'])
        out['radius'].append(t.get('radius_mm', np.nan))
        out['mdr'].append(t.get('mask_depth_ratio', np.nan))
        out['valid'].append(r['valid_ratio'])
        out['scene_med'].append(r['depth_med'])
        out['det'].append(r['det_ms'])
        out['seg'].append(r['seg_ms'])
        out['geom'].append(r['geom_ms'])
        if 'med' in t:
            out['med'].append(t['med'])
            out['mad'].append(t['mad'])
            out['n'].append(t['n'])
    return {k: np.array(v, dtype=float) for k, v in out.items()}


def rough_map_med(z, mask, win=(300, 1500)):
    """掩膜内 3x3 局部平面拟合残差中位（mm）。"""
    H, W = z.shape
    r = 1
    xs = (np.arange(W, dtype=np.float32)[None, :] - W / 2) * np.ones((H, W), np.float32)
    ys = (np.arange(H, dtype=np.float32)[:, None] - H / 2) * np.ones((1, W), np.float32)
    basis = [np.ones((H, W), np.float32), xs, ys]
    Bm = np.zeros((H, W, 3, 3), np.float32)
    rhs = np.zeros((H, W, 3), np.float32)
    cnt = np.zeros((H, W), np.int32)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
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
    ok = (z > 0) & (z >= win[0]) & (z <= win[1]) & (cnt >= 6) & (mask > 0)
    if ok.sum() < 10:
        return np.nan, 0
    coef = np.linalg.solve(Bm[ok], rhs[ok][..., None])[..., 0]
    pred = coef[:, 0] + coef[:, 1] * np.broadcast_to(xs, (H, W))[ok] \
        + coef[:, 2] * np.broadcast_to(ys, (H, W))[ok]
    return float(np.median(np.abs(z[ok] - pred))), int(ok.sum())


def roi_rough_series(tag):
    files = sorted(os.listdir(f'{ROOT}/{tag}/npz'))
    vals, sample = [], None
    for fn in files:
        d = np.load(f'{ROOT}/{tag}/npz/{fn}')
        depth = d['depth'].astype(np.float32)
        mask = d['mask']
        if not mask.any():
            continue
        m, n = rough_map_med(depth, mask)
        if np.isfinite(m):
            vals.append(m)
        if sample is None:
            lab = np.bincount(mask.ravel())
            lab[0] = 0
            m0 = mask == lab.argmax()
            ys, xs = np.where(m0)
            y0, y1, x0, x1 = max(ys.min() - 8, 0), min(ys.max() + 8, 480), \
                max(xs.min() - 8, 0), min(xs.max() + 8, 640)
            roi = depth[y0:y1, x0:x1]
            lo, hi = np.percentile(roi[roi > 0], 3), np.percentile(roi[roi > 0], 97)
            nrm = np.clip((roi - lo) / max(hi - lo, 1), 0, 1)
            vis = cv2.applyColorMap((nrm * 255).astype(np.uint8), cv2.COLORMAP_JET)
            vis[roi <= 0] = (40, 40, 40)
            sample = vis
    return np.array(vals), sample


def panel(w, h, title):
    p = np.full((h, w, 3), 245, np.uint8)
    cv2.putText(p, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1,
                cv2.LINE_AA)
    return p


def line_plot(p, series, labels, xlab, ylab, lx=60, rx=20, ty=44, by=36):
    h, w = p.shape[:2]
    cv2.line(p, (lx, h - by), (w - rx, h - by), (0, 0, 0), 1)
    cv2.line(p, (lx, ty), (lx, h - by), (0, 0, 0), 1)
    lo = min(s.min() for s, _ in series)
    hi = max(s.max() for s, _ in series)
    pad = (hi - lo) * 0.08 + 1e-9
    lo, hi = lo - pad, hi + pad
    for (s, c), lab in zip(series, labels):
        xs = np.linspace(lx + 4, w - rx - 4, len(s))
        ys = (h - by - 6) - (s - lo) / (hi - lo) * (h - by - ty - 16)
        cv2.polylines(p, [np.stack([xs, ys], 1).astype(np.int32)], False, c, 2,
                      cv2.LINE_AA)
    for i, ((s, c), lab) in enumerate(zip(series, labels)):
        cv2.rectangle(p, (lx + 8 + i * 190, ty - 12), (lx + 20 + i * 190, ty),
                      c, -1)
        cv2.putText(p, lab, (lx + 24 + i * 190, ty - 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(p, f'{hi:.1f}', (6, ty + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (60, 60, 60), 1, cv2.LINE_AA)
    cv2.putText(p, f'{lo:.1f}', (6, h - by + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (60, 60, 60), 1, cv2.LINE_AA)
    cv2.putText(p, xlab, (w - rx - 60, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (60, 60, 60), 1, cv2.LINE_AA)
    cv2.putText(p, ylab, (lx + 4, ty - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (60, 60, 60), 1, cv2.LINE_AA)


def bar_chart(title, pairs, unit, ymax, colr=(80, 120, 220)):
    p = panel(640, 300, f'{title}  [{unit}]')
    for i, (name, val) in enumerate(pairs):
        bh = int(min(val, ymax) / ymax * 190)
        x0 = 60 + i * 280
        cv2.rectangle(p, (x0, 250 - bh), (x0 + 220, 250), colr, -1)
        cv2.putText(p, f'{val:.3f}', (x0 + 60, 250 - bh - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(p, name, (x0 + 40, 275), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 1, cv2.LINE_AA)
    cv2.line(p, (50, 250), (630, 250), (0, 0, 0), 1)
    return p


def main():
    os.makedirs(REPORT, exist_ok=True)
    data = {}
    for tag, name in TAGS:
        rows = load(tag)
        d = tvec(rows)
        rough, sample = roi_rough_series(tag)
        d['rough'] = rough
        data[tag] = (rows, d, sample)
        n_frames = len(rows) + WARMUP
        dt = rows[-1]['t'] - rows[0]['t']
        e = np.array(d['entry'])
        print(f'== {name} ==')
        print(f'  frames={n_frames} rate={len(rows)/dt:.2f}Hz '
              f'(warmup 丢弃 {WARMUP})')
        print(f'  entry mean [m]={e.mean(0).round(4)}  '
              f'std [mm]={(e.std(0)*1000).round(3)}')
        print(f'  entry 帧间跳变 |Δ| 中位 [mm]='
              f'{(np.abs(np.diff(e, axis=0)).mean(0)*1000).round(2)}')
        if len(d['med']):
            print(f"  掩膜深度中位 mean={d['med'].mean():.1f}mm "
                  f"std={d['med'].std():.2f}mm")
            print(f"  掩膜 MAD mean={d['mad'].mean():.2f}mm "
                  f"(含袋形高度)  mask n={d['n'].mean():.0f}px")
            print(f"  mask_depth_ratio={np.nanmean(d['mdr']):.4f}")
        print(f"  ROI 3x3 粗糙度 mean={np.nanmean(rough):.2f}mm "
              f"unique={np.unique(np.round(rough, 2))[:6]} "
              f"(n={len(rough)})")
        print(f"  valid_ratio={d['valid'].mean():.4f}  "
              f"scene_med std={d['scene_med'].std():.2f}mm")
        print(f"  det/seg/geom ms = {d['det'].mean():.0f}/{d['seg'].mean():.0f}/"
              f"{d['geom'].mean():.0f}")

    CA, CB = (60, 160, 60), (60, 110, 240)

    # fig_a debug 并排（由 make_fig_annot.py 重建版生成，此处保留原图拷贝逻辑）
    dbg = {}
    for tag, _ in TAGS:
        fs = sorted(os.listdir(f'{ROOT}/{tag}/img'))
        dbg[tag] = cv2.imread(f'{ROOT}/{tag}/img/{fs[-1]}') if fs else None
    if all(v is not None for v in dbg.values()):
        ps = []
        for (tag, name), key in zip(TAGS, ['hh4', 's3way']):
            im = cv2.resize(dbg[tag], (640, 480)).copy()
            cv2.rectangle(im, (0, 0), (640, 40), (0, 0, 0), -1)
            cv2.putText(im, f'{name}  detection+mask+axis (last frame)', (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            ps.append(im)
        cv2.imwrite(f'{REPORT}/fig_a_raw.png', np.hstack(ps))

    # fig_b 轨迹
    panels = []
    _, da, _ = data['hh4']
    _, db, _ = data['percipio2']
    for i, ax in enumerate('xyz'):
        p = panel(640, 240, f'entry {ax} [mm] vs time')
        sa = da['entry'][:, i] * 1000
        sb = db['entry'][:, i] * 1000
        line_plot(p, [(sa, CA), (sb, CB)], ['hh4', 'percipio'], 't [s]',
                  f'{ax} [mm]')
        panels.append(p)
    p = panel(640, 240, 'mask depth median [mm] vs time')
    line_plot(p, [(da['med'], CA), (db['med'], CB)], ['hh4', 'percipio'], 't [s]',
              'mm')
    panels.append(p)
    gap = np.full((4, 640, 3), 255, np.uint8)
    fig = panels[0]
    for pn in panels[1:]:
        fig = np.vstack([fig, gap, pn])
    cv2.imwrite(f'{REPORT}/fig_b_traces.png', fig)

    # fig_c 粗糙度
    p = panel(640, 260, 'bag-ROI local plane roughness [mm] per saved frame')
    line_plot(p, [(da['rough'], CA), (db['rough'], CB)], ['hh4', 'percipio'],
              'npz frame #', 'mm')
    samples = [data[t][2] for t, _ in TAGS]
    if all(s is not None for s in samples):
        ps = []
        for s, (_, name) in zip(samples, TAGS):
            im = cv2.resize(s, (320, 240)).copy()
            cv2.rectangle(im, (0, 0), (320, 36), (0, 0, 0), -1)
            cv2.putText(im, name, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
            ps.append(im)
        p = np.vstack([p, gap, np.hstack(ps)])
    cv2.imwrite(f'{REPORT}/fig_c_rough.png', p)

    # fig_d 指标条形
    ea, eb = da['entry'], db['entry']
    charts = [
        ('entry z std', [('hh4', ea[:, 2].std() * 1000),
                         ('percipio', eb[:, 2].std() * 1000)], 'mm',
         max(ea[:, 2].std(), eb[:, 2].std()) * 1000 * 1.3 + 0.01),
        ('entry lateral std (sqrt x^2+y^2))',
         [('hh4', float(np.hypot(ea[:, 0], ea[:, 1]).std() * 1000)),
          ('percipio', float(np.hypot(eb[:, 0], eb[:, 1]).std() * 1000))], 'mm',
         max(np.hypot(ea[:, 0], ea[:, 1]).std(), np.hypot(eb[:, 0], eb[:, 1]).std())
         * 1000 * 1.3 + 0.01),
        ('mask depth median std',
         [('hh4', float(da['med'].std())), ('percipio', float(db['med'].std()))],
         'mm', max(da['med'].std(), db['med'].std()) * 1.3 + 0.01),
        ('ROI plane roughness (mean)',
         [('hh4', float(np.nanmean(da['rough']))),
          ('percipio', float(np.nanmean(db['rough'])))], 'mm',
         float(np.nanmax([np.nanmean(da['rough']), np.nanmean(db['rough'])])) * 1.3),
    ]
    fig = bar_chart(*charts[0])
    for c in charts[1:]:
        fig = np.vstack([fig, gap, bar_chart(*c)])
    cv2.imwrite(f'{REPORT}/fig_d_metrics.png', fig)

    print('\nfigs written:', sorted(os.listdir(REPORT)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
