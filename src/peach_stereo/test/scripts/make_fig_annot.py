#!/usr/bin/env python3
"""重建 fig_a：debug 图因 publish_debug_image=false 存的是纯原图——
用 npz 实例掩膜（像素值=检测序号+1）+ jsonl 位姿离线重画标注：
  每个检测：掩膜轮廓(绿) + 外接矩形框(红) + 序号
  已确认目标（掩膜面积与 jsonl 记录 n 最接近的实例）：entry 投影点(黄) + entry→bottom 轴线(黄) + 统计文本
取两档共同帧号 121（img %15==1 与 npz %10==1 的交集）。
注意：本档案副本的读取路径改为本目录 ../data/；原始运行时读 /tmp/e2e_live。
"""
import json
import os
import sys

import cv2
import numpy as np

ROOT = os.environ.get('E2E_ROOT', '/tmp/e2e_live')
REPORT = f'{ROOT}/report'
FRAME = int(os.environ.get('E2E_FRAME', '121'))
FX, FY, CX, CY = 466.17, 465.56, 326.07, 244.79
TAGS = [('hh4', 'peach_stereo hh4'), ('percipio2', 'percipio driver (device)')]


def jsonl_path(tag):
    return (f'{ROOT}/percipio2.jsonl' if tag == 'percipio2'
            else f'{ROOT}/{tag}/frames.jsonl')


def nearest_target_row(tag, fno):
    rows = [json.loads(l) for l in open(jsonl_path(tag))]
    rows = [r for r in rows if r['n_proc'] == fno and r['targets']]
    return rows[0] if rows else None


def project(p):
    u = FX * p[0] / p[2] + CX
    v = FY * p[1] / p[2] + CY
    return int(round(u)), int(round(v))


def annotate(tag, name):
    img = cv2.imread(f'{ROOT}/{tag}/img/debug_{FRAME:04d}.png')
    if img is None:  # 该帧未存图则取最近存档（img 每 15 帧存、npz 每 10 帧存）
        fs = sorted(os.listdir(f'{ROOT}/{tag}/img'))
        img = cv2.imread(f'{ROOT}/{tag}/img/{fs[-1]}')
    d = np.load(f'{ROOT}/{tag}/npz/frame_{FRAME:04d}.npz')
    mask = d['mask']
    depth = d['depth'].astype(np.float32)
    row = nearest_target_row(tag, FRAME)
    out = img.copy()

    # 每个实例掩膜：轮廓 + 外接框
    areas = {}
    for lab in range(1, int(mask.max()) + 1):
        m = mask == lab
        n = int(m.sum())
        if n < 50:
            continue
        areas[lab] = n
        ys, xs = np.where(m)
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        cv2.rectangle(out, (x0, y0), (x1, y1), (0, 0, 255), 2)
        cv2.drawContours(out, [np.stack([xs, ys], 1).reshape(-1, 1, 2).astype(np.int32)],
                         -1, (0, 255, 0), 1)
        conf_txt = ''
        mz = depth[m]
        mz = mz[(mz >= 300) & (mz <= 1500)]
        if mz.size:
            conf_txt = f' z~{np.median(mz):.0f}mm'
        cv2.putText(out, f'#{lab}{conf_txt}', (x0, max(y0 - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    # 已确认目标 = 掩膜面积与 jsonl 记录 n 最接近的实例（袋 #2 而非最大掩膜）
    if areas and row:
        t = next(iter(row['targets'].values()))
        n_px = t.get('n')
        lab = (min(areas, key=lambda k: abs(areas[k] - n_px))
               if n_px else max(areas, key=areas.get))
        e = project(t['entry'])
        b = project(t['bottom'])
        cv2.circle(out, e, 5, (0, 255, 255), -1)
        cv2.line(out, e, b, (0, 255, 255), 2)
        cv2.putText(out, 'entry', (e[0] + 8, e[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 1, cv2.LINE_AA)
        txt = (f"confirmed: conf={t['conf']:.2f} mdr={t['mask_depth_ratio']:.3f} "
               f"z={t['entry'][2]:.3f}m r={t.get('radius_mm', 0):.0f}mm")
        cv2.rectangle(out, (0, 458), (640, 480), (0, 0, 0), -1)
        cv2.putText(out, txt, (6, 474), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(out, f'mask#{lab} n={areas[lab]}px (green=mask contour, red=bbox, yellow=entry/axis)',
                    (6, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.rectangle(out, (0, 0), (640, 12), (0, 0, 0), -1)
    cv2.putText(out, f'{name}  frame {FRAME} (rebuilt from archived mask+pose)',
                (6, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)
    return out


def main():
    os.makedirs(REPORT, exist_ok=True)
    ps = [annotate(tag, name) for tag, name in TAGS]
    cv2.imwrite(f'{REPORT}/fig_a_debug.png', np.hstack(ps))
    print('rebuilt fig_a_debug.png')
    return 0


if __name__ == '__main__':
    sys.exit(main())
