#!/usr/bin/env python3
"""fig_e：自两段 rviz 窗口视频各抽一帧，等高并排（带标题条）.

2026-09-21 重录轮启用（替代截图法 make_fig_e_final.py）：窗录视频本身就是
采集期实录，抽帧即得；帧号取视频中段（数据仍在流动的窗口内）。
"""
import subprocess

import cv2
import numpy as np

ROOT = '/tmp/e2e_live'
REPORT = f'{ROOT}/report'
PANELS = [
    (f'{REPORT}/hh4_composite.mp4', 300, 'peach_stereo hh4 (annotated: box|mask|axis)'),
    (f'{REPORT}/percipio2_composite.mp4', 300, 'percipio driver (annotated: box|mask|axis)'),
]


def grab(video, frame_idx):
    out = f'/tmp/e2e_live/_fige_{frame_idx}.png'
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', video,
                    '-vf', f'select=eq(n\\,{frame_idx})', '-frames:v', '1',
                    '-update', '1', out], check=True)
    return cv2.imread(out)


def main():
    outs = []
    for video, idx, title in PANELS:
        im = grab(video, idx)
        crop = im
        crop = cv2.resize(crop, (1100, int(crop.shape[0] * 1100 / crop.shape[1])))
        h, w = crop.shape[:2]
        canvas = np.full((h + 44, w, 3), 20, np.uint8)
        canvas[44:, :] = crop
        cv2.rectangle(canvas, (0, 0), (w, 40), (0, 0, 0), -1)
        cv2.putText(canvas, f'rviz2  {title}', (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 220, 80), 2, cv2.LINE_AA)
        outs.append(canvas)
    cv2.imwrite(f'{REPORT}/fig_e_rviz.png', np.hstack(outs))
    print('fig_e_rviz.png written')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
