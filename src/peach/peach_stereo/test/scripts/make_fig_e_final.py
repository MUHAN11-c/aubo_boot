#!/usr/bin/env python3
"""最终 fig_e：两张已验证的 rviz2 窗口截图等宽并排（各自经视觉复核含彩色点云）.

左 = peach_stereo hh4（固定几何窗口 live 截图，采集期 t26）
右 = percipio 原驱动（percipio2 健康档采集期 t45 截图，视觉定位窗口裁剪）
"""
import cv2
import numpy as np

ROOT = '/tmp/e2e_live'
REPORT = f'{ROOT}/report'
PANELS = [
    (f'{ROOT}/hh4_rviz_fixed.png', (120, 920, 360, 1560),
     'peach_stereo hh4 (host SGBM, 13.8gps)'),
    (f'{ROOT}/percipio_rviz_fixed.png', (120, 920, 360, 1560),
     'percipio driver (device 18-pattern, healthy)'),
]


def main():
    outs = []
    for path, (y0, y1, x0, x1), title in PANELS:
        im = cv2.imread(path)
        crop = im[y0:y1, x0:x1]
        crop = cv2.resize(crop, (960, int(crop.shape[0] * 960 / crop.shape[1])))
        h, w = crop.shape[:2]
        canvas = np.full((620, 960, 3), 20, np.uint8)
        take = min(h, 560)
        off = 44 if h > 560 else 44 + (560 - h) // 2
        canvas[off:off + take, :w] = crop[:take]
        cv2.rectangle(canvas, (0, 0), (960, 40), (0, 0, 0), -1)
        cv2.putText(canvas, f'rviz2  {title}', (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 220, 80), 2, cv2.LINE_AA)
        outs.append(canvas)
    cv2.imwrite(f'{REPORT}/fig_e_rviz.png', np.hstack(outs))
    print('final fig_e written')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
