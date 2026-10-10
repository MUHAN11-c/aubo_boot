#!/usr/bin/env python3
"""录制自检门：composite 窗帧里必须看到检测框与分割轮廓.

判据（与 debug_draw.py 的绘制色一致，BGR）：
  绿框 (0,220,0) / 橙框 (0,180,255)——确认目标检测框，2px
  红轮廓 (80,80,230)——SAM 分割轮廓，2px
阈值取宽松窗（x11grab→png 无损，色偏极小）。exit 0=过门。
"""
import sys

import cv2
import numpy as np


def main():
    img = cv2.imread(sys.argv[1])
    if img is None:
        print('GATE: FAIL — cannot read frame')
        return 1
    b = img[..., 0].astype(int)
    g = img[..., 1].astype(int)
    r = img[..., 2].astype(int)
    green = int(((g > 170) & (r < 100) & (b < 100)).sum())
    orange = int(((b < 80) & (g > 150) & (r > 200)).sum())
    red = int(((r > 190) & (b > 40) & (b < 130) & (g > 40) & (g < 130)).sum())
    print(f'GATE: green_box={green} orange_box={orange} mask_contour={red}')
    ok = (green + orange) >= 200 and red >= 200
    print('GATE: PASS（框/掩膜可见）' if ok else 'GATE: FAIL — 框/掩膜不可见')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
