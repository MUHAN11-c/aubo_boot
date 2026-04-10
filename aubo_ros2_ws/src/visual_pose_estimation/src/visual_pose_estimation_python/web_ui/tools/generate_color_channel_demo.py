#!/usr/bin/env python3
"""生成一个 HTML 页面，用于手工确认 OpenCV BGR/RGB 颜色通道表现。"""

from __future__ import annotations

import base64
from pathlib import Path

import cv2
import numpy as np


img = np.zeros((200, 600, 3), dtype=np.uint8)

cv2.rectangle(img, (10, 10), (190, 190), (0, 255, 0), -1)
cv2.putText(img, "GREEN", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

cv2.rectangle(img, (210, 10), (390, 190), (255, 0, 0), -1)
cv2.putText(img, "BLUE", (250, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

cv2.rectangle(img, (410, 10), (590, 190), (0, 0, 255), -1)
cv2.putText(img, "RED", (450, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

_, buffer1 = cv2.imencode(".jpg", img)
base64_bgr = "data:image/jpeg;base64," + base64.b64encode(buffer1).decode("utf-8")

img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
_, buffer2 = cv2.imencode(".jpg", img_rgb)
base64_rgb = "data:image/jpeg;base64," + base64.b64encode(buffer2).decode("utf-8")

html = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <title>颜色通道测试</title>
    <style>
        body {{ font-family: sans-serif; margin: 20px; }}
        .test {{ margin: 20px 0; border: 2px solid #ccc; padding: 10px; }}
        img {{ max-width: 100%; }}
    </style>
</head>
<body>
    <h1>OpenCV 颜色通道测试</h1>
    <p>正确显示应该是：绿色 - 蓝色 - 红色。</p>

    <div class="test">
        <h2>测试1: 直接编码 BGR</h2>
        <p>如果网页显示异常颜色，说明输出前需要做 BGR -&gt; RGB 转换。</p>
        <img src="{base64_bgr}" />
    </div>

    <div class="test">
        <h2>测试2: BGR -&gt; RGB 转换后编码</h2>
        <p>这一栏应该显示正确顺序：绿色 - 蓝色 - 红色。</p>
        <img src="{base64_rgb}" />
    </div>
    </body>
</html>
"""

output_file = Path(__file__).with_name("test_color.html")
output_file.write_text(html, encoding="utf-8")

print(f"✓ 测试页面已生成: {output_file}")
print(f"  在浏览器中打开查看: file://{output_file}")
