#!/usr/bin/env python3
"""测试颜色通道转换"""

import cv2
import numpy as np
import base64

# 创建测试图像
img = np.zeros((200, 600, 3), dtype=np.uint8)

# BGR格式绘制（OpenCV默认）
cv2.rectangle(img, (10, 10), (190, 190), (0, 255, 0), -1)  # 绿色
cv2.putText(img, 'GREEN', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

cv2.rectangle(img, (210, 10), (390, 190), (255, 0, 0), -1)  # 蓝色
cv2.putText(img, 'BLUE', (250, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

cv2.rectangle(img, (410, 10), (590, 190), (0, 0, 255), -1)  # 红色
cv2.putText(img, 'RED', (450, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

# 测试1: 直接编码BGR
_, buffer1 = cv2.imencode('.jpg', img)
base64_bgr = "data:image/jpeg;base64," + base64.b64encode(buffer1).decode('utf-8')

# 测试2: BGR转RGB后编码
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
_, buffer2 = cv2.imencode('.jpg', img_rgb)
base64_rgb = "data:image/jpeg;base64," + base64.b64encode(buffer2).decode('utf-8')

# 生成HTML测试页面
html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>颜色通道测试</title>
    <style>
        body {{ font-family: Arial; margin: 20px; }}
        .test {{ margin: 20px 0; border: 2px solid #ccc; padding: 10px; }}
        img {{ max-width: 100%; }}
    </style>
</head>
<body>
    <h1>OpenCV颜色通道测试</h1>
    <p>第一个应该显示：绿色-蓝色-红色</p>
    
    <div class="test">
        <h2>测试1: 直接编码BGR（错误）</h2>
        <p>如果网页显示：<strong>蓝色-红色-绿色</strong>，说明需要BGR->RGB转换</p>
        <img src="{base64_bgr}" />
    </div>
    
    <div class="test">
        <h2>测试2: BGR->RGB转换后编码（正确）</h2>
        <p>应该显示：<strong>绿色-蓝色-红色</strong></p>
        <img src="{base64_rgb}" />
    </div>
    
    <div class="test">
        <h3>说明</h3>
        <ul>
            <li>OpenCV内部使用BGR格式（蓝-绿-红）</li>
            <li>网页/JPEG期望RGB格式（红-绿-蓝）</li>
            <li>需要在编码前转换：cv2.cvtColor(img, cv2.COLOR_BGR2RGB)</li>
        </ul>
    </div>
</body>
</html>
"""

output_file = 'test_color.html'
with open(output_file, 'w') as f:
    f.write(html)

print(f"✓ 测试页面已生成: {output_file}")
print(f"  在浏览器中打开查看: file://{output_file}")
