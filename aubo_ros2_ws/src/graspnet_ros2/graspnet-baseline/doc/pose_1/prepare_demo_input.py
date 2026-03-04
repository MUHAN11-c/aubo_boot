""" 将 pose_1 下文件整理为 demo 所需格式：color.png, depth.png, workspace_mask.png, meta.mat """
import os
import re
import numpy as np
import scipy.io as scio
from PIL import Image

DIR = os.path.dirname(os.path.abspath(__file__))

# 1. color.png <- original_image.jpg
jpg_path = os.path.join(DIR, 'original_image.jpg')
color_path = os.path.join(DIR, 'color.png')
if os.path.exists(jpg_path):
    img = Image.open(jpg_path)
    if img.mode != 'RGB':
        img = img.convert('RGB')
    img.save(color_path)
    print('Created:', color_path)
else:
    print('Skip color.png: not found', jpg_path)

# 2. depth.png <- depth_image.png (复制/重命名)
depth_src = os.path.join(DIR, 'depth_image.png')
depth_dst = os.path.join(DIR, 'depth.png')
if os.path.exists(depth_src):
    Image.open(depth_src).save(depth_dst)
    print('Created:', depth_dst)
else:
    print('Skip depth.png: not found', depth_src)

# 3. workspace_mask.png：若与 depth 尺寸不一致则缩放到与 depth 一致
depth_dst = os.path.join(DIR, 'depth.png')
ws_path = os.path.join(DIR, 'workspace_mask.png')
if os.path.exists(depth_dst) and os.path.exists(ws_path):
    d_img = Image.open(depth_dst)
    w_img = Image.open(ws_path)
    if d_img.size != w_img.size:
        w_img = w_img.resize(d_img.size, Image.NEAREST)
        w_img.save(ws_path)
        print('Resized workspace_mask.png to match depth:', d_img.size)

# 4. meta.mat <- calibration_results/*.yaml 中的 camera_matrix（不依赖 PyYAML）
def read_intrinsic_from_yaml(yaml_path):
    with open(yaml_path, 'r', encoding='utf-8') as f:
        text = f.read()
    # 找到 camera_matrix: 段（排除 camera_matrix_info），收集其后 9 个 float 组成 3x3
    idx = text.find('camera_matrix:')
    if idx < 0:
        return None
    rest = text[idx + len('camera_matrix:'):]
    nums = re.findall(r'[-]?\d+\.?\d*', rest)
    floats = []
    for s in nums:
        try:
            floats.append(float(s))
            if len(floats) >= 9:
                break
        except ValueError:
            pass
    if len(floats) >= 9:
        return np.array(floats[:9], dtype=np.float64).reshape(3, 3)
    return None

cal_dir = os.path.join(DIR, 'calibration_results')
yaml_path = None
if os.path.isdir(cal_dir):
    for f in sorted(os.listdir(cal_dir)):
        if f.endswith('.yaml'):
            yaml_path = os.path.join(cal_dir, f)
            break
if yaml_path and os.path.exists(yaml_path):
    intrinsic = read_intrinsic_from_yaml(yaml_path)
    if intrinsic is not None:
        # 深度图单位：0.25mm（与 visual_pose_estimation_python 一致）
        # depth_scale = 0.00025，即 depth_raw * 0.00025 = 米
        # factor_depth = 1 / depth_scale = 4000
        factor_depth = 4000.0  # 深度图单位：0.25mm -> 米
        scio.savemat(os.path.join(DIR, 'meta.mat'), {
            'intrinsic_matrix': intrinsic,
            'factor_depth': np.array([factor_depth])
        })
        print('Created: meta.mat (from %s, factor_depth=4000)' % os.path.basename(yaml_path))
    else:
        print('Skip meta.mat: could not parse camera_matrix from', yaml_path)
else:
    print('Skip meta.mat: no calibration yaml in', cal_dir)
