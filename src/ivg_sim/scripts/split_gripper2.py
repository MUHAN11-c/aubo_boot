#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 vendored gripper2（电动夹爪 φ60，四指十字排布、静态闭态网格）按指剖开.

剖分面 z=0.078（基座实心区上界）；指块按三角形质心方位角归属四象限
（|x|≥|y| → x±，否则 y±）。指块局部坐标 = 全局 − (0,0,0.078)
（即各指 prismatic 关节 origin 取 (0,0,0.078)，轴=各自径向单位向量）。

产物：src/ivg_sim/meshes/{visual,collision}/g2_{base,finger_xp,xn,yp,yn}.stl
用法：python3 scripts/split_gripper2.py [--check]（--check 只打印统计不落盘）
"""

import struct
import sys
from pathlib import Path

import numpy as np

SRC = Path('/home/mu/Desktop/aubo_e5_jazzy_ws/src/aubo_description/'
           'meshes')
DST = Path(__file__).resolve().parent.parent / 'meshes'
SPLIT_Z = 0.078
FINGERS = ('xp', 'xn', 'yp', 'yn')


def load_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        tris = np.empty((n, 3, 3), dtype='<f4')
        for i in range(n):
            data = f.read(50)
            v = struct.unpack('<9f', data[12:48])
            tris[i] = np.array(v, dtype='<f4').reshape(3, 3)
        return tris


def save_stl(path, tris):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'wb') as f:
        f.write(b'ivg_sim split_gripper2'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            e1 = t[1] - t[0]
            e2 = t[2] - t[0]
            n = np.cross(e1, e2)
            norm = np.linalg.norm(n)
            n = n / norm if norm > 0 else n
            f.write(struct.pack('<3f', *n.astype('<f4')))
            for v in t:
                f.write(struct.pack('<3f', *v.astype('<f4')))
            f.write(b'\0\0')


def finger_id(centroid):
    if abs(centroid[0]) >= abs(centroid[1]):
        return 'xp' if centroid[0] >= 0 else 'xn'
    return 'yp' if centroid[1] >= 0 else 'yn'


def split(kind):
    tris = load_stl(SRC / kind / 'gripper2_link.stl')
    groups = {'base': []}
    for name in FINGERS:
        groups[name] = []
    for t in tris:
        c = t.mean(axis=0)
        if c[2] < SPLIT_Z:
            groups['base'].append(t)
        else:
            groups[finger_id(c)].append(t)
    print('[%s]' % kind)
    for name, g in groups.items():
        if not g:
            raise SystemExit(f'剖分异常：{kind}/{name} 空块')
        arr = np.array(g)
        if name != 'base':
            arr = arr - np.array([0.0, 0.0, SPLIT_Z], dtype='<f4')
        if '--check' not in sys.argv:
            save_stl(DST / kind / f'g2_{name}.stl', arr)
        pts = arr.reshape(-1, 3)
        print('  %-6s %5d tris  局部z %.3f..%.3f  xy(%.0f,%.0f)..(%.0f,%.0f) mm'
              % (name, len(g), pts[:, 2].min(), pts[:, 2].max(),
                 pts[:, 0].min() * 1e3, pts[:, 1].min() * 1e3,
                 pts[:, 0].max() * 1e3, pts[:, 1].max() * 1e3))
    total = sum(len(g) for g in groups.values())
    if total != len(tris):
        raise SystemExit('三角形丢失')


for kind in ('visual', 'collision'):
    split(kind)
print('完成' if '--check' not in sys.argv else 'check 模式（未落盘）')
