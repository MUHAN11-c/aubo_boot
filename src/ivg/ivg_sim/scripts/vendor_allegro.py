#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vendor Allegro Hand（右手）进 ivg_sim（用户指定：换社区常用四指夹爪，
仅仿真）。源：felixduvallet/allegro-hand-ros（SimLab 官方包的社区维护版，
allegro_hand_description；16 revolute 关节、自带 damping/friction）。

产物：
  urdf/allegro_hand.xacro  —— 原 xacro 原样（仅 mesh 路径重写 +
                               robot name 改名避免与外层冲突）
  meshes/allegro/*.STL     —— 原 meshes 全量
用法：python3 scripts/vendor_allegro.py
"""

import re
import shutil
from pathlib import Path

SRC = Path('/tmp/ivg_sim_run/allegro_src/allegro_hand_description')
DST = Path(__file__).resolve().parent.parent


def main():
    text = (SRC / 'allegro_hand_description_right.xacro').read_text(
        encoding='utf-8')
    n_mesh = text.count('package://allegro_hand_description/meshes/')
    text = text.replace('package://allegro_hand_description/meshes/',
                        'package://ivg_sim/meshes/allegro/')
    # Jazzy xacro 要求宏先定义后使用：原文件 finger/thumb 宏在使用点
    # （FINGERS 段）之后定义——把使用段挪到 </robot> 前
    use_start = text.index('    <!-- FINGERS -->')
    use_end = text.index('    <!-- ============================================================================= -->',
                         use_start)
    use_block = text[use_start:use_end]
    text = text[:use_start] + text[use_end:]
    insert_at = text.rindex('</robot>')
    text = text[:insert_at] + use_block + '\n' + text[insert_at:]

    out = DST / 'urdf' / 'allegro_hand.xacro'
    out.write_text(text, encoding='utf-8')

    mesh_dst = DST / 'meshes' / 'allegro'
    mesh_dst.mkdir(parents=True, exist_ok=True)
    n = 0
    for stl in sorted((SRC / 'meshes').glob('*.STL')):
        shutil.copy2(stl, mesh_dst / stl.name)
        n += 1

    joints = sorted(set(re.findall(
        r'<joint name="(joint_\$\{[^}]+\}[^"]*)"', text)))
    print(f'mesh 重写 {n_mesh}，STL 拷贝 {n}（右手件：'
          f'link_12.0_right 等）')
    print('可变关节模板:', joints)
    print('输出:', out)


if __name__ == '__main__':
    main()
