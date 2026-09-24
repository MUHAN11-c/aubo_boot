#!/usr/bin/env python3
"""室外套袋桃园 Gazebo(Harmonic) 场景生成器 — 纯核、零 ROS 依赖。

一次运行产出（默认相对 orchard_scene/ 根）：

  worlds/bagged_peach_orchard.sdf       几何版世界：纯 SDF 基元，零外部依赖
  worlds/bagged_peach_orchard_hd.sdf    渲染版世界：视觉换 model:// OBJ 网格，
                                        碰撞/布局/袋位与几何版完全一致（同 seed）
  models/orchard_meshes/                HD 网格资产（tools/meshgen.py 产出）
  data/orchard_targets.json             每袋世界系目标几何，字段语义与
                                        src/peach_arm/test/fixtures/field_pregrasp_cases.yaml
                                        的 targets_20260909 一致：
                                        entry_xyz=袋底=工具入口，axis=袋底→袋颈(单位)，
                                        bag_neck=entry_xyz+axis*travel_m
  preview/orchard_layout.png            布局预览（3D/俯视/侧视/axis_z 统计）
  preview/orchard_hd_render.png         HD 网格软件渲染（真实 OBJ 三角面 + 简单光照）

场景：行距 4.0 m、株距 2.6 m 的三主枝自然开心形桃树；袋口系在结果枝梢、
袋底垂在下方，axis_z≥0.70 为主流包络，另留 ~5% 近水平袋作压测（对齐现场
1021_1）。全部 static；每袋独立 link `bag_r{行}_t{株}_b{序}` 与 JSON id 对应。

用法：
  python3 tools/generate_orchard_world.py                    # 默认种子 20260923
  python3 tools/generate_orchard_world.py --seed 7 --rows 3 --trees 6
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import meshgen  # noqa: E402

# ---------- 布局（单位 m，REP-103；x 沿行向，y 跨行，z 向上） ----------
ROW_SPACING = 4.0
TREE_SPACING = 2.6
HEADLAND_M = 3.0
GROUND_SIZE = (40.0, 30.0)
SOIL_WIDTH = 1.4
POST_R, POST_H = 0.05, 2.0

# ---------- 树形：三主枝自然开心形 ----------
TRUNK_H = (0.55, 0.75)
TRUNK_R = (0.08, 0.10)
LEADER_N = 3
LEADER_TILT_DEG = (38.0, 50.0)   # 与铅垂夹角
LEADER_LEN = (1.10, 1.50)
LEADER_R = 0.05
BRANCH_PER_LEADER = 2
BRANCH_LEN = (0.50, 0.90)
BRANCH_R = 0.025
TWIG_R = 0.006
TWIG_LEN = (0.12, 0.25)
CANOPY_R = (0.45, 0.70)

# ---------- 套袋果 ----------
BAGS_PER_TREE = (10, 14)
BAG_PAPER_R = 0.052              # 纸身半径
BAG_PAPER_LEN = 0.104            # 袋口(原点)到袋底 = 2r；即 targets 的 travel_m
TWIST_R, TWIST_LEN = 0.008, 0.026
HARVEST_Z = (0.80, 1.90)         # 冠内作业高度带
TILT_MAIN_DEG = (5.0, 44.0)      # axis_z ∈ [cos44°, 1) ≈ [0.72, 1)
TILT_HORIZ_DEG = (60.0, 80.0)    # 近水平压测袋（对齐现场 1021_1）
HORIZONTAL_FRAC = 0.05
BAG_MIN_DIST = 0.20              # 同树袋口最小间距

# ---------- 配色 ----------
COL_GRASS = (0.32, 0.46, 0.20)
COL_SOIL = (0.42, 0.33, 0.22)
COL_TRUNK = (0.46, 0.34, 0.22)
COL_BRANCH = (0.50, 0.38, 0.24)
COL_TWIG = (0.55, 0.45, 0.28)
COL_LEAF = ((0.20, 0.45, 0.15), (0.26, 0.52, 0.18))
COL_BAG = (0.82, 0.64, 0.30)
COL_TWIST = (0.55, 0.20, 0.15)
COL_WOOD = (0.58, 0.46, 0.30)

MODEL_URI = 'model://orchard_meshes/meshes'
SEED = 20260923


# ================= 姿态数学（自实现，零依赖） =================
def quat_z_to(d):
    """[0,0,1] 旋转到单位向量 d 的四元数 (qw,qx,qy,qz)。"""
    dot = max(-1.0, min(1.0, d[2]))
    half = 0.5 * math.acos(dot)
    s = math.sin(half)
    nx, ny = -d[1], d[0]
    n = math.hypot(nx, ny)
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0) if dot > 0.0 else (0.0, 1.0, 0.0, 0.0)
    return (math.cos(half), nx / n * s, ny / n * s, 0.0)


def quat_to_rpy(q):
    """四元数 → SDF 默认欧拉角 (roll, pitch, yaw)。"""
    qw, qx, qy, qz = q
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return (roll, pitch, yaw)


def rpy_to_dir(rpy):
    """欧拉角 → 单位 +x 方向（仅用于相机/诊断，标准 ZYX 外旋）。"""
    r, p, y = rpy
    return (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p))


def fmt(*vals):
    return ' '.join(f'{v:.4f}' for v in vals)


def pose_of(xyz, rpy=(0.0, 0.0, 0.0)):
    return f'{fmt(*xyz)} {fmt(*rpy)}'


def cyl_rpy(p0, p1):
    """p0→p1 圆柱（SDF 圆柱沿 +z）的位姿：中点 + 对准欧拉角。"""
    d = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2)
    return mid, quat_to_rpy(quat_z_to((d[0] / length, d[1] / length, d[2] / length))), length


def mat(color, ambient_scale=0.5):
    r, g, b = color
    return (f'<ambient>{r * ambient_scale:.4f} {g * ambient_scale:.4f} '
            f'{b * ambient_scale:.4f} 1</ambient>'
            f'<diffuse>{r:.4f} {g:.4f} {b:.4f} 1</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular>')


# ================= 场景生成（纯数据，供双档发射器与预览共用） =================
def gen_tree(rng, row, ti, tx, ty):
    """单棵开心形桃树的几何数据 + 挂袋结果。"""
    lean_az = rng.uniform(0.0, 2.0 * math.pi)
    lean = math.radians(rng.uniform(0.0, 3.0))
    h = rng.uniform(*TRUNK_H)
    r_tr = rng.uniform(*TRUNK_R)
    base = (tx, ty, 0.0)
    top = (tx + h * math.sin(lean) * math.cos(lean_az),
           ty + h * math.sin(lean) * math.sin(lean_az),
           h * math.cos(lean))

    leaders, branches, twigs, blobs, anchors = [], [], [], [], []
    for k in range(LEADER_N):
        az = math.radians(120.0 * k + rng.uniform(-12.0, 12.0))
        tilt = math.radians(rng.uniform(*LEADER_TILT_DEG))
        length = rng.uniform(*LEADER_LEN)
        d = (math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az), math.cos(tilt))
        p0, p1 = top, (top[0] + length * d[0], top[1] + length * d[1], top[2] + length * d[2])
        leaders.append((p0, p1, LEADER_R))
        anchors.append(p1)
        for _ in range(BRANCH_PER_LEADER):
            t0 = rng.uniform(0.50, 0.80)
            s = (p0[0] + (p1[0] - p0[0]) * t0,
                 p0[1] + (p1[1] - p0[1]) * t0,
                 p0[2] + (p1[2] - p0[2]) * t0)
            baz = az + math.radians(rng.uniform(-55.0, 55.0))
            el = math.radians(rng.uniform(-30.0, -5.0))
            blen = rng.uniform(*BRANCH_LEN)
            e = (s[0] + blen * math.cos(el) * math.cos(baz),
                 s[1] + blen * math.cos(el) * math.sin(baz),
                 s[2] + blen * math.sin(el))
            branches.append((s, e, BRANCH_R))
            anchors.append(e)
            tm = rng.uniform(0.50, 0.70)
            anchors.append((s[0] + (e[0] - s[0]) * tm,
                            s[1] + (e[1] - s[1]) * tm,
                            s[2] + (e[2] - s[2]) * tm))
        for t0, tone in ((0.55, 0), (0.90, 1)):
            c = (p0[0] + (p1[0] - p0[0]) * t0 + rng.uniform(-0.22, 0.22),
                 p0[1] + (p1[1] - p0[1]) * t0 + rng.uniform(-0.22, 0.22),
                 p0[2] + (p1[2] - p0[2]) * t0 + rng.uniform(0.05, 0.25))
            blobs.append((c, rng.uniform(*CANOPY_R), tone))
    blobs.append(((top[0] + rng.uniform(-0.1, 0.1), top[1] + rng.uniform(-0.1, 0.1),
                   top[2] + rng.uniform(0.40, 0.55)), rng.uniform(0.50, 0.70), 0))

    bags = []
    n_bags = rng.randint(*BAGS_PER_TREE)
    # 锚点轮转分配保证散布：每个锚点先用一轮，再进入第二轮抖动重试。
    anchor_cycle = rng.sample(anchors, len(anchors))
    for idx in range(n_bags):
        placed = False
        for _try in range(150):
            # 前段用轮转锚点保证散布；后段回退随机锚点兜底，尽量不掉袋。
            anchor = (anchor_cycle[idx % len(anchor_cycle)] if _try < 100
                      else rng.choice(anchors))
            tau = math.radians(rng.uniform(0.0, 35.0))
            om = rng.uniform(0.0, 2.0 * math.pi)
            tlen = rng.uniform(*TWIG_LEN)
            mouth = (anchor[0] + tlen * math.sin(tau) * math.cos(om),
                     anchor[1] + tlen * math.sin(tau) * math.sin(om),
                     anchor[2] - tlen * math.cos(tau))
            if not (HARVEST_Z[0] <= mouth[2] <= HARVEST_Z[1]):
                continue
            if any(math.dist(mouth, b['mouth']) < BAG_MIN_DIST for b in bags):
                continue
            tilt_kind = 'horizontal' if rng.random() < HORIZONTAL_FRAC else 'main'
            tilt = math.radians(rng.uniform(*(TILT_HORIZ_DEG if tilt_kind == 'horizontal'
                                              else TILT_MAIN_DEG)))
            aaz = rng.uniform(0.0, 2.0 * math.pi)
            axis = (math.sin(tilt) * math.cos(aaz), math.sin(tilt) * math.sin(aaz),
                    math.cos(tilt))
            twig_end = (mouth[0] + TWIST_LEN * axis[0],
                        mouth[1] + TWIST_LEN * axis[1],
                        mouth[2] + TWIST_LEN * axis[2])
            twigs.append((mouth, twig_end, TWIST_R))
            bags.append({
                'id': f'bag_r{row}_t{ti}_b{idx:02d}',
                'row': row, 'tree': ti, 'index': idx,
                'mouth': mouth, 'axis': axis, 'tilt_kind': tilt_kind,
            })
            placed = True
            break
        if not placed:
            print(f'[warn] {row=},{ti=}: bag {idx} 无法满足约束，跳过', file=sys.stderr)
    return {
        'name': f'tree_r{row}_t{ti}', 'row': row, 'tree': ti,
        'trunk': (base, top, r_tr),
        'leaders': leaders, 'branches': branches, 'twigs': twigs,
        'blobs': blobs, 'bags': bags,
    }


def gen_scene(rng, rows, trees):
    row_len = (trees - 1) * TREE_SPACING
    scene = {'rows': rows, 'trees': trees, 'row_len': row_len, 'trees_list': []}
    for r in range(rows):
        ry = (r - (rows - 1) / 2.0) * ROW_SPACING
        for t in range(trees):
            tx = (t - (trees - 1) / 2.0) * TREE_SPACING
            scene['trees_list'].append(gen_tree(rng, r, t, tx, ry))
    scene['row_y'] = [(r - (rows - 1) / 2.0) * ROW_SPACING for r in range(rows)]
    scene['posts'] = [
        ((-(row_len / 2 + 0.8), ry, 0.0), (-(row_len / 2 + 0.8), ry, POST_H))
        for ry in scene['row_y']
    ] + [
        (((row_len / 2 + 0.8), ry, 0.0), ((row_len / 2 + 0.8), ry, POST_H))
        for ry in scene['row_y']
    ]
    return scene


# ================= SDF 发射（几何版 / HD 版共用骨架） =================
def sdf_cyl_visual(name, p0, p1, r, color, *, collision=False, radius_scale=1.0):
    mid, rpy, length = cyl_rpy(p0, p1)
    pose = pose_of(mid, rpy)
    geo = f'<cylinder><radius>{r:.4f}</radius><length>{length:.4f}</length></cylinder>'
    out = f'''      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry>{geo}</geometry>
        <material>{mat(color)}</material>
      </visual>'''
    if collision:
        cr = r * radius_scale
        out += f'''
      <collision name="{name}_c">
        <pose>{pose}</pose>
        <geometry><cylinder><radius>{cr:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
      </collision>'''
    return out


def sdf_sphere_visual(name, c, r, color, *, collision=False, radius_scale=0.92):
    out = f'''      <visual name="{name}">
        <pose>{fmt(*c)} 0 0 0</pose>
        <geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>
        <material>{mat(color)}</material>
      </visual>'''
    if collision:
        out += f'''
      <collision name="{name}_c">
        <pose>{fmt(*c)} 0 0 0</pose>
        <geometry><sphere><radius>{r * radius_scale:.4f}</radius></sphere></geometry>
      </collision>'''
    return out


def sdf_mesh_visual(name, uri, xyz, rpy, scale):
    return f'''      <visual name="{name}">
        <pose>{pose_of(xyz, rpy)}</pose>
        <geometry><mesh><uri>{uri}</uri><scale>{fmt(*scale)}</scale></mesh></geometry>
      </visual>'''


def emit_bag_link(bag, hd, rng):
    """袋 link：原点=袋口（mouth），纸身沿 -axis。"""
    axis = bag['axis']
    pose = pose_of(bag['mouth'], quat_to_rpy(quat_z_to(axis)))
    tone = 'a' if (bag['row'] + bag['tree'] + bag['index']) % 2 == 0 else 'b'
    if not hd:
        paper = f'''      <visual name="paper">
        <pose>0 0 {-BAG_PAPER_R:.4f} 0 0 0</pose>
        <geometry><sphere><radius>{BAG_PAPER_R:.4f}</radius></sphere></geometry>
        <material>{mat(COL_BAG)}</material>
      </visual>'''
    else:
        paper = sdf_mesh_visual(
            'paper', f'{MODEL_URI}/bag_{tone}.obj', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (BAG_PAPER_R, BAG_PAPER_R, BAG_PAPER_LEN))
    return f'''    <link name="{bag['id']}">
      <pose>{pose}</pose>
{paper}
      <visual name="twist">
        <pose>0 0 {TWIST_LEN / 2 - 0.003:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{TWIST_R:.4f}</radius><length>{TWIST_LEN:.4f}</length></cylinder></geometry>
        <material>{mat(COL_TWIST)}</material>
      </visual>
      <collision name="paper_c">
        <pose>0 0 {-BAG_PAPER_R:.4f} 0 0 0</pose>
        <geometry><sphere><radius>{BAG_PAPER_R:.4f}</radius></sphere></geometry>
      </collision>
    </link>'''


def emit_tree(tree, hd, rng):
    base, top, r_tr = tree['trunk']
    links = [f'''    <link name="trunk">
{sdf_cyl_visual("trunk_v", base, top, r_tr, COL_TRUNK, collision=True)}
    </link>''']

    frame = [sdf_cyl_visual(f'leader{i}_v', p0, p1, r, COL_TRUNK, collision=True)
             for i, (p0, p1, r) in enumerate(tree['leaders'])]
    frame += [sdf_cyl_visual(f'branch{i}_v', p0, p1, r, COL_BRANCH)
              for i, (p0, p1, r) in enumerate(tree['branches'])]
    frame += [sdf_cyl_visual(f'twig{i}_v', p0, p1, r, COL_TWIG)
              for i, (p0, p1, r) in enumerate(tree['twigs'])]
    links.append('    <link name="frame">\n' + '\n'.join(frame) + '\n    </link>')

    if not hd:
        canopy = [sdf_sphere_visual(f'blob{i}_v', c, r, COL_LEAF[tone], collision=True)
                  for i, (c, r, tone) in enumerate(tree['blobs'])]
    else:
        canopy = []
        for i, (c, r, tone) in enumerate(tree['blobs']):
            uri = f'{MODEL_URI}/leaf_blob_{"a" if tone == 0 else "b"}.obj'
            canopy.append(sdf_mesh_visual(f'blob{i}_v', uri, c, (0.0, 0.0, 0.0),
                                          (r, r, r * 0.85)))
            canopy.append(f'''      <collision name="blob{i}_c">
        <pose>{fmt(*c)} 0 0 0</pose>
        <geometry><sphere><radius>{r * 0.92:.4f}</radius></sphere></geometry>
      </collision>''')
    links.append('    <link name="canopy">\n' + '\n'.join(canopy) + '\n    </link>')
    links += [emit_bag_link(b, hd, rng) for b in tree['bags']]

    return f'''  <model name="{tree['name']}">
    <static>true</static>
{chr(10).join(links)}
  </model>'''


def emit_static_parts(scene, hd):
    gx, gy = GROUND_SIZE
    parts = [f'''  <model name="ground">
    <static>true</static>
    <link name="link">
      <collision name="c">
        <geometry><plane><normal>0 0 1</normal><size>{gx} {gy}</size></plane></geometry>
      </collision>
      <visual name="v">
        <geometry><plane><normal>0 0 1</normal><size>{gx} {gy}</size></plane></geometry>
        <material>{mat(COL_GRASS)}</material>
      </visual>
    </link>
  </model>''']

    soil = []
    for i, ry in enumerate(scene['row_y']):
        length = scene['row_len'] + 2.0 * HEADLAND_M
        soil.append(f'''      <visual name="soil_r{i}_v">
        <pose>0 {ry:.4f} 0.012 0 0 0</pose>
        <geometry><box><size>{length:.4f} {SOIL_WIDTH} 0.02</size></box></geometry>
        <material>{mat(COL_SOIL)}</material>
      </visual>''')
    parts.append('  <model name="soil_strips">\n    <static>true</static>\n'
                 '    <link name="link">\n' + '\n'.join(soil) + '\n    </link>\n  </model>')

    posts = []
    for i, (p0, p1) in enumerate(scene['posts']):
        posts.append(sdf_cyl_visual(f'post{i}_v', p0, p1, POST_R, COL_WOOD, collision=True))
    parts.append('  <model name="posts">\n    <static>true</static>\n'
                 '    <link name="link">\n' + '\n'.join(posts) + '\n    </link>\n  </model>')
    return parts


WORLD_SKELETON = '''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8066</gravity>
    <scene>
      <ambient>0.55 0.55 0.60 1</ambient>
      <background>0.62 0.78 0.95 1</background>
      <shadows>true</shadows>
      <sky>
        <time>11.5</time>
        <clouds>
          <speed>0.6</speed>
        </clouds>
      </sky>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>10 -8 15 0 0 0</pose>
      <diffuse>0.85 0.83 0.78 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <attenuation>
        <range>80</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.35 0.30 -1</direction>
    </light>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
{camera_block}{static_parts}
{trees}
  </world>
</sdf>
'''

OVERVIEW_CAM = '''
    <model name="overview_cam">
      <static>true</static>
      <pose>13 -13 6.5 0 0.281 2.356</pose>
      <link name="link">
        <sensor name="overview_cam" type="camera">
          <pose>0 0 0 0 0 0</pose>
          <camera>
            <horizontal_fov>1.10</horizontal_fov>
            <image>
              <width>1280</width>
              <height>720</height>
              <format>RGB_INT8</format>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>true</always_on>
          <update_rate>5</update_rate>
          <topic>overview_cam/image</topic>
        </sensor>
      </link>
    </model>'''


def emit_world(scene, world_name, hd, rng):
    trees_xml = '\n'.join(emit_tree(t, hd, rng) for t in scene['trees_list'])
    static_xml = '\n'.join(emit_static_parts(scene, hd))
    camera = OVERVIEW_CAM if hd else ''
    return WORLD_SKELETON.format(world_name=world_name, camera_block=camera,
                                 static_parts=static_xml, trees=trees_xml)


# ================= targets JSON（语义对齐 targets_20260909） =================
def emit_targets_json(scene, seed):
    bags = []
    for tree in scene['trees_list']:
        for b in tree['bags']:
            mouth, axis = b['mouth'], b['axis']
            bottom = tuple(mouth[i] - BAG_PAPER_LEN * axis[i] for i in range(3))
            bags.append({
                'id': b['id'], 'row': b['row'], 'tree': b['tree'], 'index': b['index'],
                'entry_xyz': [round(v, 4) for v in bottom],
                'axis': [round(v, 4) for v in axis],
                'bag_bottom': [round(v, 4) for v in bottom],
                'bag_neck': [round(v, 4) for v in mouth],
                'travel_m': round(BAG_PAPER_LEN, 4),
                'axis_z': round(axis[2], 4),
                'height_z': round(mouth[2], 4),
                'tilt_kind': b['tilt_kind'],
            })
    axis_zs = [b['axis_z'] for b in bags]
    zs = [b['height_z'] for b in bags]
    doc = {
        'schema': 'orchard_targets_v1',
        'frame': 'world：原点=果园中心，+x 沿行向，+y 跨行，+z 向上（单位 m）',
        'generated_by': 'orchard_scene/tools/generate_orchard_world.py',
        'seed': seed,
        'layout': {
            'rows': scene['rows'], 'trees_per_row': scene['trees'],
            'row_spacing_m': ROW_SPACING, 'tree_spacing_m': TREE_SPACING,
            'harvest_band_z_m': list(HARVEST_Z),
        },
        'bag_geometry': {
            'paper_radius_m': BAG_PAPER_R,
            'travel_m': BAG_PAPER_LEN,
            'note': 'entry_xyz=袋底=工具入口；axis=袋底→袋颈(单位向量)；'
                    'bag_neck=entry_xyz+axis*travel_m。字段语义对齐 '
                    'src/peach_arm/test/fixtures/field_pregrasp_cases.yaml '
                    'targets_20260909（该文件为 base_link 系，本文件为 world 系）',
        },
        'stats': {
            'bag_total': len(bags),
            'axis_z_min': min(axis_zs),
            'axis_z_max': max(axis_zs),
            'axis_z_ge_0p70_frac': round(sum(1 for z in axis_zs if z >= 0.70) / len(axis_zs), 4),
            'horizontal_bags': sum(1 for b in bags if b['tilt_kind'] == 'horizontal'),
            'height_z_range': [round(min(zs), 3), round(max(zs), 3)],
        },
        'bags': bags,
    }
    return doc


# ================= 预览图（matplotlib，缺库则跳过不阻塞） =================
def render_previews(scene, targets, outdir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.patches import Circle, Rectangle
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError as exc:
        print(f'[warn] matplotlib 不可用（{exc}），跳过预览图', file=sys.stderr)
        return

    bags = targets['bags']

    # ---------- 图 1：布局四联图 ----------
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f'Outdoor bagged-peach orchard — seed={targets["seed"]}, '
                 f'{scene["rows"]} rows x {scene["trees"]} trees, '
                 f'{len(bags)} bags (geometric tier)', fontsize=13)

    ax = fig.add_subplot(2, 2, 1, projection='3d')
    for tree in scene['trees_list']:
        base, top, _ = tree['trunk']
        ax.plot([base[0], top[0]], [base[1], top[1]], [base[2], top[2]],
                color='saddlebrown', lw=2)
        for p0, p1, _r in tree['leaders'] + tree['branches']:
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]],
                    color='peru', lw=0.8, alpha=0.7)
    cx = np.array([[c[0] for c, _, _ in t['blobs']] for t in scene['trees_list']]).ravel()
    cy = np.array([[c[1] for c, _, _ in t['blobs']] for t in scene['trees_list']]).ravel()
    cz = np.array([[c[2] for c, _, _ in t['blobs']] for t in scene['trees_list']]).ravel()
    cr = np.array([[r for _, r, _ in t['blobs']] for t in scene['trees_list']]).ravel()
    ax.scatter(cx, cy, cz, s=cr ** 2 * 900, c='#3d7a26', alpha=0.30, depthshade=False)
    bx = [b['bag_neck'][0] for b in bags]
    by = [b['bag_neck'][1] for b in bags]
    bz = [b['bag_neck'][2] for b in bags]
    ax.scatter(bx, by, bz, s=14, c='#d29b3a', depthshade=False, label='bag mouth')
    ax.scatter([], [], [], s=40, c='#3d7a26', alpha=0.4, label='canopy blob')
    ax.legend(loc='upper left')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_title('3D view')
    ax.view_init(elev=18, azim=-60)
    try:
        ax.set_box_aspect((14, 12, 5))
    except AttributeError:
        pass

    ax = fig.add_subplot(2, 2, 2)
    for i, ry in enumerate(scene['row_y']):
        length = scene['row_len'] + 2 * HEADLAND_M
        ax.add_patch(Rectangle((-length / 2, ry - SOIL_WIDTH / 2), length, SOIL_WIDTH,
                               facecolor='#6b5538', alpha=0.55, zorder=1))
    for tree in scene['trees_list']:
        for c, r, tone in tree['blobs']:
            ax.add_patch(Circle((c[0], c[1]), r, facecolor='#3d7a26' if tone == 0 else '#4d8a2c',
                                alpha=0.35, zorder=2, edgecolor='none'))
    ax.scatter(bx, by, s=9, c='#c47f17', zorder=3, label='bags (top view)')
    px = [p[0][0] for p in scene['posts']]
    py = [p[0][1] for p in scene['posts']]
    ax.scatter(px, py, marker='s', s=28, c='#8a6d3b', zorder=3, label='end posts')
    ax.annotate('', xy=(scene['row_len'] / 2 + 2.2, scene['row_y'][0]),
                xytext=(scene['row_len'] / 2 + 2.2, scene['row_y'][1]),
                arrowprops=dict(arrowstyle='<->', color='k', lw=1))
    ax.text(scene['row_len'] / 2 + 2.35, (scene['row_y'][0] + scene['row_y'][1]) / 2,
            f'row spacing {ROW_SPACING:.1f} m', fontsize=8, va='center')
    ax.annotate('', xy=(0, scene['row_y'][1] - 1.35), xytext=(TREE_SPACING, scene['row_y'][1] - 1.35),
                arrowprops=dict(arrowstyle='<->', color='k', lw=1))
    ax.text(TREE_SPACING / 2, scene['row_y'][1] - 1.62, f'tree spacing {TREE_SPACING:.1f} m',
            fontsize=8, ha='center')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (along rows)')
    ax.set_ylabel('y [m] (across rows)')
    ax.set_title('Top view')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.25)

    ax = fig.add_subplot(2, 2, 3)
    ax.axhspan(HARVEST_Z[0], HARVEST_Z[1], facecolor='#f2c94c', alpha=0.18,
               label=f'harvest band {HARVEST_Z[0]:.1f}-{HARVEST_Z[1]:.1f} m')
    for tree in scene['trees_list']:
        for c, r, tone in tree['blobs']:
            ax.add_patch(Circle((c[0], c[2]), r, facecolor='#3d7a26' if tone == 0 else '#4d8a2c',
                                alpha=0.30, edgecolor='none'))
    ax.scatter(bx, bz, s=9, c='#c47f17')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('z [m]')
    ax.set_title('Side view (projected on x-z)')
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(alpha=0.25)

    ax = fig.add_subplot(2, 2, 4)
    az = [b['axis_z'] for b in bags]
    ax.hist(az, bins=24, color='#d29b3a', edgecolor='k', alpha=0.8)
    ax.axvline(0.70, color='crimson', ls='--', lw=1.5, label='envelope axis_z >= 0.70')
    st = targets['stats']
    ax.text(0.02, 0.97,
            f'total bags: {st["bag_total"]}\n'
            f'axis_z >= 0.70: {st["axis_z_ge_0p70_frac"] * 100:.1f}%\n'
            f'near-horizontal: {st["horizontal_bags"]}\n'
            f'bag height range: {st["height_z_range"][0]:.2f}-{st["height_z_range"][1]:.2f} m\n'
            f'travel (bottom->neck): {BAG_PAPER_LEN:.3f} m',
            transform=ax.transAxes, va='top', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.85))
    ax.set_xlabel('bag axis_z (bottom -> neck, unit vector)')
    ax.set_ylabel('count')
    ax.set_title('Bag axis orientation')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out1 = outdir / 'preview' / 'orchard_layout.png'
    fig.savefig(out1, dpi=140)
    plt.close(fig)

    # ---------- 图 2：HD 网格软件渲染（真实 OBJ 三角面 + Lambert 着色） ----------
    light = np.array([-0.45, 0.35, 0.82])
    light /= np.linalg.norm(light)

    def rot_from_dir(d):
        q = quat_z_to(d)
        qw, qx, qy, qz = q
        return np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)],
        ])

    def add_mesh(ax, verts, faces, base_rgb, transform, collections):
        v = np.asarray(verts) @ transform[:3, :3].T + transform[:3, 3]
        tris = v[np.asarray(faces)]
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        norm = np.linalg.norm(n, axis=1, keepdims=True)
        norm[norm == 0] = 1.0
        n /= norm
        k = 0.35 + 0.65 * np.clip(n @ light, 0.0, 1.0)
        cols = np.clip(k[:, None] * np.asarray(base_rgb)[None, :], 0, 1)
        pc = Poly3DCollection(tris, facecolors=cols, edgecolors='none')
        ax.add_collection3d(pc)
        collections.append(len(tris))

    cv, cf, _ = meshgen.tapered_cyl()
    la, lf = meshgen.icosphere(2, noise_fn=lambda v: 1.0)
    bv, bf, _ = meshgen.bag_shape()

    def draw_tree(ax, tree, stats):
        base, top, r_tr = tree['trunk']
        d = np.array(top) - np.array(base)
        T = np.eye(4)
        # 先局部缩放再旋转（R@S）：错序会把倾斜的枝/袋沿世界轴压扁。
        T[:3, :3] = rot_from_dir(tuple(d / np.linalg.norm(d))) @ np.diag(
            [r_tr, r_tr, np.linalg.norm(d)])
        T[:3, 3] = base
        add_mesh(ax, cv, cf, COL_TRUNK, T, stats)
        for kind, elems, col in (('leader', tree['leaders'], COL_TRUNK),
                                 ('branch', tree['branches'], COL_BRANCH),
                                 ('twig', tree['twigs'], COL_TWIG)):
            for p0, p1, r in elems:
                d = np.array(p1) - np.array(p0)
                L = np.linalg.norm(d)
                T = np.eye(4)
                T[:3, :3] = rot_from_dir(tuple(d / L)) @ np.diag([r, r, L])
                T[:3, 3] = p0
                add_mesh(ax, cv, cf, col, T, stats)
        for c, r, tone in tree['blobs']:
            T = np.eye(4)
            T[:3, :3] = np.diag([r, r, r * 0.85])
            T[:3, 3] = c
            add_mesh(ax, la, lf, COL_LEAF[tone], T, stats)
        for b in tree['bags']:
            T = np.eye(4)
            T[:3, :3] = rot_from_dir(tuple(b['axis'])) @ np.diag(
                [BAG_PAPER_R, BAG_PAPER_R, BAG_PAPER_LEN])
            T[:3, 3] = b['mouth']
            tone = (b['row'] + b['tree'] + b['index']) % 2
            add_mesh(ax, bv, bf, COL_BAG if tone == 0 else (0.78, 0.60, 0.27), T, stats)

    def style_axes_as_outdoor(ax, xlim, ylim, zlim, elev, azim, box_aspect):
        # mplot3d 的集合级深度排序会把地面多边形盖在树前面；改用 z 轴底板
        # 当草地（pane 是背景层，永不遮挡），土带/木桩退化为细线。
        ax.view_init(elev=elev, azim=azim)
        try:
            ax.set_box_aspect(box_aspect)
        except AttributeError:
            pass
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_zlim(*zlim)
        ax.zaxis.set_pane_color((*COL_GRASS, 1.0))
        ax.xaxis.set_pane_color((0.93, 0.95, 0.97, 1.0))
        ax.yaxis.set_pane_color((0.90, 0.93, 0.96, 1.0))

    def draw_soil_and_posts(ax, rows_y, z=0.01):
        length = scene['row_len'] + 2 * HEADLAND_M
        for ry in rows_y:
            ax.plot([-length / 2, length / 2], [ry, ry], [z, z],
                    color=COL_SOIL, lw=6, solid_capstyle='butt')
        for p0, p1 in scene['posts']:
            if any(abs(p0[1] - ry) < 1e-6 for ry in rows_y):
                ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]],
                        color=COL_WOOD, lw=3)

    fig = plt.figure(figsize=(16, 8))
    fig.suptitle('HD tier — software render of the actual OBJ meshes '
                 '(same geometry as gazebo HD world)', fontsize=13)
    stats = []
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    style_axes_as_outdoor(ax, (-9, 9), (-8, 8), (0, 6), 24, -55, (16, 14, 6))
    draw_soil_and_posts(ax, scene['row_y'])
    for tree in scene['trees_list']:
        draw_tree(ax, tree, stats)
    ax.set_title(f'Overview — {len(bags)} bags / {len(scene["trees_list"])} trees')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')

    center = next(t for t in scene['trees_list'] if t['row'] == 1 and t['tree'] == 2)
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    style_axes_as_outdoor(ax, (-2.2, 2.2), (-2.2, 2.2), (0.0, 2.6), 18, -120,
                          (4.4, 4.4, 2.6))
    draw_soil_and_posts(ax, [center['trunk'][0][1]])
    sub = {'trunk': center['trunk'],
           'leaders': center['leaders'], 'branches': center['branches'],
           'twigs': center['twigs'], 'blobs': center['blobs'], 'bags': center['bags']}
    draw_tree(ax, sub, stats)
    ax.set_title(f'Close-up — {center["name"]} ({len(center["bags"])} bags)')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out2 = outdir / 'preview' / 'orchard_hd_render.png'
    fig.savefig(out2, dpi=140)
    plt.close(fig)
    print(f'[ok] 预览图: {out1.name} ({out1.stat().st_size // 1024} KB), '
          f'{out2.name} ({out2.stat().st_size // 1024} KB), '
          f'HD 渲染三角面 {sum(stats)}')


# ================= main =================
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seed', type=int, default=SEED)
    ap.add_argument('--rows', type=int, default=4)
    ap.add_argument('--trees', type=int, default=5)
    ap.add_argument('--outdir', type=Path, default=HERE)
    ap.add_argument('--no-preview', action='store_true')
    args = ap.parse_args()
    t0 = time.time()

    rng = random.Random(args.seed)
    scene = gen_scene(rng, args.rows, args.trees)

    worlds = args.outdir / 'worlds'
    worlds.mkdir(parents=True, exist_ok=True)
    (worlds / 'bagged_peach_orchard.sdf').write_text(
        emit_world(scene, 'bagged_peach_orchard', hd=False, rng=random.Random(args.seed + 1)))
    (worlds / 'bagged_peach_orchard_hd.sdf').write_text(
        emit_world(scene, 'bagged_peach_orchard_hd', hd=True, rng=random.Random(args.seed + 1)))

    meshes_dir = args.outdir / 'models' / 'orchard_meshes' / 'meshes'
    materials_dir = args.outdir / 'models' / 'orchard_meshes' / 'materials'
    mesh_stats = meshgen.build_assets(meshes_dir, materials_dir, random.Random(args.seed + 2))
    (args.outdir / 'models' / 'orchard_meshes' / 'model.config').write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>orchard_meshes</name>\n'
        '  <version>1.0</version>\n  <sdf version="1.8">model.sdf</sdf>\n'
        '  <author><name>orchard_scene generator</name><email>none@invalid</email></author>\n'
        '  <description>Procedural meshes for the bagged-peach orchard scene'
        ' (visual only; collisions stay coarse primitives).</description>\n</model>\n')
    (args.outdir / 'models' / 'orchard_meshes' / 'model.sdf').write_text(
        '<?xml version="1.0"?>\n<sdf version="1.8">\n'
        '  <model name="orchard_meshes">\n    <static>true</static>\n  </model>\n</sdf>\n')

    targets = emit_targets_json(scene, args.seed)
    data_path = args.outdir / 'data' / 'orchard_targets.json'
    data_path.parent.mkdir(parents=True, exist_ok=True)
    data_path.write_text(json.dumps(targets, ensure_ascii=False, indent=1) + '\n')

    if not args.no_preview:
        render_previews(scene, targets, args.outdir)

    n_bags = targets['stats']['bag_total']
    st = targets['stats']
    print(f'[ok] 场景: {args.rows} 行 x {args.trees} 株 = {len(scene["trees_list"])} 树, {n_bags} 袋')
    print(f'[ok] 包络: axis_z∈[{st["axis_z_min"]:.3f},{st["axis_z_max"]:.3f}], '
          f'>=0.70 占比 {st["axis_z_ge_0p70_frac"] * 100:.1f}%, '
          f'近水平压测袋 {st["horizontal_bags"]} 只, '
          f'高度带 {st["height_z_range"][0]:.2f}-{st["height_z_range"][1]:.2f} m')
    for name, (nv, nf) in mesh_stats.items():
        print(f'[ok] mesh {name}: {nv}v/{nf}f')
    for f in sorted(worlds.glob('*.sdf')):
        print(f'[ok] {f.relative_to(args.outdir)}: {f.stat().st_size // 1024} KB')
    print(f'[ok] {data_path.relative_to(args.outdir)}: {data_path.stat().st_size // 1024} KB')
    print(f'[ok] 用时 {time.time() - t0:.1f}s (seed={args.seed})')


if __name__ == '__main__':
    main()
