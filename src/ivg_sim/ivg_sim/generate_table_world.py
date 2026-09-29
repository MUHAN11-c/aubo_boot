#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_table_world：由 config/table_layout.yaml 生成 Gazebo Harmonic 世界.

产物（均写入包 worlds/ 目录）：
  ivg_table.sdf          —— SDF 1.11 世界：地面 + 平行光 + 桌面板 +
                            10 YCB 对象（<include>）+ 静态俯视 rgbd 相机
  ivg_table.manifest.yaml —— GT 对象位姿 / 相机位姿 / seed / 世界 sha256
                            （score_grasps 的 ground truth）

用法：
  ros2 run ivg_sim generate_table_world [--layout <yaml>] [--jitter --seed 7]
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
from typing import Optional

import yaml

from .layout import (
    TableLayout,
    apply_jitter,
    build_manifest,
    camera_world_pose,
    load_layout,
)

PKG_ROOT = Path(__file__).resolve().parent.parent
WORLDS_DIR = PKG_ROOT / 'worlds'


def generate_sdf(layout: TableLayout) -> str:
    """渲染世界 SDF（peach_sim 风格：显式 systems 插件 + SDF 1.11）."""
    cam_xyz, cam_rpy = camera_world_pose(layout)
    table_sx, table_sy, table_sz = layout.table_size
    table_cz = layout.table_top_z - table_sz / 2.0
    tr, tg, tb = layout.table_color
    fov_rad = math.radians(layout.camera_fov_deg)
    near, far = layout.camera_clip
    width, height = layout.camera_image

    includes = []
    for idx, placement in enumerate(layout.placements):
        yaw_rad = math.radians(placement.yaw_deg)
        includes.append(f'''  <include>
    <uri>model://{placement.model}</uri>
    <name>obj_{idx:02d}_{placement.model}</name>
    <pose>{placement.x:.6f} {placement.y:.6f} {placement.z:.6f} 0 0 {yaw_rad:.6f}</pose>
  </include>''')
    includes_block = '\n'.join(includes)

    return f'''<?xml version="1.0" ?>
<sdf version="1.11">
  <world name="{layout.world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>{layout.physics_step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <!-- 零重力：对象冻结在摆位（GT=manifest 精确一致）；抓取检测基准要
           确定性场景，动态交互验证属后续轮 -->
      <gravity>0 0 0</gravity>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 3 0 0 0</pose>
      <diffuse>0.9 0.9 0.9</diffuse>
      <specular>0.2 0.2 0.2</specular>
      <attenuation>
        <range>20</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6</ambient>
            <diffuse>0.7 0.7 0.7</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="table">
      <static>true</static>
      <pose>0 0 {table_cz:.6f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{table_sx} {table_sy} {table_sz}</size></box>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{table_sx} {table_sy} {table_sz}</size></box>
          </geometry>
          <material>
            <ambient>{tr} {tg} {tb}</ambient>
            <diffuse>{tr} {tg} {tb}</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="camera_rig">
      <static>true</static>
      <pose>{cam_xyz[0]:.6f} {cam_xyz[1]:.6f} {cam_xyz[2]:.6f} {cam_rpy[0]:.6f} {cam_rpy[1]:.6f} {cam_rpy[2]:.6f}</pose>
      <link name="link">
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>1e-4</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1e-4</iyy><iyz>0</iyz><izz>1e-4</izz>
          </inertia>
        </inertial>
        <sensor name="rgbd" type="rgbd_camera">
          <topic>camera/depth</topic>
          <update_rate>{layout.camera_update_rate}</update_rate>
          <camera name="rgbd">
            <horizontal_fov>{fov_rad:.6f}</horizontal_fov>
            <image>
              <width>{width}</width>
              <height>{height}</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>{near}</near>
              <far>{far}</far>
            </clip>
            <depth_camera>
              <output>depths</output>
            </depth_camera>
          </camera>
        </sensor>
      </link>
    </model>

{includes_block}
  </world>
</sdf>
'''


def generate(layout_path: Optional[str] = None, jitter: bool = False,
             seed: Optional[int] = None,
             worlds_dir: Optional[Path] = None) -> Path:
    """生成世界 SDF + GT manifest，返回 SDF 路径."""
    layout = load_layout(layout_path)
    if jitter:
        if seed is None:
            raise SystemExit('--jitter 需要配合 --seed N（确定性复现）')
        layout = apply_jitter(layout, seed)

    sdf_text = generate_sdf(layout)
    world_sha256 = hashlib.sha256(sdf_text.encode('utf-8')).hexdigest()
    manifest = build_manifest(layout, world_sha256)

    out_dir = Path(worlds_dir) if worlds_dir else WORLDS_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    world_path = out_dir / f'{layout.world_name}.sdf'
    manifest_path = out_dir / f'{layout.world_name}.manifest.yaml'
    world_path.write_text(sdf_text, encoding='utf-8')
    manifest_path.write_text(
        yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False),
        encoding='utf-8',
    )
    return world_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', default=None,
                        help='布局 yaml（缺省包内 config/table_layout.yaml）')
    parser.add_argument('--jitter', action='store_true',
                        help='对摆位做确定性随机扰动')
    parser.add_argument('--seed', type=int, default=None,
                        help='扰动种子（配合 --jitter，同 seed 同布局）')
    args = parser.parse_args()

    world_path = generate(layout_path=args.layout, jitter=args.jitter,
                          seed=args.seed)
    print(f'世界已生成: {world_path}')
    print(f'GT manifest: {world_path.parent / (world_path.stem + ".manifest.yaml")}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
