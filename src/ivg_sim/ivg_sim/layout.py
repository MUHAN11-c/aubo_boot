# -*- coding: utf-8 -*-
"""
场景布局纯核：table_layout.yaml 的加载、摆位数学与相机光学系 TF 推导.

零 ROS 依赖（生成器 / launch / 评分工具共用，杜绝多处硬编码漂移）。
相机约定：世界系 z 向上；相机 rig 静态悬于桌面正上方，光轴竖直向下。
gz-sim 点云的轴系约定（GZ_OPTICAL_CONVENTION）经 spike 实测裁定：
- 'body'   ：点云在传感器体系（x 前=视轴、y 左、z 上）——gz-sim 8 实测值
             （2026-09-29：x∈[0.51,1.5] 为深度轴，与 R_y(π/2) 推导一致）
- 'opengl' ：点云在 OpenGL 光学系（x 右、y 上、z 前）——备用
- 'ros'    ：点云在 REP-103 光学系（x 右、y 下、z 前）——备用
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

GZ_OPTICAL_CONVENTION = 'body'

DEFAULT_LAYOUT_PATH = Path(__file__).resolve().parent.parent / 'config' / 'table_layout.yaml'


@dataclass
class ObjectPlacement:
    """单个 YCB 对象的桌面摆位（世界系；z 为对象底部即桌面高度）."""

    model: str
    x: float
    y: float
    z: float
    yaw_deg: float
    radius_m: float  # 近似包围半径（评分命中半径用）


@dataclass
class TableLayout:
    """解析后的场景布局."""

    world_name: str
    physics_step: float
    table_top_z: float
    table_size: Tuple[float, float, float]
    table_color: Tuple[float, float, float]
    camera_height_above_table: float
    camera_xy: Tuple[float, float]
    camera_fov_deg: float
    camera_update_rate: float
    camera_image: Tuple[int, int]
    camera_clip: Tuple[float, float]
    optical_frame: str
    camera_tilt_deg: float = 0.0  # 光轴偏离竖直的角度（朝 +X 倾斜）
    placements: List[ObjectPlacement] = field(default_factory=list)
    source_path: str = ''
    seed: Optional[int] = None


def load_layout(path: Optional[str] = None) -> TableLayout:
    """加载并解析 table_layout.yaml."""
    layout_path = Path(path) if path else DEFAULT_LAYOUT_PATH
    raw = yaml.safe_load(layout_path.read_text(encoding='utf-8'))
    table = raw['table']
    camera = raw['camera']
    world = raw.get('world', {})
    placements = [
        ObjectPlacement(
            model=str(item['model']),
            x=float(item['xy'][0]),
            y=float(item['xy'][1]),
            z=float(table['top_z']),
            yaw_deg=float(item.get('yaw', 0.0)),
            radius_m=float(item.get('radius', 0.08)),
        )
        for item in raw.get('objects', [])
    ]
    return TableLayout(
        world_name=str(world.get('name', 'ivg_table')),
        physics_step=float(world.get('physics_step', 0.004)),
        table_top_z=float(table['top_z']),
        table_size=tuple(float(v) for v in table['size']),
        table_color=tuple(float(v) for v in table.get('color', (0.55, 0.45, 0.35))),
        camera_height_above_table=float(camera['height_above_table']),
        camera_xy=tuple(float(v) for v in camera.get('xy', (0.0, 0.0))),
        camera_fov_deg=float(camera['fov_deg']),
        camera_update_rate=float(camera['update_rate']),
        camera_image=tuple(int(v) for v in camera['image']),
        camera_clip=tuple(float(v) for v in camera['clip']),
        optical_frame=str(camera['optical_frame']),
        camera_tilt_deg=float(camera.get('tilt_deg', 0.0)),
        placements=placements,
        source_path=str(layout_path),
        seed=raw.get('seed'),
    )


# ---------------------------------------------------------------------------
# 摆位数学
# ---------------------------------------------------------------------------
def apply_jitter(layout: TableLayout, seed: int, spread_m: float = 0.05,
                 yaw_jitter_deg: float = 25.0) -> TableLayout:
    """确定性扰动摆位（同 seed 同结果）：xy 平面高斯扰动 + yaw 均匀扰动.

    返回新 TableLayout（不修改入参）；扰动保持在桌面内（xy 夹到桌半幅-半径）。
    """
    rng = np.random.default_rng(seed)
    half_x = layout.table_size[0] / 2.0
    half_y = layout.table_size[1] / 2.0
    new_placements = []
    for placement in layout.placements:
        dx, dy = rng.normal(0.0, spread_m / 2.0, 2)
        x = float(np.clip(placement.x + dx, -half_x + placement.radius_m,
                          half_x - placement.radius_m))
        y = float(np.clip(placement.y + dy, -half_y + placement.radius_m,
                          half_y - placement.radius_m))
        new_placements.append(ObjectPlacement(
            model=placement.model, x=x, y=y, z=placement.z,
            yaw_deg=float(placement.yaw_deg + rng.uniform(-yaw_jitter_deg, yaw_jitter_deg)),
            radius_m=placement.radius_m,
        ))
    return TableLayout(
        world_name=layout.world_name, physics_step=layout.physics_step,
        table_top_z=layout.table_top_z, table_size=layout.table_size,
        table_color=layout.table_color,
        camera_height_above_table=layout.camera_height_above_table,
        camera_xy=layout.camera_xy, camera_fov_deg=layout.camera_fov_deg,
        camera_update_rate=layout.camera_update_rate,
        camera_image=layout.camera_image, camera_clip=layout.camera_clip,
        optical_frame=layout.optical_frame,
        camera_tilt_deg=layout.camera_tilt_deg,
        placements=new_placements,
        source_path=layout.source_path, seed=seed,
    )


# ---------------------------------------------------------------------------
# 相机几何
# ---------------------------------------------------------------------------
def camera_world_pose(layout: TableLayout) -> Tuple[Tuple[float, float, float],
                                                     Tuple[float, float, float]]:
    """相机 rig（SDF 传感器 pose）的世界位姿：(xyz, rpy_rad).

    传感器体系 +X 为视轴。tilt=0 时光轴竖直向下（RPY(0, π/2, 0)）；
    tilt>0 时光轴朝 +X 倾斜（pitch = π/2 − tilt），相机沿 −视轴方向
    后退保持对桌面中心的视距 = height_above_table。
    """
    tilt = math.radians(layout.camera_tilt_deg)
    distance = layout.camera_height_above_table
    cx, cy = layout.camera_xy
    xyz = (
        cx - distance * math.sin(tilt),
        cy,
        layout.table_top_z + distance * math.cos(tilt),
    )
    pitch = math.pi / 2.0 - tilt
    return xyz, (0.0, pitch, 0.0)


def _quat_from_matrix(rot: np.ndarray) -> Tuple[float, float, float, float]:
    """旋转矩阵 → 四元数 xyzw（scipy 不引入，纯 numpy 实现）."""
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def optical_frame_rotation(tilt_deg: float = 0.0,
                           convention: str = GZ_OPTICAL_CONVENTION) -> np.ndarray:
    """rig RPY(0, π/2−tilt, 0) 下，点云坐标系三轴在世界系中的方向（列向量）.

    传感器体系（rig pose 后）：X_b=视轴、Y_b=世界+Y、Z_b=视轴绕 Y 左转 90°。
    tilt=0 时 X_b=(0,0,−1) 竖直向下（spike 实测基准）。
    """
    tilt = math.radians(tilt_deg)
    body_x = np.array([math.sin(tilt), 0.0, -math.cos(tilt)])  # 视轴
    body_y = np.array([0.0, 1.0, 0.0])
    body_z = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
    if convention == 'body':
        cols = [body_x, body_y, body_z]
    elif convention == 'opengl':
        # OpenGL 光学系：x 右(+Y_b)、y 上(+Z_b)、z 前(+X_b)
        cols = [body_y, body_z, body_x]
    elif convention == 'ros':
        # REP-103 光学系：x 右(+Y_b)、y 下(-Z_b)、z 前(+X_b)
        cols = [body_y, -body_z, body_x]
    else:
        raise ValueError(f'未知点云轴系约定: {convention!r}')
    rot = np.column_stack(cols)
    assert np.isclose(np.linalg.det(rot), 1.0), '光学系旋转必须右手系'
    return rot


def camera_optical_tf(layout: TableLayout,
                      convention: str = GZ_OPTICAL_CONVENTION
                      ) -> Tuple[Tuple[float, float, float],
                                 Tuple[float, float, float, float]]:
    """base_link(=世界原点) ← 光学系 的静态 TF：(平移, 四元数 xyzw).

    launch 的 static_transform_publisher 与评分工具共用本推导。
    """
    xyz, _rpy = camera_world_pose(layout)
    rot = optical_frame_rotation(layout.camera_tilt_deg, convention)
    quat = _quat_from_matrix(rot)
    return xyz, quat


# ---------------------------------------------------------------------------
# manifest（GT 位姿单一事实源，评分工具消费）
# ---------------------------------------------------------------------------
def build_manifest(layout: TableLayout, world_sha256: str) -> Dict[str, Any]:
    """GT manifest：对象世界位姿（含顶部中心 z+r）、相机位姿、seed、世界哈希."""
    xyz, rpy = camera_world_pose(layout)
    return {
        'world_name': layout.world_name,
        'seed': layout.seed,
        'world_sha256': world_sha256,
        'table_top_z': layout.table_top_z,
        'camera': {
            'xyz': list(xyz), 'rpy_rad': list(rpy),
            'tilt_deg': layout.camera_tilt_deg,
            'optical_frame': layout.optical_frame,
            'convention': GZ_OPTICAL_CONVENTION,
        },
        'objects': [
            {
                'model': p.model,
                'xyz': [round(p.x, 6), round(p.y, 6), round(p.z, 6)],
                'yaw_deg': round(p.yaw_deg, 4),
                'radius_m': p.radius_m,
                'top_center_xyz': [
                    round(p.x, 6), round(p.y, 6), round(p.z + p.radius_m, 6),
                ],
            }
            for p in layout.placements
        ],
    }
