"""
果园场景 SDF 与目标清单生成（零 ROS import）.

一次布局计算同时产出两件同源产物：

* 世界 SDF（``worlds/peach_orchard.sdf``）：室外光照 + 地面/行带 + 立柱拉线 +
  桃树低模 + 套袋桃模型 + 采摘工位摆位（履带车与臂由 launch 生成进世界）；
* 目标清单（``worlds/peach_orchard.manifest.yaml``）：每颗套袋桃的世界系
  袋底/袋颈/轴/果心与可达标记，供感知/重建回归对 GT。

随机性全部来自 ``seed`` 派生的逐实体随机流（``random.Random(f'{seed}/{id}')``），
与生成顺序无关：同参数必同产物。方向一律用 (roll=0, pitch, yaw) 表达——
SDF ``pose`` 取 Rz(yaw)·Ry(pitch)·Rx(roll)，此时局部 +Z 对准方向向量、
局部 +Y 自动水平，正好给扁平袋体定厚向，无需通用旋转矩阵。
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import random
import xml.etree.ElementTree as ET

from .params import check_params, OrchardParams, Vec3
from .textures import INNER_PAPER as _INNER_RGB
from .textures import PAPER_DARK as _PAPER_DARK_RGB

SDF_VERSION = '1.11'
WORLD_NAME = 'peach_orchard'
PLATFORM_MODEL = 'harvester'
MANIFEST_SCHEMA = 'peach_sim/orchard_manifest/v1'

# 材质（ambient/diffuse/specular 同色，低模不追光照贴图）
_SOIL = (0.36, 0.29, 0.21)
_GRASS = (0.30, 0.40, 0.19)
_BARK = (0.34, 0.25, 0.16)
_TWIG = (0.47, 0.36, 0.23)
_LEAF = (0.24, 0.38, 0.16)
_LEAF_LIGHT = (0.32, 0.46, 0.20)
_PAPER = (0.51, 0.27, 0.27)
_PAPER_NECK = (0.46, 0.24, 0.25)
_TIE = (0.25, 0.25, 0.25)
_FRUIT = (0.87, 0.42, 0.30)
_POST = (0.62, 0.62, 0.60)
_WIRE = (0.45, 0.45, 0.45)


def _flat(rgb255: tuple[int, int, int]) -> Vec3:
    """贴图调色板（0–255 整数）→ SDF 颜色（0–1 浮点）."""
    return (rgb255[0] / 255.0, rgb255[1] / 255.0, rgb255[2] / 255.0)


PAPER_DARK = _flat(_PAPER_DARK_RGB)
INNER_PAPER = _flat(_INNER_RGB)


def _vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _vec_norm(a: Vec3) -> Vec3:
    length = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    if length <= 0.0:
        raise ValueError(f'零向量不可单位化: {a}')
    return (a[0] / length, a[1] / length, a[2] / length)


def _vec_dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def aim_rpy(direction: Vec3) -> tuple[float, float, float]:
    """
    把局部 +Z 对准 ``direction`` 的 (roll, pitch, yaw).

    roll 恒为 0：此时局部 +Y = (-sin yaw, cos yaw, 0) 自动水平，可直接当
    扁平袋体的厚向；圆柱绕轴无差别，同样适用。
    """
    x, y, z = _vec_norm(direction)
    return (0.0, math.atan2(math.hypot(x, y), z), math.atan2(y, x))


def _fmt(*values: float) -> str:
    return ' '.join(f'{value:.6g}' for value in values)


def _rng(seed: int, *parts: str) -> random.Random:
    return random.Random(f'{seed}/{"|".join(parts)}')


def _sample(rng: random.Random, span: tuple[float, float]) -> float:
    return rng.uniform(span[0], span[1])


def _count(rng: random.Random, span: tuple[float, float]) -> int:
    return max(1, int(round(rng.uniform(span[0], span[1]))))


@dataclass(frozen=True)
class BagTarget:
    """一颗套袋桃：袋底（=入口）为局部原点，+Z = 轴（底→颈）."""

    target_id: str
    tree_id: str
    bottom: Vec3
    axis: Vec3
    bottom_to_neck: float
    body_diameter: float
    body_thickness: float
    body_length: float
    fruit_diameter: float
    neck_diameter: float
    neck_length: float
    tilt_yaw: float

    @property
    def neck(self) -> Vec3:
        return _vec_add(self.bottom, _vec_scale(self.axis, self.bottom_to_neck))

    @property
    def fruit_center(self) -> Vec3:
        return _vec_add(
            self.bottom, _vec_scale(self.axis, self.fruit_diameter * 0.5 - 0.004))


@dataclass(frozen=True)
class WorkPose:
    """采摘工位：base_link（臂座）与车体在果园世界系的位姿."""

    mount: Vec3
    mount_yaw: float
    vehicle_yaw: float
    vehicle_center: Vec3


@dataclass(frozen=True)
class Scene:
    """同源产物对：世界 SDF 文本 + 目标清单."""

    world_sdf: str
    manifest: dict


def work_pose(params: OrchardParams) -> WorkPose:
    """由 ``platform`` 参数推出工位位姿（行沿 Y，行距沿 X）."""
    rows, platform = params.rows, params.platform
    row_x = (platform.face_row_index - (rows.count - 1) / 2.0) * rows.spacing
    center = (row_x + platform.side * platform.row_gap, platform.y, 0.0)
    # 车体 +X 沿行方向行驶，行在车体 -Y 侧（side=+1 → 车在行的 +x 侧）
    vehicle_yaw = -platform.side * math.pi / 2.0
    mount_yaw = vehicle_yaw + math.radians(platform.arm_mount_yaw_deg)
    mount = (center[0], center[1], platform.arm_mount_height)
    return WorkPose(
        mount=mount, mount_yaw=mount_yaw, vehicle_yaw=vehicle_yaw,
        vehicle_center=center)


def tree_positions(params: OrchardParams) -> list[tuple[str, int, int, Vec3]]:
    """全部树位：(树 id, 行号, 株号, 树干根部世界坐标)."""
    rows = params.rows
    out: list[tuple[str, int, int, Vec3]] = []
    for row in range(rows.count):
        row_x = (row - (rows.count - 1) / 2.0) * rows.spacing
        for index in range(rows.trees_per_row):
            tree_y = (index - (rows.trees_per_row - 1) / 2.0) * rows.tree_spacing
            out.append((
                f'tree_r{row}_t{index}', row, index,
                (row_x, tree_y, 0.0),
            ))
    return out


def work_zone_tree_ids(params: OrchardParams) -> set[str]:
    """作业位沿行覆盖到的树（参与可达断言）."""
    platform, zone = params.platform, params.work_zone
    out: set[str] = set()
    for tree_id, row, _index, origin in tree_positions(params):
        if row != platform.face_row_index:
            continue
        if abs(origin[1] - platform.y) <= zone.y_span:
            out.add(tree_id)
    return out


def _element(parent: ET.Element, tag: str, text: str | None = None,
             **attrs: str) -> ET.Element:
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = text
    return node


# Blender 网格名义尺寸（blender/build_assets.py）：实例按实参非均匀缩放
MESH_BAG_W = 0.075
MESH_BAG_T = 0.052
MESH_BAG_L = 0.088
MESH_DIR = 'model://peach_sim/meshes'


def _visual_mesh(link: ET.Element, name: str, pose: tuple[float, ...],
                 uri: str, scale: Vec3) -> None:
    """网格视觉（材质随 GLB 内嵌 PBR），碰撞仍用基元保性能."""
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*pose))
    geometry = _element(visual, 'geometry')
    mesh = _element(geometry, 'mesh')
    _element(mesh, 'uri', uri)
    _element(mesh, 'scale', _fmt(*scale))


def _material_pbr(parent: ET.Element, texture: str, roughness: float) -> None:
    """
    PBR 贴图材质（albedo + 切线空间法线 + 粗糙度），贴图相对世界文件.

    ogre2 把 ``<diffuse>`` 当底色乘子（缺省 0 → 整面渲黑，实测最小复现坐实），
    因此这里给白底，颜色全部由 albedo 贴图承担。
    """
    material = _element(parent, 'material')
    _element(material, 'ambient', _fmt(1.0, 1.0, 1.0, 1.0))
    _element(material, 'diffuse', _fmt(1.0, 1.0, 1.0, 1.0))
    _element(material, 'specular', _fmt(0.1, 0.1, 0.1, 1.0))
    metal = _element(_element(material, 'pbr'), 'metal')
    _element(metal, 'albedo_map', f'textures/{texture}_albedo.png')
    _element(metal, 'normal_map', f'textures/{texture}_normal.png',
             type='tangent')
    _element(metal, 'roughness', _fmt(roughness))
    _element(metal, 'metalness', _fmt(0.0))


def _material(parent: ET.Element, rgb: Vec3) -> None:
    material = _element(parent, 'material')
    _element(material, 'ambient', _fmt(*rgb, 1.0))
    _element(material, 'diffuse', _fmt(*rgb, 1.0))
    _element(material, 'specular', _fmt(0.1, 0.1, 0.1, 1.0))


def _geometry(parent: ET.Element, kind: str, **values: float) -> ET.Element:
    geometry = _element(parent, 'geometry')
    shape = _element(geometry, kind)
    for key, value in values.items():
        _element(shape, key, _fmt(*value) if isinstance(value, tuple)
                 else _fmt(value))
    return geometry


def _visual_cylinder(link: ET.Element, name: str, center: Vec3,
                     direction: Vec3, length: float, radius: float,
                     rgb: Vec3, texture: str | None = None,
                     roughness: float = 0.85) -> None:
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*center, *aim_rpy(direction)))
    _geometry(visual, 'cylinder', radius=radius, length=length)
    if texture is None:
        _material(visual, rgb)
    else:
        _material_pbr(visual, texture, roughness)


def _visual_sphere(link: ET.Element, name: str, center: Vec3, radius: float,
                   rgb: Vec3, texture: str | None = None,
                   roughness: float = 0.85) -> None:
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*center, 0.0, 0.0, 0.0))
    _geometry(visual, 'sphere', radius=radius)
    if texture is None:
        _material(visual, rgb)
    else:
        _material_pbr(visual, texture, roughness)


def _visual_ellipsoid(link: ET.Element, name: str, pose: tuple[float, ...],
                      radii: Vec3, rgb: Vec3, texture: str | None = None,
                      roughness: float = 0.85) -> None:
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*pose))
    _geometry(visual, 'ellipsoid', radii=radii)
    if texture is None:
        _material(visual, rgb)
    else:
        _material_pbr(visual, texture, roughness)


def _visual_box(link: ET.Element, name: str, pose: tuple[float, ...],
                size: Vec3, rgb: Vec3, texture: str | None = None,
                roughness: float = 0.85) -> None:
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*pose))
    _geometry(visual, 'box', size=size)
    if texture is None:
        _material(visual, rgb)
    else:
        _material_pbr(visual, texture, roughness)


def _collision_cylinder(link: ET.Element, name: str, center: Vec3,
                        direction: Vec3, length: float,
                        radius: float) -> None:
    collision = _element(link, 'collision', name=f'c_{name}')
    _element(collision, 'pose', _fmt(*center, *aim_rpy(direction)))
    _geometry(collision, 'cylinder', radius=radius, length=length)


def _collision_sphere(link: ET.Element, name: str, center: Vec3,
                      radius: float) -> None:
    collision = _element(link, 'collision', name=f'c_{name}')
    _element(collision, 'pose', _fmt(*center, 0.0, 0.0, 0.0))
    _geometry(collision, 'sphere', radius=radius)


def _collision_box(link: ET.Element, name: str, pose: tuple[float, ...],
                   size: Vec3) -> None:
    collision = _element(link, 'collision', name=f'c_{name}')
    _element(collision, 'pose', _fmt(*pose))
    _geometry(collision, 'box', size=size)


def _static_model(world: ET.Element, name: str,
                  pose: tuple[float, ...] | None = None) -> ET.Element:
    model = _element(world, 'model', name=name)
    if pose is not None:
        _element(model, 'pose', _fmt(*pose))
    _element(model, 'static', 'true')
    return model


def _ground(params: OrchardParams, world: ET.Element) -> None:
    ground = params.ground
    model = _static_model(world, 'ground')
    link = _element(model, 'link', name='link')
    plane = _element(link, 'visual', name='soil')
    _element(plane, 'pose', _fmt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    _geometry(plane, 'plane', normal=(0.0, 0.0, 1.0),
              size=(ground.size_x, ground.size_y))
    _material_pbr(plane, 'soil', 0.95)
    collision = _element(link, 'collision', name='c_soil')
    _geometry(collision, 'plane', normal=(0.0, 0.0, 1.0),
              size=(ground.size_x, ground.size_y))

    # 行带：树行下保持裸土（除草带），行间与两侧铺草带
    cover = _static_model(world, 'ground_cover')
    cover_link = _element(cover, 'link', name='link')
    rows = params.rows
    half_span = (rows.count - 1) / 2.0 * rows.spacing
    inner_width = rows.spacing - ground.row_strip_width
    outer_width = (ground.size_x / 2.0 - half_span - ground.row_strip_width / 2.0)
    strips: list[tuple[float, float]] = [
        (-ground.size_x / 2.0 + outer_width / 2.0, outer_width),
        (ground.size_x / 2.0 - outer_width / 2.0, outer_width),
    ]
    for index in range(rows.count - 1):
        x = (index - (rows.count - 1) / 2.0) * rows.spacing + rows.spacing / 2.0
        strips.append((x, inner_width))
    for index, (x, width) in enumerate(strips):
        if width <= 0.05:
            continue
        _visual_box(
            cover_link, f'grass_{index}',
            (x, 0.0, 0.006, 0.0, 0.0, 0.0),
            (width, ground.size_y, 0.012), _GRASS, texture='grass',
            roughness=0.95)


def _trellis(params: OrchardParams, world: ET.Element) -> None:
    """果园立柱拉线：每行端柱/中柱 + 两道横线."""
    rows = params.rows
    row_length = (rows.trees_per_row - 1) * rows.tree_spacing
    post_height = 2.5
    for row in range(rows.count):
        row_x = (row - (rows.count - 1) / 2.0) * rows.spacing
        model = _static_model(world, f'trellis_row_{row}')
        link = _element(model, 'link', name='link')
        # 立柱落在株间中点（不与树干重合），端柱外移半株距
        post_ys = [(-0.5 - (rows.trees_per_row - 1) / 2.0) * rows.tree_spacing,
                   ((rows.trees_per_row - 1) / 2.0 + 0.5) * rows.tree_spacing]
        post_ys += [
            (index - (rows.trees_per_row - 1) / 2.0) * rows.tree_spacing
            + rows.tree_spacing / 2.0
            for index in range(0, rows.trees_per_row - 1, 3)
        ]
        for index, y in enumerate(post_ys):
            _visual_cylinder(
                link, f'post_{index}', (row_x, y, post_height / 2.0),
                (0.0, 0.0, 1.0), post_height, 0.045, _POST)
            _collision_cylinder(
                link, f'post_{index}', (row_x, y, post_height / 2.0),
                (0.0, 0.0, 1.0), post_height, 0.045)
        for index, height in enumerate((1.8, 2.3)):
            _visual_cylinder(
                link, f'wire_{index}', (row_x, 0.0, height),
                (0.0, 1.0, 0.0), row_length, 0.006, _WIRE)


def _bag_targets(params: OrchardParams, tree_id: str, origin: Vec3,
                 alley_dir: Vec3) -> list[BagTarget]:
    """
    一株树的挂袋果：**沿果枝串挂**（位置关系锚真实 3D 数据）.

    PeachDataSet 深度反投影实测（n=4074）：3D 袋间距中位 110mm（p10 10 /
    p90 249mm），同帧袋数中位 10 成串——即袋沿果枝串生，既有贴袋也有散开。
    故每树 2–3 簇（一条果枝=一簇），簇内沿枝向步进串挂、逐袋步距 50–160mm，
    并守车体安全带（袋到树行中心线径向 0.22–0.55 m）。
    """
    bag = params.bag
    rng = _rng(params.seed, 'bag', tree_id)
    targets: list[BagTarget] = []
    count = _count(rng, bag.per_tree)
    clusters = _count(rng, bag.cluster_count)
    row_dir = (0.0, 1.0, 0.0)
    for cluster in range(clusters):
        share = max(2, count // clusters + (1 if cluster < count % clusters else 0))
        on_alley = rng.random() < bag.alley_side_ratio
        radial = _sample(rng, (0.22, 0.55)) * (1.0 if on_alley else -0.35)
        lateral = rng.uniform(-bag.row_spread, bag.row_spread)
        anchor_z = _sample(rng, bag.bottom_height)
        # 果枝方向：沿行为主、略朝作业道、略上翘
        branch_dir = _vec_norm((
            row_dir[0] * rng.uniform(-1.0, 1.0) + alley_dir[0] * rng.uniform(-0.6, 0.6),
            row_dir[1] * rng.uniform(-1.0, 1.0) + alley_dir[1] * rng.uniform(-0.6, 0.6),
            rng.uniform(-0.25, 0.35)))
        step = 0.0
        for _ in range(share):
            if len(targets) >= count:
                break
            # 步距按实测 3D 最近袋间距分布（p10=3mm/中位 111/p90 252）混合采样：
            # 14% 贴袋对（<30mm）+ 86% 散袋（70–220mm）→ 长尾对齐
            if rng.random() < 0.14:
                step += rng.uniform(0.005, 0.030)
            else:
                step += rng.uniform(0.095, 0.270)
            jitter = rng.uniform(-0.03, 0.03)
            lateral_now = lateral + step * branch_dir[1] + jitter
            # 车体安全带：袋到树行中心线的径向距离限 0.22–0.55 m
            radial_now = max(0.22, min(0.55, radial + step * branch_dir[0]))
            # 袋底高度钳制在现场窗（runs/field_pregrasp_*：base_link+0.52~0.71）
            z_now = anchor_z + step * branch_dir[2] + rng.uniform(-0.03, 0.03)
            bottom = (
                origin[0] + alley_dir[0] * radial_now + row_dir[0] * lateral_now,
                origin[1] + alley_dir[1] * radial_now + row_dir[1] * lateral_now,
                max(1.27, min(1.45, z_now)),
            )
            body_diameter = _sample(rng, bag.body_diameter)
            fruit_hi = min(bag.fruit_diameter[1], body_diameter - 0.004)
            fruit_diameter = rng.uniform(bag.fruit_diameter[0], fruit_hi)
            bottom_to_neck = _sample(rng, bag.bottom_to_neck)
            # 吊挂角 θ = cap·u^1.9：中位≈12.5°（实测 13.2°）、上限=实测 p90
            magnitude = math.radians(bag.tilt_max_deg) * rng.random() ** 1.9
            tilt_yaw = rng.uniform(0.0, 2.0 * math.pi)
            axis = _vec_norm((
                math.sin(magnitude) * math.cos(tilt_yaw),
                math.sin(magnitude) * math.sin(tilt_yaw),
                math.cos(magnitude)))
            target = BagTarget(
                target_id=f'peach_{tree_id}_{len(targets):02d}',
                tree_id=tree_id,
                bottom=bottom,
                axis=axis,
                bottom_to_neck=bottom_to_neck,
                body_diameter=body_diameter,
                body_thickness=body_diameter * _sample(
                    rng, bag.body_thickness_ratio),
                body_length=bottom_to_neck - bag.neck_length,
                fruit_diameter=fruit_diameter,
                neck_diameter=bag.neck_diameter,
                neck_length=bag.neck_length,
                tilt_yaw=aim_rpy(axis)[2],
            )
            targets.append(target)
    return targets


def _tree(params: OrchardParams, world: ET.Element, tree_id: str,
          origin: Vec3, alley_dir: Vec3) -> list[BagTarget]:
    """
    一株桃树（低模开心形）.

    树形参数取自桃园树体结构规范：干高约 0.6 m、2–3 主枝开角 45–60°、
    枝组与主枝夹角 ≥75°、树高 ≤2.5 m；主枝方位偏向行向（拉枝后近同一平面）。
    枝干用锥台堆叠出收分并挂树皮贴图，冠层是多枚叶幕球（叶簇贴图）。
    """
    tree = params.tree
    rng = _rng(params.seed, 'tree', tree_id)
    model = _static_model(world, tree_id, pose=(*origin, 0.0, 0.0, 0.0))
    link = _element(model, 'link', name='link')

    trunk_height = _sample(rng, tree.trunk_height)
    trunk_radius = _sample(rng, tree.trunk_radius)
    variant = 'a' if int(tree_id.split('_t')[-1]) % 2 == 0 else 'b'
    scale = 0.92 + 0.16 * rng.random()
    _visual_mesh(link, 'tree_mesh', (0.0, 0.0, 0.0, 0.0, 0.0,
                                     rng.uniform(-0.35, 0.35)),
                 f'{MESH_DIR}/peach_tree_{variant}.glb',
                 (scale, scale, scale))
    # 主干收分两段 + 根颈外扩
    for index, (height, radius, base) in enumerate((
            (trunk_height * 0.55, trunk_radius, 0.0),
            (trunk_height * 0.45, trunk_radius * 0.72, trunk_height * 0.55),
    )):
        _visual_cylinder(link, f'trunk_{index}',
                         (0.0, 0.0, base + height / 2.0), (0.0, 0.0, 1.0),
                         height, radius, _BARK, texture='bark', roughness=0.9)
        _collision_cylinder(link, f'trunk_{index}',
                            (0.0, 0.0, base + height / 2.0), (0.0, 0.0, 1.0),
                            height, radius)
    _visual_cylinder(link, 'root_flare', (0.0, 0.0, 0.035),
                     (0.0, 0.0, 1.0), 0.07, trunk_radius * 1.45, _BARK,
                     texture='bark', roughness=0.95)

    # 主枝（开心形）：方位偏向行向，开角 45–60°
    scaffold_rng = _rng(params.seed, 'scaffold', tree_id)
    ends: list[Vec3] = []
    count = _count(scaffold_rng, tree.scaffold_count)
    base_azimuth = scaffold_rng.uniform(-0.35, 0.35) + math.pi / 2.0
    for index in range(count):
        side = 1.0 if index % 2 == 0 else -1.0
        azimuth = base_azimuth + side * (0.5 + 0.35 * (index // 2)) * (
            1.0 if index else 0.0) + scaffold_rng.uniform(-0.2, 0.2)
        polar = math.radians(_sample(scaffold_rng, tree.scaffold_tilt_deg))
        direction = _vec_norm((
            math.sin(polar) * math.cos(azimuth),
            math.sin(polar) * math.sin(azimuth),
            math.cos(polar)))
        length = _sample(scaffold_rng, tree.scaffold_length)
        start = (0.0, 0.0, trunk_height * 0.92)
        end = _vec_add(start, _vec_scale(direction, length))
        ends.append(end)
        middle = _vec_add(start, _vec_scale(direction, length / 2.0))
        _visual_cylinder(link, f'scaffold_{index}', middle, direction, length,
                         tree.scaffold_radius, _BARK, texture='bark',
                         roughness=0.9)
        _collision_cylinder(link, f'scaffold_{index}', middle, direction,
                            length, tree.scaffold_radius)

        # 枝组：与主枝夹角 ≥75°（近垂直的果枝群）
        for branchlet in range(_count(scaffold_rng, (2.0, 3.0))):
            anchor = _vec_add(start, _vec_scale(
                direction, length * scaffold_rng.uniform(0.45, 0.9)))
            shoot_dir = _vec_norm((
                direction[0] * 0.25 + scaffold_rng.uniform(-0.6, 0.6),
                direction[1] * 0.25 + scaffold_rng.uniform(-0.6, 0.6),
                scaffold_rng.uniform(-0.15, 0.75)))
            shoot_len = _sample(scaffold_rng, tree.branchlet_length)
            _visual_cylinder(
                link, f'branchlet_{index}_{branchlet}',
                _vec_add(anchor, _vec_scale(shoot_dir, shoot_len / 2.0)),
                shoot_dir, shoot_len, tree.branchlet_radius, _TWIG,
                texture='bark', roughness=0.9)

    # 冠层：叶幕球串在枝组梢端一带（叶簇贴图）
    canopy_rng = _rng(params.seed, 'canopy', tree_id)
    for index in range(_count(canopy_rng, tree.canopy_count)):
        anchor = ends[index % len(ends)]
        center = (
            anchor[0] * 0.7 + canopy_rng.uniform(-0.3, 0.3),
            anchor[1] * 0.85 + canopy_rng.uniform(-0.4, 0.4),
            _sample(canopy_rng, tree.canopy_height),
        )
        radius = _sample(canopy_rng, tree.canopy_radius)
        rgb = _LEAF if index % 2 else _LEAF_LIGHT
        _visual_sphere(link, f'canopy_{index}', center, radius, rgb,
                       texture='leaf', roughness=0.85)
        if index < 3:
            _collision_sphere(link, f'canopy_{index}', center, radius)

    # 挂袋果枝：袋颈在枝梢，袋体垂在枝下
    targets = _bag_targets(params, tree_id, origin, alley_dir)
    # 叶层包住挂袋（实测袋-叶层深度差中位仅 9mm：袋嵌在叶丛里）——
    # 沿每个簇的袋位补叶幕球，让前景/背景都有叶
    for index, target in enumerate(targets):
        if index % 4:   # 真袋周叶占比仅 0.19：叶幕稀疏贴袋，不密裹
            continue
        centre = (
            target.bottom[0] - origin[0] + target.axis[0] * 0.05
            + rng.uniform(-0.10, 0.10),
            target.bottom[1] - origin[1] + target.axis[1] * 0.05
            + rng.uniform(-0.10, 0.10),
            target.bottom[2] - origin[2] + rng.uniform(-0.08, 0.10),
        )
        radius = rng.uniform(0.08, 0.13)
        _visual_sphere(link, f'foliage_{index}', centre, radius, _LEAF,
                       texture='leaf', roughness=0.85)
    twig_rng = _rng(params.seed, 'twig', tree_id)
    for index, target in enumerate(targets):
        neck_local = (
            target.bottom[0] - origin[0] + target.axis[0] * target.bottom_to_neck,
            target.bottom[1] - origin[1] + target.axis[1] * target.bottom_to_neck,
            target.bottom[2] - origin[2] + target.axis[2] * target.bottom_to_neck,
        )
        twig_dir = _vec_norm((
            -alley_dir[0] * 0.4 + twig_rng.uniform(-0.3, 0.3),
            -alley_dir[1] * 0.4 + twig_rng.uniform(-0.3, 0.3),
            0.8,
        ))
        twig_length = twig_rng.uniform(0.10, 0.22)
        twig_start = _vec_add(neck_local, _vec_scale(twig_dir, twig_length))
        _visual_cylinder(
            link, f'twig_{index}',
            _vec_add(twig_start, _vec_scale(twig_dir, -twig_length / 2.0)),
            twig_dir, twig_length, tree.twig_radius, _TWIG,
            texture='bark', roughness=0.9)
    return targets


def _bagged_peach(world: ET.Element, target: BagTarget,
                  pose: tuple[float, ...]) -> None:
    """
    一颗套袋桃（局部系：原点=袋底，+Z=底→颈）.

    构造对齐实物（专利 CN201025802Y 与桃果套袋栽培规范）：袋体=撑开的扁筒、
    袋底立体折边、袋口聚集后**封口铁丝**缠绕扎在枝上、内衬遮光纸层在袋口露出、
    果实在袋内膨起把袋底顶出弧形。透孔/针孔/排水孔画进纸张贴图。
    """
    model = _static_model(world, target.target_id, pose=pose)
    link = _element(model, 'link', name='bag')
    body_length = target.body_length
    half_w = target.body_diameter / 2.0
    half_t = target.body_thickness / 2.0

    # 袋体整体网格（Blender 建模：皱褶袋体/袋底折边/收拢颈/铁丝/内衬/果），
    # 名义尺寸按 config 区间中值，实例做非均匀缩放对齐各自实参
    paper_length = target.bottom_to_neck + target.neck_length
    _visual_mesh(
        link, 'bag_mesh',
        (0.0, 0.0, paper_length / 2.0, 0.0, 0.0, target.tilt_yaw * 0.0),
        f'{MESH_DIR}/bagged_peach.glb',
        (target.body_diameter / MESH_BAG_W,
         target.body_thickness / MESH_BAG_T,
         paper_length / MESH_BAG_L))
    # 袋体：扁筒被果实撑起（纸张贴图带折痕/透孔/针孔）
    _visual_ellipsoid(
        link, 'body', (0.0, 0.0, body_length / 2.0, 0.0, 0.0, 0.0),
        (half_w, half_t, body_length * 0.54), _PAPER,
        texture='paper_bag', roughness=0.72)
    # 袋底立体折边：横向压痕接缝
    _visual_box(
        link, 'bottom_fold', (0.0, 0.0, 0.012, 0.0, 0.0, 0.35),
        (half_w * 1.55, half_t * 1.15, 0.022), PAPER_DARK,
        texture='paper_bag', roughness=0.8)
    # 果实膨起：把袋底顶出弧形（果皮贴图，露出袋口下沿一点更像实物）
    _visual_sphere(
        link, 'fruit', (0.0, 0.0, target.fruit_diameter * 0.5 - 0.006),
        target.fruit_diameter * 0.52, _FRUIT,
        texture='fruit_skin', roughness=0.55)
    # 袋口聚集：锥形收拢到袋颈
    _visual_cylinder(
        link, 'neck', (0.0, 0.0, body_length + target.neck_length / 2.0),
        (0.0, 0.0, 1.0), target.neck_length, target.neck_diameter / 2.0,
        _PAPER_NECK, texture='paper_bag', roughness=0.75)
    gather = target.neck_diameter / 2.0 + 0.006
    _visual_cylinder(
        link, 'gather', (0.0, 0.0, body_length - 0.010),
        (0.0, 0.0, 1.0), 0.020, gather, PAPER_DARK,
        texture='paper_bag', roughness=0.8)
    # 内衬遮光纸层：袋口露出的深色内圈
    _visual_cylinder(
        link, 'inner_paper', (0.0, 0.0, body_length + target.neck_length
                              - 0.004),
        (0.0, 0.0, 1.0), 0.008, target.neck_diameter / 2.0 - 0.003, INNER_PAPER)
    # 封口铁丝：两圈缠绕 + 翘起的线尾
    for index in range(2):
        _visual_cylinder(
            link, f'wire_{index}',
            (0.0, 0.0, body_length + 0.004 + index * 0.007),
            (0.0, 0.0, 1.0), 0.0035, gather + 0.0015, _TIE)
    _visual_cylinder(
        link, 'wire_tail',
        (0.010, 0.0, body_length + 0.012), (0.5, 0.0, 0.87), 0.026,
        0.0012, _TIE)
    # 碰撞用圆柱包络（扁平袋体的保守近似，插入/剪切接触留待物理仿真轮）
    _collision_cylinder(
        link, 'body', (0.0, 0.0, body_length / 2.0), (0.0, 0.0, 1.0),
        body_length, half_w)


def _sun_direction(params: OrchardParams) -> Vec3:
    light = params.lighting
    azimuth = math.radians(light.sun_azimuth_deg)
    elevation = math.radians(light.sun_elevation_deg)
    # 光线方向（太阳→地面）：由方位角/高度角反推
    return (-math.cos(elevation) * math.cos(azimuth),
            -math.cos(elevation) * math.sin(azimuth),
            -math.sin(elevation))


def _world_shell(params: OrchardParams, world: ET.Element) -> None:
    light = params.lighting
    physics = _element(world, 'physics', name='4ms', type='ignored')
    _element(physics, 'max_step_size', _fmt(0.004))
    _element(physics, 'real_time_factor', _fmt(1.0))
    for name, filename in (
            ('gz::sim::systems::Physics', 'gz-sim-physics-system'),
            ('gz::sim::systems::UserCommands', 'gz-sim-user-commands-system'),
            ('gz::sim::systems::SceneBroadcaster',
             'gz-sim-scene-broadcaster-system')):
        _element(world, 'plugin', name=name, filename=filename)

    scene = _element(world, 'scene')
    _element(scene, 'ambient', _fmt(*light.ambient, 1.0))
    _element(scene, 'background', _fmt(*light.background, 1.0))
    _element(scene, 'shadows', 'true' if light.shadows else 'false')
    sky = _element(scene, 'sky')
    clouds = _element(sky, 'clouds')
    _element(clouds, 'speed', _fmt(4.0))
    _element(clouds, 'direction', _fmt(1.2))
    _element(clouds, 'humidity', _fmt(0.6))
    _element(clouds, 'mean_size', _fmt(0.8))
    fog = _element(scene, 'fog')
    _element(fog, 'density', _fmt(light.fog_density))
    _element(fog, 'color', _fmt(0.85, 0.88, 0.92, 1.0))

    # 打开 gz GUI 即全景：相机在果园南侧高处看向中心（X 前、Z 上的相机系）
    gui = _element(world, 'gui')
    camera = _element(gui, 'camera', name='user_camera')
    _element(camera, 'pose', _fmt(
        0.0, -0.55 * params.ground.size_y, 0.45 * params.ground.size_x,
        0.0, 0.42, 1.5708))

    sun = _element(world, 'light', name='sun', type='directional')
    _element(sun, 'cast_shadows', 'true' if light.shadows else 'false')
    _element(sun, 'pose', _fmt(0.0, 0.0, 30.0, 0.0, 0.0, 0.0))
    _element(sun, 'diffuse', _fmt(1.0, 0.97, 0.92, 1.0))
    # 柔光（真图袋/区域亮度比 0.84）；SDF 颜色限 [0,1]，亮度走 intensity
    _element(sun, 'intensity', _fmt(4.0))
    _element(sun, 'specular', _fmt(0.3, 0.3, 0.3, 1.0))
    attenuation = _element(sun, 'attenuation')
    _element(attenuation, 'range', _fmt(1000.0))
    _element(attenuation, 'constant', _fmt(0.9))
    _element(attenuation, 'linear', _fmt(0.01))
    _element(attenuation, 'quadratic', _fmt(0.001))
    _element(sun, 'direction', _fmt(*_sun_direction(params)))


def _manifest(params: OrchardParams, targets: list[BagTarget], work: WorkPose,
              world_text: str) -> dict:
    zone_ids = work_zone_tree_ids(params)
    entries: list[dict] = []
    reachable = 0
    for target in targets:
        distance = _vec_dist(work.mount, target.neck)
        is_reachable = distance <= params.work_zone.reach_max_m
        reachable += int(is_reachable)
        entries.append({
            'id': target.target_id,
            'tree': target.tree_id,
            'work_zone': target.tree_id in zone_ids,
            'bag_bottom': [round(v, 6) for v in target.bottom],
            'entry': [round(v, 6) for v in target.bottom],
            'bag_neck': [round(v, 6) for v in target.neck],
            'axis': [round(v, 6) for v in target.axis],
            'fruit_center': [round(v, 6) for v in target.fruit_center],
            'fruit_diameter': round(target.fruit_diameter, 6),
            'body_diameter': round(target.body_diameter, 6),
            'body_thickness': round(target.body_thickness, 6),
            'bottom_to_neck': round(target.bottom_to_neck, 6),
            'neck_diameter': target.neck_diameter,
            'sdf_pose': [round(v, 6) for v in (
                *target.bottom, 0.0, aim_rpy(target.axis)[1],
                target.tilt_yaw)],
            'dist_to_mount': round(distance, 6),
            'reachable': is_reachable,
        })
    return {
        'schema': MANIFEST_SCHEMA,
        'world': 'worlds/peach_orchard.sdf',
        'world_sha256': hashlib.sha256(world_text.encode('utf-8')).hexdigest(),
        'seed': params.seed,
        'tool_profile': params.tool.profile_id,
        'frame': 'world（REP-103：z 上，树行沿 Y）',
        'entry_note': 'entry = bag_bottom（grasp_standoffs.yaml entry_standoff_m=0）',
        'platform': {
            'model_name': PLATFORM_MODEL,
            # gz 模型原点 = URDF `world` = base_link（臂座），world_joint 恒等
            'spawn_pose': [round(v, 6) for v in (
                *work.mount, 0.0, 0.0, work.mount_yaw)],
            'arm_mount': [round(v, 6) for v in work.mount],
            'vehicle_center': [round(v, 6) for v in work.vehicle_center],
            'vehicle_yaw_deg': round(math.degrees(work.vehicle_yaw), 6),
            'arm_mount_yaw_deg': round(math.degrees(work.mount_yaw), 6),
            'reach_max_m': params.work_zone.reach_max_m,
        },
        'counts': {
            'rows': params.rows.count,
            'trees': params.rows.count * params.rows.trees_per_row,
            'targets': len(entries),
            'reachable': reachable,
            'work_zone_trees': len(zone_ids),
            'work_zone_targets': sum(1 for e in entries if e['work_zone']),
        },
        'targets': entries,
    }


def render_scene(params: OrchardParams) -> Scene:
    """一次布局计算，产出世界 SDF 与目标清单（同源、确定性）."""
    problems = check_params(params)
    if problems:
        raise ValueError('场景参数不合法：\n- ' + '\n- '.join(problems))

    work = work_pose(params)
    alley_sign = float(params.platform.side)
    root = ET.Element('sdf', {'version': SDF_VERSION})
    world = ET.SubElement(root, 'world', {'name': WORLD_NAME})
    _world_shell(params, world)
    _ground(params, world)
    _trellis(params, world)

    targets: list[BagTarget] = []
    for tree_id, _row, _index, origin in tree_positions(params):
        alley_dir = (alley_sign, 0.0, 0.0)
        targets.extend(_tree(params, world, tree_id, origin, alley_dir))

    for target in targets:
        pose = (*target.bottom, 0.0, aim_rpy(target.axis)[1], target.tilt_yaw)
        _bagged_peach(world, target, pose)

    ET.indent(root, space='  ')
    world_text = ET.tostring(root, encoding='unicode')
    world_text = '<?xml version="1.0"?>\n' + world_text + '\n'
    return Scene(
        world_sdf=world_text,
        manifest=_manifest(params, targets, work, world_text),
    )


def build_world(params: OrchardParams) -> str:
    """只取世界 SDF 文本."""
    return render_scene(params).world_sdf


def build_manifest(params: OrchardParams) -> dict:
    """只取目标清单."""
    return render_scene(params).manifest
