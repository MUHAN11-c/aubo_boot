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

SDF_VERSION = '1.11'
WORLD_NAME = 'peach_orchard'
PLATFORM_MODEL = 'harvester'
MANIFEST_SCHEMA = 'peach_sim/orchard_manifest/v1'

# 材质（ambient/diffuse/specular 同色，低模不追光照贴图）
_SOIL = (0.36, 0.29, 0.21)
_GRASS = (0.30, 0.40, 0.19)
_BARK = (0.34, 0.25, 0.16)
_TWIG = (0.30, 0.23, 0.14)
_LEAF = (0.24, 0.38, 0.16)
_LEAF_LIGHT = (0.32, 0.46, 0.20)
_PAPER = (0.84, 0.76, 0.56)
_PAPER_NECK = (0.78, 0.70, 0.50)
_TIE = (0.25, 0.25, 0.25)
_FRUIT = (0.87, 0.42, 0.30)
_POST = (0.62, 0.62, 0.60)
_WIRE = (0.45, 0.45, 0.45)


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
                     rgb: Vec3) -> None:
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*center, *aim_rpy(direction)))
    _geometry(visual, 'cylinder', radius=radius, length=length)
    _material(visual, rgb)


def _visual_sphere(link: ET.Element, name: str, center: Vec3, radius: float,
                   rgb: Vec3) -> None:
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*center, 0.0, 0.0, 0.0))
    _geometry(visual, 'sphere', radius=radius)
    _material(visual, rgb)


def _visual_ellipsoid(link: ET.Element, name: str, pose: tuple[float, ...],
                      radii: Vec3, rgb: Vec3) -> None:
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*pose))
    _geometry(visual, 'ellipsoid', radii=radii)
    _material(visual, rgb)


def _visual_box(link: ET.Element, name: str, pose: tuple[float, ...],
                size: Vec3, rgb: Vec3) -> None:
    visual = _element(link, 'visual', name=name)
    _element(visual, 'pose', _fmt(*pose))
    _geometry(visual, 'box', size=size)
    _material(visual, rgb)


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
    _material(plane, ground.soil_rgb)
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
            (width, ground.size_y, 0.012), _GRASS)


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
    """一株树的挂袋果：袋挂在作业道侧果枝末端，自然下垂（轴=底→颈）."""
    bag = params.bag
    rng = _rng(params.seed, 'bag', tree_id)
    targets: list[BagTarget] = []
    count = _count(rng, bag.per_tree)
    row_dir = (0.0, 1.0, 0.0)
    tilt = math.tan(math.radians(bag.tilt_max_deg))
    for index in range(count):
        body_diameter = _sample(rng, bag.body_diameter)
        fruit_hi = min(bag.fruit_diameter[1], body_diameter - 0.004)
        fruit_diameter = rng.uniform(bag.fruit_diameter[0], fruit_hi)
        bottom_to_neck = _sample(rng, bag.bottom_to_neck)
        on_alley = rng.random() < bag.alley_side_ratio
        radial = _sample(rng, (0.20, 0.55)) * (1.0 if on_alley else -0.35)
        lateral = rng.uniform(-bag.row_spread, bag.row_spread)
        bottom = _vec_add(
            origin,
            _vec_add(_vec_scale(alley_dir, radial),
                     _vec_scale(row_dir, lateral)))
        bottom = (bottom[0], bottom[1], _sample(rng, bag.bottom_height))
        axis = _vec_norm((
            rng.uniform(-tilt, tilt), rng.uniform(-tilt, tilt), 1.0))
        target = BagTarget(
            target_id=f'peach_{tree_id}_{index:02d}',
            tree_id=tree_id,
            bottom=bottom,
            axis=axis,
            bottom_to_neck=bottom_to_neck,
            body_diameter=body_diameter,
            body_thickness=body_diameter * _sample(rng, bag.body_thickness_ratio),
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
    tree = params.tree
    rng = _rng(params.seed, 'tree', tree_id)
    model = _static_model(world, tree_id, pose=(*origin, 0.0, 0.0, 0.0))
    link = _element(model, 'link', name='link')

    trunk_height = _sample(rng, tree.trunk_height)
    trunk_radius = _sample(rng, tree.trunk_radius)
    _visual_cylinder(link, 'trunk', (0.0, 0.0, trunk_height / 2.0),
                     (0.0, 0.0, 1.0), trunk_height, trunk_radius, _BARK)
    _collision_cylinder(link, 'trunk', (0.0, 0.0, trunk_height / 2.0),
                        (0.0, 0.0, 1.0), trunk_height, trunk_radius)

    # 主枝（开心形）：绕干散开，梢端挂冠层球
    scaffold_rng = _rng(params.seed, 'scaffold', tree_id)
    ends: list[Vec3] = []
    count = _count(scaffold_rng, tree.scaffold_count)
    base_azimuth = scaffold_rng.uniform(0.0, math.pi)
    for index in range(count):
        azimuth = base_azimuth + index * 2.0 * math.pi / count
        tilt_deg = _sample(scaffold_rng, tree.scaffold_tilt_deg)
        polar = math.radians(tilt_deg)
        direction = _vec_norm((
            math.sin(polar) * math.cos(azimuth),
            math.sin(polar) * math.sin(azimuth),
            math.cos(polar)))
        length = _sample(scaffold_rng, tree.scaffold_length)
        start = (0.0, 0.0, trunk_height * 0.95)
        end = _vec_add(start, _vec_scale(direction, length))
        ends.append(end)
        _visual_cylinder(link, f'scaffold_{index}', _vec_add(
            start, _vec_scale(direction, length / 2.0)), direction, length,
            tree.scaffold_radius, _BARK)
        _collision_cylinder(link, f'scaffold_{index}', _vec_add(
            start, _vec_scale(direction, length / 2.0)), direction, length,
            tree.scaffold_radius)

    # 冠层：沿行方向（世界 Y）串球，形成篱壁状果墙
    canopy_rng = _rng(params.seed, 'canopy', tree_id)
    for index in range(_count(canopy_rng, tree.canopy_count)):
        anchor = ends[index % len(ends)]
        center = (
            anchor[0] * 0.6 + canopy_rng.uniform(-0.25, 0.25),
            anchor[1] * 0.8 + canopy_rng.uniform(-0.35, 0.35),
            _sample(canopy_rng, tree.canopy_height),
        )
        radius = _sample(canopy_rng, tree.canopy_radius)
        rgb = _LEAF if index % 2 else _LEAF_LIGHT
        _visual_sphere(link, f'canopy_{index}', center, radius, rgb)
        if index < 3:
            _collision_sphere(link, f'canopy_{index}', center, radius)

    # 挂袋果枝：袋颈在枝梢，袋体垂在枝下
    targets = _bag_targets(params, tree_id, origin, alley_dir)
    for index, target in enumerate(targets):
        neck_local = (
            target.bottom[0] - origin[0] + target.axis[0] * target.bottom_to_neck,
            target.bottom[1] - origin[1] + target.axis[1] * target.bottom_to_neck,
            target.bottom[2] - origin[2] + target.axis[2] * target.bottom_to_neck,
        )
        twig_dir = _vec_norm((
            -alley_dir[0] * 0.4 + _sample(rng, (-0.3, 0.3)),
            -alley_dir[1] * 0.4 + _sample(rng, (-0.3, 0.3)),
            0.8,
        ))
        twig_length = _sample(rng, (0.10, 0.22))
        twig_start = _vec_add(neck_local, _vec_scale(twig_dir, twig_length))
        _visual_cylinder(
            link, f'twig_{index}',
            _vec_add(twig_start, _vec_scale(twig_dir, -twig_length / 2.0)),
            twig_dir, twig_length, tree.twig_radius, _TWIG)
    return targets


def _bagged_peach(world: ET.Element, target: BagTarget,
                  pose: tuple[float, ...]) -> None:
    model = _static_model(world, target.target_id, pose=pose)
    link = _element(model, 'link', name='bag')
    body_length = target.body_length
    _visual_ellipsoid(
        link, 'body', (0.0, 0.0, body_length / 2.0, 0.0, 0.0, 0.0),
        (target.body_diameter / 2.0, target.body_thickness / 2.0,
         body_length * 0.52),
        _PAPER)
    _visual_sphere(
        link, 'fruit', (0.0, 0.0, target.fruit_diameter * 0.5 - 0.004),
        target.fruit_diameter / 2.0, _FRUIT)
    _visual_cylinder(
        link, 'neck', (0.0, 0.0, body_length + target.neck_length / 2.0),
        (0.0, 0.0, 1.0), target.neck_length, target.neck_diameter / 2.0,
        _PAPER_NECK)
    _visual_cylinder(
        link, 'tie', (0.0, 0.0, body_length + 0.012), (0.0, 0.0, 1.0),
        0.008, target.neck_diameter / 2.0 + 0.005, _TIE)
    # 碰撞用圆柱包络（扁平袋体的保守近似，插入/剪切接触留待物理仿真轮）
    _collision_cylinder(
        link, 'body', (0.0, 0.0, body_length / 2.0), (0.0, 0.0, 1.0),
        body_length, target.body_diameter / 2.0)


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

    sun = _element(world, 'light', name='sun', type='directional')
    _element(sun, 'cast_shadows', 'true' if light.shadows else 'false')
    _element(sun, 'pose', _fmt(0.0, 0.0, 30.0, 0.0, 0.0, 0.0))
    _element(sun, 'diffuse', _fmt(0.95, 0.93, 0.88, 1.0))
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
