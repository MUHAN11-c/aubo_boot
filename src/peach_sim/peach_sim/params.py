"""
果园场景参数 schema 与校验（零 ROS import）.

``config/orchard.yaml`` 是部署事实源（全量清单）；本模块是唯一 schema：
``params_from_dict`` 逐键构造不可变 dataclass，未知键与越界值一律收集进错误表，
``check_params`` 汇总**全部**问题后由调用方抛 ``ValueError``（非法参数拒绝生成）。

区间键（如 ``bag.bottom_to_neck``）接受标量（等价于 [v, v]）或二元列表 [lo, hi]，
生成期在区间内均匀采样。所有长度单位为米，角度为度。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

Vec3 = tuple[float, float, float]
Range = tuple[float, float]

_TOP_KEYS = (
    'seed', 'ground', 'rows', 'tree', 'bag', 'tool', 'platform', 'work_zone',
    'lighting',
)


@dataclass(frozen=True)
class Ground:
    """地面与行带."""

    size_x: float
    size_y: float
    alley_grass_width: float
    row_strip_width: float
    soil_rgb: Vec3
    grass_rgb: Vec3


@dataclass(frozen=True)
class Rows:
    """树行布局：行沿世界 Y 延伸，行距沿 X."""

    count: int
    spacing: float
    trees_per_row: int
    tree_spacing: float


@dataclass(frozen=True)
class Tree:
    """桃树低模（主干 + 主枝 + 冠层球 + 果枝）."""

    trunk_height: Range
    trunk_radius: Range
    scaffold_count: Range
    scaffold_length: Range
    scaffold_radius: float
    scaffold_tilt_deg: Range
    branchlet_count: Range
    branchlet_length: Range
    branchlet_radius: float
    twig_radius: float
    canopy_count: Range
    canopy_radius: Range
    canopy_height: Range
    height_max: float


@dataclass(frozen=True)
class Bag:
    """套袋桃（袋体 + 果 + 袋颈/扎口）."""

    fruit_diameter: Range
    body_diameter: Range
    body_thickness_ratio: Range
    bottom_to_neck: Range
    neck_diameter: float
    neck_length: float
    per_tree: Range
    cluster_count: Range
    cluster_radius: Range
    cluster_gap: float
    bottom_height: Range
    tilt_max_deg: float
    row_spread: float
    alley_side_ratio: float


@dataclass(frozen=True)
class Tool:
    """末端工具几何（镜像 aubo_description/config/<profile>.yaml，测试对账）."""

    profile_id: str
    d_inner: float
    l_insert: float
    clearance_min: float


@dataclass(frozen=True)
class Platform:
    """采摘工位：履带底盘 + AUBO E5 安装座."""

    face_row_index: int
    side: int
    row_gap: float
    y: float
    arm_mount_height: float
    arm_mount_yaw_deg: float
    body_size: Vec3
    body_bottom: float
    track_length: float
    track_width: float
    track_height: float
    track_separation: float


@dataclass(frozen=True)
class WorkZone:
    """作业位可达性粗判（球形包络，非 IK）."""

    reach_max_m: float
    y_span: float


@dataclass(frozen=True)
class Lighting:
    """室外光照：太阳 + 天空 + 薄雾."""

    sun_azimuth_deg: float
    sun_elevation_deg: float
    ambient: Vec3
    background: Vec3
    fog_density: float
    shadows: bool


@dataclass(frozen=True)
class OrchardParams:
    """场景全量参数."""

    seed: int
    ground: Ground
    rows: Rows
    tree: Tree
    bag: Bag
    tool: Tool
    platform: Platform
    work_zone: WorkZone
    lighting: Lighting


class _Reader:
    """按路径收集键级问题；一路读到底，不遇错即停."""

    def __init__(self, data: Mapping[str, Any], path: str = '',
                 errors: list[str] | None = None,
                 unknown: list[str] | None = None) -> None:
        self._data = data
        self._path = path
        # 与父共享同一张表：读键过程中产生的问题不会被创建期快照丢掉
        self.errors = errors if errors is not None else []
        self.unknown = unknown if unknown is not None else []

    def sub(self, key: str) -> '_Reader':
        value = self._data.get(key)
        if isinstance(value, Mapping):
            return _Reader(value, self._where(key), self.errors, self.unknown)
        self.errors.append(f'{self._where(key)}: 缺失或不是映射')
        return _Reader({}, self._where(key), self.errors, self.unknown)

    def _where(self, key: str) -> str:
        return f'{self._path}.{key}' if self._path else key

    def finish(self, known: tuple[str, ...]) -> None:
        for key in self._data:
            if key not in known:
                self.unknown.append(f'{self._where(key)}: 未知键')

    def _value(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)

    def number(self, key: str, *, lo: float | None = None,
               hi: float | None = None) -> float:
        return _number(self._value(key), self._where(key), self.errors, lo=lo, hi=hi)

    def integer(self, key: str, *, lo: int | None = None,
                hi: int | None = None) -> int:
        return _integer(self._value(key), self._where(key), self.errors, lo=lo, hi=hi)

    def boolean(self, key: str) -> bool:
        value = self._value(key)
        if isinstance(value, bool):
            return value
        self.errors.append(f'{self._where(key)}: 需要布尔，得到 {value!r}')
        return False

    def text(self, key: str) -> str:
        value = self._value(key)
        if isinstance(value, str) and value:
            return value
        self.errors.append(f'{self._where(key)}: 需要非空字符串，得到 {value!r}')
        return ''

    def rng(self, key: str) -> Range:
        return _range(self._value(key), self._where(key), self.errors)

    def vec3(self, key: str, *, color: bool = False) -> Vec3:
        return _vec3(self._value(key), self._where(key), self.errors, color=color)


def _number(value: Any, where: str, errors: list[str], *, lo: float | None,
            hi: float | None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        errors.append(f'{where}: 需要数值，得到 {value!r}')
        return 0.0
    out = float(value)
    if lo is not None and out < lo:
        errors.append(f'{where}: {out} 小于下限 {lo}')
    if hi is not None and out > hi:
        errors.append(f'{where}: {out} 大于上限 {hi}')
    return out


def _integer(value: Any, where: str, errors: list[str], *, lo: int | None,
             hi: int | None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        errors.append(f'{where}: 需要整数，得到 {value!r}')
        return 0
    out = int(value)
    if lo is not None and out < lo:
        errors.append(f'{where}: {out} 小于下限 {lo}')
    if hi is not None and out > hi:
        errors.append(f'{where}: {out} 大于上限 {hi}')
    return out


def _range(value: Any, where: str, errors: list[str]) -> Range:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        out = (float(value), float(value))
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        out = (_number(value[0], where, errors, lo=None, hi=None),
               _number(value[1], where, errors, lo=None, hi=None))
    else:
        errors.append(f'{where}: 需要标量或 [lo, hi]，得到 {value!r}')
        return (0.0, 0.0)
    if out[0] > out[1]:
        errors.append(f'{where}: 区间上下限倒置 {out}')
    return out


def _vec3(value: Any, where: str, errors: list[str], *, color: bool) -> Vec3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        errors.append(f'{where}: 需要三分量列表，得到 {value!r}')
        return (0.0, 0.0, 0.0)
    hi = 1.0 if color else None
    out = tuple(
        _number(item, where, errors, lo=0.0 if color else None, hi=hi)
        for item in value
    )
    return (out[0], out[1], out[2])


def params_from_dict(data: Mapping[str, Any]) -> tuple[OrchardParams, list[str]]:
    """按 schema 构造参数；返回 (参数, 全部问题)。问题非空即不可用."""
    top = _Reader(data)
    top.finish(_TOP_KEYS)

    ground_r = top.sub('ground')
    ground_r.finish((
        'size_x', 'size_y', 'alley_grass_width', 'row_strip_width',
        'soil_rgb', 'grass_rgb',
    ))
    ground = Ground(
        size_x=ground_r.number('size_x', lo=1.0),
        size_y=ground_r.number('size_y', lo=1.0),
        alley_grass_width=ground_r.number('alley_grass_width', lo=0.0),
        row_strip_width=ground_r.number('row_strip_width', lo=0.0),
        soil_rgb=ground_r.vec3('soil_rgb', color=True),
        grass_rgb=ground_r.vec3('grass_rgb', color=True),
    )

    rows_r = top.sub('rows')
    rows_r.finish(('count', 'spacing', 'trees_per_row', 'tree_spacing'))
    rows = Rows(
        count=rows_r.integer('count', lo=1),
        spacing=rows_r.number('spacing', lo=0.5),
        trees_per_row=rows_r.integer('trees_per_row', lo=1),
        tree_spacing=rows_r.number('tree_spacing', lo=0.5),
    )

    tree_r = top.sub('tree')
    tree_r.finish((
        'trunk_height', 'trunk_radius', 'scaffold_count', 'scaffold_length',
        'scaffold_radius', 'scaffold_tilt_deg', 'branchlet_count',
        'branchlet_length', 'branchlet_radius', 'twig_radius', 'canopy_count',
        'canopy_radius', 'canopy_height', 'height_max',
    ))
    tree = Tree(
        trunk_height=tree_r.rng('trunk_height'),
        trunk_radius=tree_r.rng('trunk_radius'),
        scaffold_count=tree_r.rng('scaffold_count'),
        scaffold_length=tree_r.rng('scaffold_length'),
        scaffold_radius=tree_r.number('scaffold_radius', lo=0.0),
        scaffold_tilt_deg=tree_r.rng('scaffold_tilt_deg'),
        branchlet_count=tree_r.rng('branchlet_count'),
        branchlet_length=tree_r.rng('branchlet_length'),
        branchlet_radius=tree_r.number('branchlet_radius', lo=0.0),
        twig_radius=tree_r.number('twig_radius', lo=0.0),
        canopy_count=tree_r.rng('canopy_count'),
        canopy_radius=tree_r.rng('canopy_radius'),
        canopy_height=tree_r.rng('canopy_height'),
        height_max=tree_r.number('height_max', lo=0.5),
    )

    bag_r = top.sub('bag')
    bag_r.finish((
        'fruit_diameter', 'body_diameter', 'body_thickness_ratio',
        'bottom_to_neck', 'neck_diameter', 'neck_length', 'per_tree',
        'cluster_count', 'cluster_radius', 'cluster_gap', 'bottom_height',
        'tilt_max_deg',
        'row_spread', 'alley_side_ratio',
    ))
    bag = Bag(
        fruit_diameter=bag_r.rng('fruit_diameter'),
        body_diameter=bag_r.rng('body_diameter'),
        body_thickness_ratio=bag_r.rng('body_thickness_ratio'),
        bottom_to_neck=bag_r.rng('bottom_to_neck'),
        neck_diameter=bag_r.number('neck_diameter', lo=0.0),
        neck_length=bag_r.number('neck_length', lo=0.0),
        per_tree=bag_r.rng('per_tree'),
        cluster_count=bag_r.rng('cluster_count'),
        cluster_radius=bag_r.rng('cluster_radius'),
        cluster_gap=bag_r.number('cluster_gap', lo=0.0),
        bottom_height=bag_r.rng('bottom_height'),
        tilt_max_deg=bag_r.number('tilt_max_deg', lo=0.0, hi=80.0),
        row_spread=bag_r.number('row_spread', lo=0.0),
        alley_side_ratio=bag_r.number('alley_side_ratio', lo=0.0, hi=1.0),
    )

    tool_r = top.sub('tool')
    tool_r.finish(('profile_id', 'd_inner', 'l_insert', 'clearance_min'))
    tool = Tool(
        profile_id=tool_r.text('profile_id'),
        d_inner=tool_r.number('d_inner', lo=0.0),
        l_insert=tool_r.number('l_insert', lo=0.0),
        clearance_min=tool_r.number('clearance_min', lo=0.0),
    )

    platform_r = top.sub('platform')
    platform_r.finish((
        'face_row_index', 'side', 'row_gap', 'y', 'arm_mount_height',
        'arm_mount_yaw_deg', 'body_size', 'body_bottom', 'track_length',
        'track_width', 'track_height', 'track_separation',
    ))
    platform = Platform(
        face_row_index=platform_r.integer('face_row_index', lo=0),
        side=platform_r.integer('side', lo=-1, hi=1),
        row_gap=platform_r.number('row_gap', lo=0.0),
        y=platform_r.number('y'),
        arm_mount_height=platform_r.number('arm_mount_height', lo=0.1),
        arm_mount_yaw_deg=platform_r.number('arm_mount_yaw_deg'),
        body_size=platform_r.vec3('body_size'),
        body_bottom=platform_r.number('body_bottom', lo=0.0),
        track_length=platform_r.number('track_length', lo=0.0),
        track_width=platform_r.number('track_width', lo=0.0),
        track_height=platform_r.number('track_height', lo=0.0),
        track_separation=platform_r.number('track_separation', lo=0.0),
    )

    zone_r = top.sub('work_zone')
    zone_r.finish(('reach_max_m', 'y_span'))
    work_zone = WorkZone(
        reach_max_m=zone_r.number('reach_max_m', lo=0.1),
        y_span=zone_r.number('y_span', lo=0.0),
    )

    light_r = top.sub('lighting')
    light_r.finish((
        'sun_azimuth_deg', 'sun_elevation_deg', 'ambient', 'background',
        'fog_density', 'shadows',
    ))
    lighting = Lighting(
        sun_azimuth_deg=light_r.number('sun_azimuth_deg'),
        sun_elevation_deg=light_r.number('sun_elevation_deg', lo=1.0, hi=89.0),
        ambient=light_r.vec3('ambient', color=True),
        background=light_r.vec3('background', color=True),
        fog_density=light_r.number('fog_density', lo=0.0),
        shadows=light_r.boolean('shadows'),
    )

    params = OrchardParams(
        seed=top.integer('seed', lo=0),
        ground=ground, rows=rows, tree=tree, bag=bag, tool=tool,
        platform=platform, work_zone=work_zone, lighting=lighting,
    )
    return params, top.errors + top.unknown


def check_params(params: OrchardParams) -> list[str]:
    """跨键约束（袋具与工具余量、行间不穿模、工位几何）."""
    problems: list[str] = []
    bag, tool, tree, rows, platform = (
        params.bag, params.tool, params.tree, params.rows, params.platform,
    )

    bag_outer = bag.body_diameter[1] + 2.0 * tool.clearance_min
    if bag_outer > tool.d_inner:
        problems.append(
            'bag.body_diameter 上限 + 2*tool.clearance_min '
            f'({bag_outer:.4f}) 超过 tool.d_inner ({tool.d_inner:.4f})：袋体无法通过工具'
        )
    if bag.bottom_to_neck[1] > tool.l_insert:
        problems.append(
            'bag.bottom_to_neck 上限 '
            f'({bag.bottom_to_neck[1]:.4f}) 超过 tool.l_insert ({tool.l_insert:.4f})：'
            '袋体插不进筒底'
        )
    if bag.body_diameter[0] < bag.fruit_diameter[0]:
        problems.append(
            'bag.body_diameter 下限小于 fruit_diameter 下限：袋体包不住果'
        )
    if bag.bottom_to_neck[0] < bag.fruit_diameter[0]:
        problems.append(
            'bag.bottom_to_neck 下限小于 fruit_diameter 下限：袋底→袋颈装不下果'
        )

    canopy_half_x = 0.30 + tree.canopy_radius[1]
    if rows.spacing < 2.0 * canopy_half_x + 0.40:
        problems.append(
            f'rows.spacing ({rows.spacing}) 相对冠层横向包络 '
            f'(±{canopy_half_x:.2f}) 过窄：相邻行冠层穿模'
        )
    bag_half_y = bag.row_spread + bag.body_diameter[1]
    if rows.tree_spacing < 2.0 * bag_half_y + 0.20:
        problems.append(
            f'rows.tree_spacing ({rows.tree_spacing}) 相对挂果横向散布 '
            f'(±{bag_half_y:.2f}) 过窄：相邻树袋体穿模'
        )
    if bag.bottom_height[1] + bag.bottom_to_neck[1] > tree.height_max:
        problems.append(
            'bag.bottom_height 上限 + bottom_to_neck 超过 tree.height_max：袋颈高出树高'
        )

    if params.platform.side == 0:
        problems.append('platform.side 只能是 1 或 -1')
    if params.platform.face_row_index >= rows.count:
        problems.append(
            f'platform.face_row_index ({params.platform.face_row_index}) '
            f'超出 rows.count ({rows.count})'
        )
    deck_top = platform.body_bottom + platform.body_size[2]
    if platform.arm_mount_height <= deck_top:
        problems.append(
            f'platform.arm_mount_height ({platform.arm_mount_height}) 不高于车体顶面 '
            f'({deck_top:.3f})：臂座无立柱空间'
        )
    if platform.track_separation < platform.track_width:
        problems.append('platform.track_separation 小于 track_width')
    half_width = 0.5 * (platform.track_separation + platform.track_width)
    if platform.row_gap <= half_width + 0.15:
        problems.append(
            f'platform.row_gap ({platform.row_gap}) 距树行不足车体半宽 '
            f'({half_width:.2f}) + 0.15 m 安全间隙'
        )
    return problems
