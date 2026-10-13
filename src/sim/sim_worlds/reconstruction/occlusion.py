"""叶遮挡与枝干扰受控变量（纯核：决策与位置，无 bpy）.

背景：合并前的叶布置是"避让式"（袋正面 0.09 m 内不插叶），遮挡不是
受控变量。本模块给每只袋分档 {none, light, heavy}，产出前景叶槽位与
干扰枝段；实际网格由 build_scene 用 geometry.Leaves / tube 落地。
名义覆盖率只是布叶意图，真实覆盖率由评测器从渲染深度序反测（叶无
实例 ID，按"袋框内深度明显更近的像素占比"计）。

坐标约定：所有输入输出为世界系 (x, y, z)；view_dir 为从袋指向观察侧
（作业道相机）的单位向量，场景级常量由调用方传入并记录 manifest。
"""

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

Vec3 = Sequence[float]

LEVELS: Dict[str, Dict] = {
    'none': {'weight': .40, 'leaves': 0, 'nominal_coverage': .0,
             'branch_front': False, 'branch_corridor': False},
    'light': {'weight': .35, 'leaves': 2, 'nominal_coverage': .20,
              'branch_front': False, 'branch_corridor': False},
    'heavy': {'weight': .25, 'leaves': 4, 'nominal_coverage': .45,
              'branch_front': True, 'branch_corridor': False},
}
"""heavy 档必带袋前横枝；corridor 枝按独立小概率叠加（见 corridor_branch）."""


@dataclass(frozen=True)
class OcclusionPlan:
    level: str
    leaves: int
    nominal_coverage: float
    branch_front: bool
    branch_corridor: bool


def assign_levels(rng, count: int, levels: Dict[str, Dict] = None) -> List[OcclusionPlan]:
    """按权重给 count 只袋分档；确定性来自传入 rng."""
    spec = levels or LEVELS
    names = sorted(spec)
    weights = [spec[n]['weight'] for n in names]
    plans = []
    for _ in range(count):
        name = rng.choices(names, weights=weights, k=1)[0]
        item = spec[name]
        plans.append(OcclusionPlan(
            name, item['leaves'], item['nominal_coverage'],
            item['branch_front'], item['branch_corridor']))
    return plans


@dataclass(frozen=True)
class BagFrame:
    """袋的定位框架（世界系），由建场侧从 bag 对象算出."""

    neck: Vec3
    """袋颈（挂点）."""
    center: Vec3
    """袋体中心."""
    bottom: Vec3
    """袋底."""
    face_dir: Vec3
    """袋面朝向（单位向量，指向观察侧的那面）."""
    width: float
    height: float


def _add(a: Vec3, b: Vec3) -> List[float]:
    return [a[i] + b[i] for i in range(3)]


def _sub(a: Vec3, b: Vec3) -> List[float]:
    return [a[i] - b[i] for i in range(3)]


def _scale(a: Vec3, s: float) -> List[float]:
    return [x * s for x in a]


def _norm(a: Vec3) -> float:
    return sum(x * x for x in a) ** .5


def foreground_leaf_slots(
    bag: BagFrame, plan: OcclusionPlan, rng,
    view_dir: Vec3,
) -> List[Tuple[Vec3, Vec3]]:
    """前景叶 (root, tip) 槽位：挂在袋颈果枝侧、叶面横在袋与相机之间.

    root 取袋颈上方果枝一小段上的点（物理连接故事：结果枝叶片本就
    聚在果柄附近）；tip 朝相机侧伸出并略下垂，落在袋面投影带内以
    形成遮挡。距离按袋宽比例缩放，覆盖 light≈20% / heavy≈45% 的
    名义目标（真实值以渲染深度反测为准）。
    """
    if plan.leaves <= 0:
        return []
    face = [c / max(_norm(bag.face_dir), 1e-9) for c in bag.face_dir]
    slots = []
    up = [0.0, 0.0, 1.0]
    side = [
        face[1] * up[2] - face[2] * up[1],
        face[2] * up[0] - face[0] * up[2],
        face[0] * up[1] - face[1] * up[0]]
    if _norm(side) < 1e-6:
        side = [1.0, 0.0, 0.0]
    side = [c / _norm(side) for c in side]
    span_y = bag.width * .55
    for i in range(plan.leaves):
        # 沿袋高分层 + 横向错位，叶端落在袋面前方 0.3–0.7 倍袋宽处。
        t = (i + .5) / max(plan.leaves, 1)
        lateral = rng.uniform(-span_y, span_y)
        depth = bag.width * rng.uniform(.30, .70)
        root = _add(bag.neck, _scale(side, lateral * .4))
        root = _add(root, [0, 0, bag.height * (.10 + .18 * t)])
        tip = _add(bag.center,
                   _add(_scale(side, lateral),
                        _add(_scale(face, depth),
                             [0, 0, bag.height * (.30 - .35 * t)])))
        slots.append((root, tip))
    return slots


def front_branch_segment(
    bag: BagFrame, rng,
) -> Tuple[Vec3, Vec3, float]:
    """袋前横枝 (p0, p1, radius)：横穿袋面前方，两端接向树体侧.

    距袋心 0.5–1.0 倍袋宽、走向大致水平（垂直于 face_dir 与 up 的
    混合），半径 8–12 mm。
    """
    face = [c / max(_norm(bag.face_dir), 1e-9) for c in bag.face_dir]
    distance = bag.width * rng.uniform(.50, 1.0)
    mid = _add(bag.center, _add(_scale(face, distance),
                                [0, 0, bag.height * rng.uniform(-.15, .25)]))
    along = [face[1], -face[0], 0.0]
    if _norm(along) < 1e-6:
        along = [1.0, 0.0, 0.0]
    along = [c / _norm(along) for c in along]
    half = bag.width * rng.uniform(.8, 1.3)
    p0 = _add(mid, _scale(along, -half))
    p1 = _add(mid, _scale(along, half))
    return p0, p1, rng.uniform(.008, .012)


def corridor_branch_segment(
    bag: BagFrame, rng,
    approach_dir: Vec3,
    insert_depth: float = .09,
) -> Tuple[Vec3, Vec3, float]:
    """套入接近走廊的穿越枝：横在袋底外 approach 方向 insert_depth 处.

    approach_dir = 袋轴向外（工具套入方向，单位向量）；走廊口径与
    adaptive_shear 档案（D_inner 0.120 / L_insert 0.090）对齐。半径
    6–10 mm，足够干扰又不封死走廊。
    """
    depth = insert_depth * rng.uniform(.8, 1.2)
    unit = [c / max(_norm(approach_dir), 1e-9) for c in approach_dir]
    base = _add(bag.bottom, _scale(unit, depth))
    along = [base[1], -base[0], 0.0]
    if _norm(along) < 1e-6:
        along = [1.0, 0.0, 0.0]
    along = [c / _norm(along) for c in along]
    half = rng.uniform(.09, .13)
    p0 = _add(base, _add(_scale(along, -half), [0, 0, rng.uniform(-.02, .05)]))
    p1 = _add(base, _add(_scale(along, half), [0, 0, rng.uniform(-.02, .05)]))
    return p0, p1, rng.uniform(.006, .010)
