"""场景生成纯核测试：确定性、袋具几何、现场包络对齐、可达性（零 ROS）."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from peach_sim.params import params_from_dict
from peach_sim.scene import (
    aim_rpy,
    render_scene,
    tree_positions,
    work_pose,
    work_zone_tree_ids,
)
import yaml

PKG_DIR = Path(__file__).resolve().parents[1]
CONFIG = PKG_DIR / 'config' / 'orchard.yaml'
WORLD = PKG_DIR / 'worlds' / 'peach_orchard.sdf'
MANIFEST = PKG_DIR / 'worlds' / 'peach_orchard.manifest.yaml'

# runs/field_pregrasp_* 实测包络（base_link 系）：袋底→袋颈、袋底离臂座高
FIELD_BOTTOM_TO_NECK = (0.05, 0.12)
FIELD_BOTTOM_ABOVE_MOUNT = (0.50, 0.73)


def _scene():
    """按部署 yaml 渲染一次场景."""
    raw = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw)
    assert problems == []
    return params, render_scene(params)


def _unit(vector):
    norm = math.sqrt(sum(item * item for item in vector))
    return tuple(item / norm for item in vector)


def test_render_is_deterministic():
    """同参数两次渲染必须逐字节一致（含随机流）."""
    _params, first = _scene()
    _params, second = _scene()
    assert first.world_sdf == second.world_sdf
    assert first.manifest == second.manifest


def test_committed_world_matches_generator():
    """入库世界/清单必须是当前生成器产物（防漂移）."""
    _params, scene = _scene()
    assert WORLD.read_text(encoding='utf-8') == scene.world_sdf
    committed = yaml.safe_load(MANIFEST.read_text(encoding='utf-8'))
    assert committed == scene.manifest


def test_manifest_targets_exist_in_world():
    """清单每颗目标都能在世界里找到同名模型."""
    _params, scene = _scene()
    names = {
        model.get('name')
        for model in ET.fromstring(scene.world_sdf).iter('model')
    }
    targets = scene.manifest['targets']
    assert targets
    for entry in targets:
        assert entry['id'] in names, f'世界缺模型 {entry["id"]}'
        assert entry['tree'] in names, f'世界缺树 {entry["tree"]}'


def test_bag_geometry_within_tool_budget():
    """袋体尺寸受工具内径/插入深度/现场果径三重约束."""
    params, scene = _scene()
    for entry in scene.manifest['targets']:
        outer = entry['body_diameter'] + 2.0 * params.tool.clearance_min
        assert outer <= params.tool.d_inner + 1e-9, entry['id']
        assert entry['bottom_to_neck'] <= params.tool.l_insert + 1e-9, entry['id']
        assert entry['body_diameter'] >= entry['fruit_diameter'], entry['id']
        assert entry['body_thickness'] <= entry['body_diameter'], entry['id']


def test_targets_match_field_envelope():
    """袋几何与离地高度对齐 runs/field_pregrasp_* 实测包络."""
    params, scene = _scene()
    mount_z = work_pose(params).mount[2]
    for entry in scene.manifest['targets']:
        low, high = FIELD_BOTTOM_TO_NECK
        assert low <= entry['bottom_to_neck'] <= high, entry['id']
        above = entry['bag_bottom'][2] - mount_z
        low, high = FIELD_BOTTOM_ABOVE_MOUNT
        assert low <= above <= high, f'{entry["id"]} 高 {above:.3f}'


def test_axis_points_bottom_to_neck():
    """轴是单位向量且由袋底指向袋颈（口径同感知 direction=底→颈）."""
    _params, scene = _scene()
    for entry in scene.manifest['targets']:
        bottom = entry['bag_bottom']
        neck = entry['bag_neck']
        axis = entry['axis']
        assert abs(math.dist((0.0, 0.0, 0.0), axis) - 1.0) < 1e-6
        rebuilt = _unit(tuple(
            neck[index] - bottom[index] for index in range(3)))
        for index in range(3):
            assert abs(rebuilt[index] - axis[index]) < 1e-4, entry['id']
        assert axis[2] > 0.0, f'{entry["id"]} 袋颈不在袋底上方'


def test_aim_rpy_round_trip():
    """aim_rpy 的 (0, pitch, yaw) 应把局部 +Z 转回方向向量."""
    for direction in (
            (0.0, 0.0, 1.0), (0.2, -0.1, 0.97), (0.5, 0.5, 0.7)):
        _roll, pitch, yaw = aim_rpy(direction)
        rebuilt = (
            math.sin(pitch) * math.cos(yaw),
            math.sin(pitch) * math.sin(yaw),
            math.cos(pitch),
        )
        expected = _unit(direction)
        for index in range(3):
            assert abs(rebuilt[index] - expected[index]) < 1e-9


def test_work_zone_has_reachable_targets():
    """作业位覆盖的每株树都得有足够可达套袋桃（场景是有效试验台）."""
    params, scene = _scene()
    zone_ids = work_zone_tree_ids(params)
    assert zone_ids, '作业区没有树'
    per_tree: dict[str, int] = {tree_id: 0 for tree_id in zone_ids}
    for entry in scene.manifest['targets']:
        if entry['tree'] in per_tree and entry['reachable']:
            per_tree[entry['tree']] += 1
        assert entry['dist_to_mount'] > 0.3, f'{entry["id"]} 贴着臂座'
        if entry['reachable']:
            assert entry['dist_to_mount'] <= params.work_zone.reach_max_m
    for tree_id, count in per_tree.items():
        assert count >= 2, f'{tree_id} 可达套袋桃仅 {count} 颗'


def test_targets_never_touch_ground_or_platform():
    """袋底高于地面且不与车体包络相交（车体半宽 + 0.15 m 安全间隙）."""
    params, scene = _scene()
    work = work_pose(params)
    half_width = 0.5 * (params.platform.track_separation
                        + params.platform.track_width)
    for entry in scene.manifest['targets']:
        assert entry['bag_bottom'][2] > 0.5, entry['id']
        horizontal = math.hypot(
            entry['bag_bottom'][0] - work.vehicle_center[0],
            entry['bag_bottom'][1] - work.vehicle_center[1])
        assert horizontal > half_width, f'{entry["id"]} 落进车体投影'


def test_tree_grid_is_symmetric():
    """树位网格对称（行沿 Y、行距沿 X），共 rows×trees_per_row 株."""
    raw = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    params, _problems = params_from_dict(raw)
    positions = tree_positions(params)
    assert len(positions) == params.rows.count * params.rows.trees_per_row
    xs = sorted({round(origin[0], 6) for _n, _r, _i, origin in positions})
    assert len(xs) == params.rows.count
    assert abs(xs[len(xs) // 2]) < 1e-9
