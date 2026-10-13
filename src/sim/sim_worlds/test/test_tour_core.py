"""巡航停靠位序列纯核测试（零 ROS）."""

from pathlib import Path

from sim_worlds.params import params_from_dict
from sim_worlds.scene import work_pose
from sim_worlds.tour_core import stations, stations_by_row
import yaml

PKG_DIR = Path(__file__).resolve().parents[1]
CONFIG = PKG_DIR / 'config' / 'orchard.yaml'


def _params():
    raw = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw)
    assert problems == []
    return params


def _tree_items(plan):
    """只取树停靠位（index ≥ 0），排除 headland 换行航点."""
    return [item for item in plan if item.index >= 0]


def test_station_count_covers_all_trees_plus_headlands():
    """每株树一个停靠位 + 每行末一个 headland 换行航点（末行除外）."""
    plan = stations(_params())
    trees = _tree_items(plan)
    assert len(trees) == 35
    assert len({item.tree_id for item in trees}) == 35
    headlands = [item for item in plan if item.index == -1]
    assert len(headlands) == 4  # 行 0–3 各一个出口（行 4 是末行）


def test_headland_waypoints_in_open_area():
    """换行航点在走廊中心、越过末株与端柱，朝向横穿方向."""
    plan = stations(_params())
    headlands = {item.tree_id: item for item in plan if item.index == -1}
    exit0 = headlands['headland_r0_exit']
    assert abs(exit0.x - (-8.0 + 4.0 / 2.0)) < 1e-6  # 行 0 与行 1 之间走廊
    assert abs(exit0.y - (6.6 + 1.8)) < 1e-6  # 末株 6.6 外 1.8 m（端柱 7.7）
    assert exit0.yaw == 0.0  # 朝 +X 横穿
    exit1 = headlands['headland_r1_exit']
    assert exit1.y < 0  # 奇数行从 -Y 端出


def test_serpentine_order_and_yaw():
    """蛇形排序：偶行 +Y 升、奇行 -Y 降；车头朝向 = 行进方向."""
    plan = stations(_params())
    rows = sorted({item.row for item in plan})
    assert rows == [0, 1, 2, 3, 4]
    for row in rows:
        lane = [item for item in _tree_items(plan) if item.row == row]
        ys = [item.y for item in lane]
        ascending = row % 2 == 0
        assert ys == sorted(ys) if ascending else ys == sorted(ys, reverse=True)
        want_yaw = 1.5707963 if ascending else -1.5707963
        assert all(abs(item.yaw - want_yaw) < 1e-6 for item in lane)


def test_station_geometry_matches_work_pose_convention():
    """停靠位横向偏移 = 行线 + side*row_gap（与 scene.work_pose 同口径）."""
    params = _params()
    plan = stations(params)
    pose = work_pose(params)
    row2 = [item for item in _tree_items(plan) if item.row == 2]
    assert all(abs(item.x - pose.vehicle_center[0]) < 1e-6 for item in row2)
    # 树株位沿行间距 2.2 m
    assert abs(row2[1].y - row2[0].y - 2.2) < 1e-6


def test_filter_by_rows():
    """--rows 过滤只保留指定行 + 进入集内下一行的换行航点."""
    plan = stations_by_row(_params(), [2, 3])
    trees = _tree_items(plan)
    assert {item.row for item in trees} == {2, 3}
    assert len(trees) == 14
    headlands = [item.tree_id for item in plan if item.index == -1]
    assert headlands == ['headland_r2_exit']  # 只服务 2→3 换行
    # 行 2 升序、行 3 降序
    ys2 = [item.y for item in trees if item.row == 2]
    ys3 = [item.y for item in trees if item.row == 3]
    assert ys2 == sorted(ys2)
    assert ys3 == sorted(ys3, reverse=True)
