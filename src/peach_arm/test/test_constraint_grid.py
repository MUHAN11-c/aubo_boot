"""感知约束网格夹具：零 ROS，供 sim_field_targets --grid 对账."""
from pathlib import Path

import yaml

GRID = (
    Path(__file__).resolve().parents[1]
    / 'test' / 'fixtures' / 'perception_constraint_grid.yaml')


def test_grid_cases_have_expect_and_geometry():
    book = yaml.safe_load(GRID.read_text())
    cases = book['cases']
    expects = {row['expect'] for row in cases.values()}
    assert 'succeed' in expects
    assert 'skip_cartesian' in expects
    assert 'skip_ik' in expects
    assert 'skip_select' in expects
    assert 'travel_max' in cases
    assert cases['travel_max']['travel_m'] == 0.20
    for cid, case in cases.items():
        assert case.get('entry_xyz') and len(case['entry_xyz']) == 3, cid
        assert case.get('axis') and len(case['axis']) == 3, cid
        assert case['expect'] in {
            'succeed', 'skip_ik', 'skip_cartesian', 'skip_select'}, cid


def test_succeed_entries_are_spread():
    book = yaml.safe_load(GRID.read_text())
    pts = [
        tuple(case['entry_xyz'])
        for case in book['cases'].values() if case['expect'] == 'succeed']
    unique = {tuple(round(v, 3) for v in pt) for pt in pts}
    assert len(unique) >= 5
    xs = [pt[0] for pt in pts]
    ys = [pt[1] for pt in pts]
    assert max(xs) - min(xs) > 0.20
    assert max(ys) - min(ys) > 0.10


def test_lab_oos_case_is_beyond_one_meter():
    book = yaml.safe_load(GRID.read_text())
    case = book['cases']['lab_oos_20260922']
    e = case['entry_xyz']
    axis = case['axis']
    n = sum(a * a for a in axis) ** 0.5
    travel = float(case['travel_m'])
    sleeve = [e[i] + axis[i] / n * travel for i in range(3)]
    radius = sum(v * v for v in sleeve) ** 0.5
    assert radius > 1.0
    assert case['expect'] == 'skip_cartesian'
