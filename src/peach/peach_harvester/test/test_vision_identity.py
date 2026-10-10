"""Zero-ROS tests for frame-level identity commit and ambiguity (F03/F04)."""
from __future__ import annotations

import numpy as np

from peach_harvester.vision.scene_perception.identity import (
    assign_detections,
    TargetRegistry,
)


def _det(xyz, class_id=0):
    return {'position': np.asarray(xyz, dtype=float), 'class_id': class_id}


def test_nan_position_stays_untracked():
    registry = TargetRegistry(max_targets=4, confirm_frames=1)
    registry.begin_frame(now=1.0)
    rows = registry.match_or_register_frame(
        [_det([np.nan, 0.0, 0.5])], now=1.0)
    assert rows[0][0].startswith('untracked_')
    assert rows[0][1] is False
    assert registry.stats()['n_targets'] == 0


def test_capacity_does_not_evict_assigned_or_raise():
    """新目标排在已有命中之前时，不得淘汰本帧已分配轨道（F03）."""
    registry = TargetRegistry(
        max_targets=2, match_radius=0.06, confirm_frames=1,
        tentative_ttl_frames=50, max_age_s=600.0)
    registry.begin_frame(now=1.0)
    first = registry.match_or_register_frame(
        [_det([0.0, 0.0, 0.5]), _det([0.2, 0.0, 0.5])], now=1.0)
    id_a, id_b = first[0][0], first[1][0]
    assert {id_a, id_b} == set(registry._targets)

    def _third_frame(order):
        clone = TargetRegistry(
            max_targets=2, match_radius=0.06, confirm_frames=1,
            tentative_ttl_frames=50, max_age_s=600.0)
        clone.begin_frame(now=1.0)
        clone.match_or_register_frame(
            [_det([0.0, 0.0, 0.5]), _det([0.2, 0.0, 0.5])], now=1.0)
        clone.begin_frame(now=2.0)
        return clone.match_or_register_frame(order, now=2.0), clone

    new = _det([1.0, 0.0, 0.5])
    a = _det([0.0, 0.0, 0.5])
    b = _det([0.2, 0.0, 0.5])
    rows_new_first, left = _third_frame([new, a, b])
    rows_new_last, right = _third_frame([a, b, new])
    assigned_left = {tid for tid, _ in left._targets.items()}
    assigned_right = {tid for tid, _ in right._targets.items()}
    assert assigned_left == assigned_right
    assert id_a in assigned_left and id_b in assigned_left
    assert left.stats()['n_targets'] == 2
    assert right.stats()['n_targets'] == 2
    # 新检测不得挤掉本帧命中；顺序不得改变已分配身份
    assert rows_new_first[1][0] == id_a
    assert rows_new_first[2][0] == id_b
    assert rows_new_last[0][0] == id_a
    assert rows_new_last[1][0] == id_b


def test_edge_frames_do_not_accumulate_confirm():
    """贴边帧不计确认（09-17 真机 P1-A）：贴边恒显的滑动噪声块不得转正."""
    registry = TargetRegistry(
        max_targets=4, match_radius=0.06, confirm_frames=3)
    registry.begin_frame(now=1.0)
    edge_det = _det([0.0, 0.0, 0.5])
    edge_det['at_edge'] = True
    rows = [(None, None)]
    for frame in range(2, 6):
        registry.begin_frame(now=float(frame))
        rows = registry.match_or_register_frame(
            [dict(edge_det)], now=float(frame))
    tid = rows[0][0]
    entry = registry.get(tid)
    assert entry['obs_count'] == 0
    assert entry['confirmed'] is False
    # 移入视野内（非贴边）即恢复累积并转正
    ok_det = _det([0.0, 0.0, 0.5])
    for frame in range(6, 9):
        registry.begin_frame(now=float(frame))
        registry.match_or_register_frame([dict(ok_det)], now=float(frame))
    assert registry.get(tid)['obs_count'] == 3
    assert registry.get(tid)['confirmed'] is True


def test_edge_first_frame_starts_unconfirmed():
    """贴边首帧注册从零攒（confirm_frames=1 也不得立即转正）."""
    registry = TargetRegistry(
        max_targets=4, match_radius=0.06, confirm_frames=1)
    registry.begin_frame(now=1.0)
    edge_det = _det([0.0, 0.0, 0.5])
    edge_det['at_edge'] = True
    rows = registry.match_or_register_frame([edge_det], now=1.0)
    entry = registry.get(rows[0][0])
    assert entry['obs_count'] == 0
    assert entry['confirmed'] is False


def test_equal_cost_full_matching_is_ambiguous():
    """全占用时交叉等价解须标歧义，不得更新确认锚点（F04）."""
    pos = np.array([0.5, 0.0, 1.0], dtype=float)
    table = {
        't0': {'position': pos.copy(), 'class_id': 0},
        't1': {'position': pos.copy(), 'class_id': 0},
    }
    detections = [_det(pos), _det(pos)]
    results = assign_detections(
        detections, table, frame_used=set(), match_radius=0.06)
    assert len(results) == 2
    assert all(status == 'ambiguous' for _, _, status in results)
    assert all(tid is None for tid, _, _ in results)

    registry = TargetRegistry(
        max_targets=4, match_radius=0.06, confirm_frames=1)
    registry.begin_frame(now=1.0)
    registry.match_or_register_frame([_det(pos)], now=1.0)
    # 人为塞入第二条同位置轨道，模拟两确认锚点重合
    first_id = next(iter(registry._targets))
    registry._targets['t_dup'] = dict(registry._targets[first_id])
    registry._targets['t_dup']['target_id'] = 't_dup'
    anchor = registry._targets[first_id]['position'].copy()
    registry.begin_frame(now=2.0)
    registry.match_or_register_frame([_det(pos), _det(pos)], now=2.0)
    np.testing.assert_allclose(registry._targets[first_id]['position'], anchor)
    np.testing.assert_allclose(registry._targets['t_dup']['position'], anchor)


# ---- W2/V2+V3 回归：淘汰回传与匿名 id 唯一性 ----
def test_begin_frame_returns_evicted_ids():
    registry = TargetRegistry(
        max_targets=8, confirm_frames=5, tentative_ttl_frames=2)
    registry.begin_frame(now=1.0)
    registry.match_or_register_frame([_det([0.0, 0.0, 0.5])], now=1.0)
    tid = next(iter(registry._targets))
    # 越过 TTL（未确认表项按帧计，判定为严格大于）：begin_frame 必须回传
    # 被淘汰 id，供 pipeline 同步清理 bbox_at_edge 旁路缓存（防长运行泄漏）。
    registry.begin_frame(now=1.0)
    registry.begin_frame(now=1.0)
    evicted = registry.begin_frame(now=1.0)
    assert evicted == [tid]
    assert registry.stats()['n_targets'] == 0


def test_anonymous_ids_unique_within_frame():
    from types import SimpleNamespace

    registry = TargetRegistry(max_targets=8, confirm_frames=1)
    registry.begin_frame(now=1.0)
    matched = SimpleNamespace(status='ambiguous', target_id=None)
    first = registry._commit_match(
        np.asarray([0.0, 0.0, 0.5]), 0, None, 0.0, 'ok', 1.0, matched)
    second = registry._commit_match(
        np.asarray([0.2, 0.0, 0.5]), 0, None, 0.0, 'ok', 1.0, matched)
    # 旧式 f'{frame_index}_{len(frame_used)}' 同帧两次歧义会撞号
    assert first[0] != second[0]
    assert first[0].startswith('ambiguous_')
    assert second[0].startswith('ambiguous_')
