"""全局采摘计划收齐式锁定语义与运行数据契约测试."""
import numpy as np

from peach_pose_ros2.harvest_data import HarvestDataStore
from peach_pose_ros2.harvest_plan import GlobalHarvestPlan


def _rec(target_id, status=0, distance=1.0, confidence=0.9, height=0.5,
         confirmed=True, flags=()):
    """构造一条候选观测 record（height=None 时省略 base_height_m 键）."""
    record = {
        'target_id': target_id,
        'confirmed': confirmed,
        'status': status,
        'camera_distance_m': distance,
        'confidence': confidence,
        'diagnostic_flags': list(flags),
    }
    if height is not None:
        record['base_height_m'] = height
    return record


def _plan(**kwargs):
    """小窗口计划实例：3 帧起步、2 帧静止、100 s 超时（默认不触发超时）."""
    kwargs.setdefault('min_collect_frames', 3)
    kwargs.setdefault('lock_settle_frames', 2)
    kwargs.setdefault('max_collect_s', 100.0)
    return GlobalHarvestPlan(**kwargs)


def test_locks_after_collect_window_and_advances_by_fixed_priority():
    """收齐窗口跨帧累积目标，静止帧数满足后一次性锁定，锁定后推进不变."""
    plan = _plan()
    far = _rec('far', distance=1.2, height=0.8)
    near = _rec('near', distance=0.7, height=0.4)
    # 第 1 帧只见 far，第 2 帧新增 near：窗口未稳，不锁定
    plan.update([far], now=1.0)
    assert not plan.locked
    plan.update([far, near], now=2.0)
    assert not plan.locked
    # 第 3 帧无新增（帧数 ≥3 但距上次新增仅 1 帧 < settle=2），仍未锁定
    plan.update([far, near], now=3.0)
    assert not plan.locked
    # 第 4 帧：连续 2 帧无新增确认 ID → 窗口关闭，一次性锁定
    plan.update([far, near], now=4.0)
    assert plan.locked
    assert plan.locked_ids == ('near', 'far')
    assert plan.target_count == 2
    assert plan.selected_target_id == 'near'
    assert plan.snapshot_id == 1
    # 锁定后目标暂时丢失不改变选中 ID 与数量
    plan.update([far], now=5.0)
    assert plan.selected_target_id == 'near'
    assert plan.target_count == 2
    # 完成推进按固定优先级（与收齐锁定前语义一致）
    plan.update([far, near], now=6.0)
    assert plan.complete_selected() == 'far'
    assert plan.harvest_status('near') == 'HARVESTED'
    plan.update([far, near], now=7.0)
    assert plan.complete_selected() == ''
    assert plan.harvest_status('far') == 'HARVESTED'


def test_rejected_target_waits_until_quality_recovers():
    """REJECT 目标计入锁定总数，恢复安全前不会成为 selected ID."""
    plan = _plan()
    safe = _rec('safe', distance=0.8)
    blocked = _rec('blocked', status=2, distance=0.6, confidence=0.95)
    for frame in range(4):
        plan.update([safe, blocked], now=float(frame + 1))
    assert plan.locked
    # REJECT 也入锁定集（计总数与优先级，排序垫底）
    assert plan.locked_ids == ('safe', 'blocked')
    assert plan.target_count == 2
    assert plan.selected_target_id == 'safe'
    assert plan.complete_selected() == ''
    assert plan.harvest_status('blocked') == 'WAITING_QUALITY'
    blocked['status'] = 1
    plan.update([blocked], now=5.0)
    assert plan.selected_target_id == 'blocked'


def test_locks_on_timeout_even_without_settle():
    """超过 max_collect_s 仍未静止也强制锁定（超时兜底，含新目标）."""
    plan = GlobalHarvestPlan(
        min_collect_frames=100, lock_settle_frames=50, max_collect_s=5.0)
    plan.update([_rec('a')], now=10.0)
    assert not plan.locked
    plan.update([_rec('a'), _rec('b', distance=0.5)], now=12.0)
    assert not plan.locked
    # 窗口起点 10.0 s；15.5 s 时已超 5 s → 强制锁定（b 虽新增不久也入集）
    plan.update([_rec('a'), _rec('b', distance=0.5)], now=15.5)
    assert plan.locked
    assert plan.locked_ids == ('b', 'a')


def test_empty_window_locks_with_zero_targets():
    """窗口期满无确认目标也锁定（locked=True、count=0、selected=''）."""
    plan = _plan()
    plan.update([], now=1.0)
    assert not plan.locked
    plan.update([], now=2.0)
    assert not plan.locked  # 帧数 2 < min_collect_frames=3
    plan.update([], now=3.0)
    assert plan.locked
    assert plan.locked_ids == ()
    assert plan.target_count == 0
    assert plan.selected_target_id == ''
    assert plan.snapshot_id == 1
    assert plan.complete_selected() == ''


def test_new_ids_after_lock_are_ignored():
    """锁定后新出现的 ID 不入 locked_ids，不改变数量与既有优先级."""
    plan = _plan()
    for frame in range(4):
        plan.update([_rec('a'), _rec('b', distance=0.5)],
                    now=float(frame + 1))
    assert plan.locked
    assert plan.locked_ids == ('b', 'a')
    # c 更近更优，但锁定后不再入集
    plan.update(
        [_rec('a'), _rec('b', distance=0.5), _rec('c', distance=0.3)],
        now=5.0)
    assert plan.locked_ids == ('b', 'a')
    assert plan.target_count == 2
    assert plan.priority('c') == 0


def test_priority_orders_status_distance_height_confidence():
    """排序键次序：状态 → 距离（先近）→ 高度（先低）→ 置信度 → ID."""
    plan = _plan()
    far_low = _rec('far_low', status=0, distance=1.5, height=0.3,
                   confidence=0.9)
    near_high = _rec('near_high', status=0, distance=0.8, height=0.9,
                     confidence=0.9)
    near_low = _rec('near_low', status=0, distance=0.8, height=0.4,
                    confidence=0.8)
    near_low_hi = _rec('near_low_hi', status=0, distance=0.8, height=0.4,
                       confidence=0.95)
    reobs = _rec('reobs', status=1, distance=0.5, height=0.2, confidence=0.99)
    records = [far_low, near_high, near_low, near_low_hi, reobs]
    for frame in range(4):
        plan.update(records, now=float(frame + 1))
    assert plan.locked
    # ACCEPT 全部先于 REOBSERVE；同状态先近后远；同距先低后高；
    # 同距同高置信度高者优先
    assert plan.locked_ids == (
        'near_low_hi', 'near_low', 'near_high', 'far_low', 'reobs')
    assert plan.priorities['near_low_hi'] == 1
    assert plan.priorities['reobs'] == 5
    assert plan.selected_target_id == 'near_low_hi'


def test_priority_ignores_height_when_disabled():
    """prefer_lower_first=False 时高度不参与排序，同距按置信度决胜."""
    plan = _plan(prefer_lower_first=False)
    low_weak = _rec('low_weak', distance=0.8, height=0.2, confidence=0.7)
    high_strong = _rec('high_strong', distance=0.8, height=0.9,
                       confidence=0.9)
    for frame in range(4):
        plan.update([low_weak, high_strong], now=float(frame + 1))
    assert plan.locked_ids == ('high_strong', 'low_weak')


def test_reobservation_overwrites_and_keeps_single_entry():
    """同 ID 跨帧重复观测只入集一次，锁定排序取最新一帧的质量量."""
    plan = _plan()
    plan.update([_rec('a', distance=1.0), _rec('b', distance=0.8)], now=1.0)
    # a 的最新观测变近：锁定排序应反映新距离（a 0.4 < b 0.8）
    plan.update([_rec('a', distance=0.4), _rec('b', distance=0.8)], now=2.0)
    plan.update([_rec('a', distance=0.4), _rec('b', distance=0.8)], now=3.0)
    plan.update([_rec('a', distance=0.4), _rec('b', distance=0.8)], now=4.0)
    assert plan.locked
    assert plan.locked_ids == ('a', 'b')
    assert len(plan.locked_ids) == len(set(plan.locked_ids))


def test_unconfirmed_records_never_enter_lock_set():
    """未确认记录不进累积集，窗口期满按空集锁定."""
    plan = _plan()
    for frame in range(4):
        plan.update([_rec('a', confirmed=False)], now=float(frame + 1))
    assert plan.locked
    assert plan.locked_ids == ()


def test_reset_reopens_collect_window():
    """Reset 清空锁定状态并重新进入收齐窗口（snapshot_id 递增）."""
    plan = _plan()
    for frame in range(4):
        plan.update([_rec('a')], now=float(frame + 1))
    assert plan.locked_ids == ('a',)
    assert plan.snapshot_id == 1
    plan.reset()
    assert not plan.locked
    assert plan.target_count == 0
    plan.update([_rec('b', distance=0.5)], now=5.0)
    assert not plan.locked
    plan.update([_rec('b', distance=0.5)], now=6.0)
    plan.update([_rec('b', distance=0.5)], now=7.0)
    assert plan.locked_ids == ('b',)
    assert plan.snapshot_id == 2


def test_harvest_store_manifest_events_and_mask(tmp_path):
    """一个 run 可关联 manifest、来源事件和时间戳掩膜."""
    store = HarvestDataStore(root=tmp_path)
    run_dir = store.start('run_1', {'target_count': 1})
    store.append_event({'source': 'perception', 'event': 'locked'})
    mask_path = store.save_mask(
        'target_0', 123, np.ones((4, 5), dtype=np.uint8))
    assert (run_dir / 'manifest.yaml').is_file()
    assert (run_dir / 'events.jsonl').is_file()
    assert (run_dir / 'latest_perception.json').is_file()
    assert (run_dir / mask_path).is_file()
