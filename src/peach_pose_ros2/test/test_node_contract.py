"""全局采摘计划收齐式锁定语义与运行数据契约测试."""
import numpy as np

from peach_core.harvest_data import HarvestDataStore
from peach_pose_ros2.peach_pose.harvest_plan import GlobalHarvestPlan
from peach_pose_ros2.peach_pose.target_registry import TargetRegistry
from peach_pose_ros2.peach_pose.timing_metrics import TimingMetrics
import pytest


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


def test_reopen_target_restores_selectability():
    """E3 残局抬质量：终局目标重开后按固定优先级重新可选（观测在场时）."""
    plan = _plan()
    far = _rec('far', distance=1.2, height=0.8)
    near = _rec('near', distance=0.7, height=0.4)
    for frame in range(4):
        plan.update([far, near], now=float(frame + 1))
    assert plan.locked
    assert plan.selected_target_id == 'near'
    # 残局场景：一轮耗尽，两个目标均已终局（selected 空）
    assert plan.complete_selected() == 'far'
    assert plan.complete_selected() == ''
    assert plan.selected_target_id == ''
    # 抬质量成功 → 重开 near：移出 completed，下帧按优先级重新选中
    assert plan.reopen_target('near') == ''
    assert 'near' not in plan.completed_ids
    plan.update([far, near], now=5.0)
    assert plan.selected_target_id == 'near'
    # 幂等守卫：已重开（无 completed 账目）再次重开被拒
    assert '未终局' in plan.reopen_target('near')


def test_reopen_target_does_not_preempt_active_selection():
    """选中粘性：重开的更高优先级目标不抢占在办 selected，等其自然终局."""
    plan = _plan()
    far = _rec('far', distance=1.2, height=0.8)
    near = _rec('near', distance=0.7, height=0.4)
    for frame in range(4):
        plan.update([far, near], now=float(frame + 1))
    assert plan.locked
    assert plan.complete_selected() == 'far'  # near 终局，选中推进到 far
    assert plan.reopen_target('near') == ''
    # far 仍在办：near 不抢占（与既有推进不变语义一致，防抖动切换）
    plan.update([far, near], now=5.0)
    assert plan.selected_target_id == 'far'
    # far 终局后 near 按优先级接上
    assert plan.complete_selected() == 'near'


def test_reopen_target_guards():
    """Reopen 三重守卫：未锁定 / 不在锁定集 / 未终局，均不动账目."""
    plan = _plan()
    rec = _rec('a', distance=0.7)
    assert '尚未锁定' in plan.reopen_target('a')
    for frame in range(4):
        plan.update([rec], now=float(frame + 1))
    assert plan.locked
    assert '不在本轮锁定集' in plan.reopen_target('ghost')
    assert '未终局' in plan.reopen_target('a')
    # 守卫拒绝后账目不变：a 仍是选中目标且可正常推进
    assert plan.selected_target_id == 'a'
    assert plan.complete_selected() == ''
    assert plan.harvest_status('a') == 'HARVESTED'


def test_reopen_target_waits_for_observation_to_reselect():
    """重开目标当前帧不可选（记录缺席）时不强选，观测恢复后按优先级重选."""
    plan = _plan()
    a = _rec('a', distance=0.7)
    b = _rec('b', distance=0.9)
    for frame in range(4):
        plan.update([a, b], now=float(frame + 1))
    assert plan.locked
    # 残局场景：两个目标均终局，selected 空
    assert plan.complete_selected() == 'b'
    assert plan.complete_selected() == ''
    assert plan.selected_target_id == ''
    assert plan.reopen_target('a') == ''
    # 当前帧 a 无观测记录（消失态）：不强选，selected 保持空
    plan.update([], now=5.0)
    assert plan.selected_target_id == ''
    # a 观测恢复：可选判定按优先级选中 a
    plan.update([a], now=6.0)
    assert plan.selected_target_id == 'a'


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


def test_window_waits_for_pending_confirmation():
    """有未确认记录（确认攒帧中）时窗口不按静止关闭，防提前锁定空集."""
    plan = _plan()
    pending = _rec('late', confirmed=False)
    # 帧数与静止条件都满足，但仍有未确认记录 → 不得锁定
    for frame in range(5):
        plan.update([pending], now=float(frame + 1))
        assert not plan.locked
    # 确认入场：新增确认 ID 重置静止计数
    confirmed = _rec('late', confirmed=True)
    plan.update([confirmed], now=6.0)
    assert not plan.locked
    plan.update([confirmed], now=7.0)
    plan.update([confirmed], now=8.0)
    assert plan.locked
    assert plan.locked_ids == ('late',)


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
    """未确认记录不进累积集：确认进行中窗口保持打开，消失后按空集锁定."""
    plan = _plan()
    for frame in range(4):
        plan.update([_rec('a', confirmed=False)], now=float(frame + 1))
        assert not plan.locked  # 确认进行中，窗口不按静止关闭
    # 未确认记录消失（TTL 清除）后静止条件生效，空集锁定
    plan.update([], now=5.0)
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


def _locked_ab_plan(**kwargs):
    """锁定 ('a','b') 两目标的计划（a 近优先，selected='a'）；4 帧关窗."""
    plan = _plan(**kwargs)
    for frame in range(4):
        plan.update([_rec('a', distance=0.5), _rec('b', distance=0.8)],
                    now=float(frame + 1))
    assert plan.locked and plan.locked_ids == ('a', 'b')
    assert plan.selected_target_id == 'a'
    return plan


def test_anchor_stale_excludes_selection_and_recovers():
    """
    阶段 D1（协议 2.4）：LOST 超 anchor_max_age 视为不可选但不移除.

    LOST 帧龄按 update 帧计数（节点按秒级配置 ÷ 帧率 EMA 折算帧数阈值）。
    陈旧期去选并按固定序重选；重新被观测帧龄归零，恢复可选。
    """
    plan = _locked_ab_plan(anchor_max_age_frames=3, anchor_drop_frames=100)
    # a 消失：帧龄 1..3 未超阈，选中保持（锚点回填期内粘性选中语义不变）
    for frame in range(5, 8):
        plan.update([_rec('b', distance=0.8)], now=float(frame))
        assert plan.selected_target_id == 'a'
        assert 'a' not in plan.anchor_stale_ids
    # 帧龄 4 > 3：入 anchor_stale_ids、去选 a、按序重选 b；a 不移除
    plan.update([_rec('b', distance=0.8)], now=8.0)
    assert plan.anchor_stale_ids == {'a'}
    assert plan.selected_target_id == 'b'
    assert 'a' in plan.locked_ids
    # 重新被观测：帧龄归零，自动恢复可选（不回跳 selected——不跳跃）
    plan.update([_rec('a', distance=0.5), _rec('b', distance=0.8)], now=9.0)
    assert plan.anchor_stale_ids == set()
    assert plan.selected_target_id == 'b'
    # b 完成后 a 可被正常重选
    assert plan.complete_selected() == 'a'


def test_anchor_drop_removes_from_plan_and_records():
    """
    阶段 D1（协议 2.4）：LOST 超 anchor_drop 从计划移除并记账.

    移除入口语义：locked_ids/priorities 剔除、dropped_ids 记录、
    pop_dropped 供节点取走记 target_dropped 事件、selected 自动顺延。
    """
    plan = _locked_ab_plan(anchor_max_age_frames=2, anchor_drop_frames=4)
    # a 消失：帧龄 3（>2 陈旧）→4 未超 drop，5 > 4 移除
    plan.update([_rec('b', distance=0.8)], now=5.0)   # 龄 1
    plan.update([_rec('b', distance=0.8)], now=6.0)   # 龄 2
    plan.update([_rec('b', distance=0.8)], now=7.0)   # 龄 3 → 陈旧
    assert plan.anchor_stale_ids == {'a'}
    assert plan.selected_target_id == 'b'
    plan.update([_rec('b', distance=0.8)], now=8.0)   # 龄 4，未超
    assert 'a' in plan.locked_ids
    plan.update([_rec('b', distance=0.8)], now=9.0)   # 龄 5 > 4 → 移除
    assert plan.locked_ids == ('b',)
    assert plan.priority('a') == 0
    assert plan.dropped_ids == {'a'}
    assert plan.pop_dropped() == ['a']
    assert plan.pop_dropped() == []     # 队列一次性取走
    assert plan.selected_target_id == 'b'


def test_out_of_view_target_is_not_selectable():
    """
    阶段 D1（协议 2.4）：OUT_OF_VIEW 目标视为不可选（复扫无益）.

    单位姿模型下回到同一拍照位姿也看不到已走出视野的目标；节点按
    「消失前最后检测框触图像边缘」分类并逐帧注入 out_of_view_ids。
    """
    plan = _locked_ab_plan(anchor_max_age_frames=100, anchor_drop_frames=200)
    plan.update([_rec('b', distance=0.8)], now=5.0, out_of_view_ids={'a'})
    assert plan.out_of_view_ids == {'a'}
    assert plan.selected_target_id == 'b'     # 去选 a 按序重选 b
    # 非锁定 ID 注入被忽略（防御交集）
    plan.update([_rec('b', distance=0.8)], now=6.0,
                out_of_view_ids={'a', 'ghost'})
    assert plan.out_of_view_ids == {'a'}
    # b 完成：a 出视野仍不可选 → selected 空串
    assert plan.complete_selected() == ''
    # a 重新被观测且不再注入出视野 → 恢复可选
    plan.update([_rec('a', distance=0.5)], now=7.0)
    assert plan.out_of_view_ids == set()
    assert plan.selected_target_id == 'a'


def test_swinging_target_is_not_selectable():
    """
    阶段 D1（协议 2.4）：record 带 target_swinging 阻断旗标视为不可选.

    摆动由 TargetRegistry 观测残差连击判定、节点打入 record 旗标；
    选中目标起摆即去选重选，平息后（旗标消失）恢复可选但不回跳。
    """
    plan = _locked_ab_plan(anchor_max_age_frames=100, anchor_drop_frames=200)
    swinging_a = _rec('a', distance=0.5, flags=('target_swinging',))
    plan.update([swinging_a, _rec('b', distance=0.8)], now=5.0)
    assert 'a' not in plan.current_selectable_ids
    assert plan.selected_target_id == 'b'
    # 全部可选目标都摆动时 selected 为空串（不跳跃，恢复后重选）
    plan.update([swinging_a, _rec('b', distance=0.8,
                                  flags=('target_swinging',))], now=6.0)
    assert plan.selected_target_id == ''
    plan.update([_rec('a', distance=0.5), _rec('b', distance=0.8)], now=7.0)
    assert plan.selected_target_id == 'a'


def test_reset_clears_outdoor_state():
    """阶段 D1 状态随 reset 一并复位（新一轮不带入陈旧/出视野/移除账）."""
    plan = _locked_ab_plan(anchor_max_age_frames=2, anchor_drop_frames=3)
    plan.update([_rec('b', distance=0.8)], now=5.0, out_of_view_ids=set())
    plan.update([_rec('b', distance=0.8)], now=6.0)
    plan.update([_rec('b', distance=0.8)], now=7.0)
    plan.update([_rec('b', distance=0.8)], now=8.0)   # 龄 4 > 3 → 移除 a
    assert plan.dropped_ids == {'a'}
    plan.reset()
    assert plan.anchor_stale_ids == set()
    assert plan.out_of_view_ids == set()
    assert plan.dropped_ids == set()
    assert plan.pop_dropped() == []
    assert plan.locked_ids == ()


def test_anchor_frame_thresholds_validated_and_writable():
    """锚点帧龄阈值：≥1 校验，运行期可按帧率 EMA 折算逐帧改写（I4）."""
    plan = _plan()
    with pytest.raises(ValueError):
        plan.anchor_max_age_frames = 0
    with pytest.raises(ValueError):
        plan.anchor_drop_frames = 0
    plan.anchor_max_age_frames = 60    # 帧率 10 fps 时 30 s → 300 帧等
    plan.anchor_drop_frames = 240
    assert plan.anchor_max_age_frames == 60
    assert plan.anchor_drop_frames == 240


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


def test_collecting_count_tracks_window_progress_then_lock_size():
    """
    R-D8：collecting_count 锁定前=累积已确认数，锁定后=锁定集大小.

    节点把本属性填入 target_observations.collecting_count 与 harvest_state
    同名键，本测试锁定其语义即锁定 msg 字段填充契约。
    """
    plan = _plan()
    assert plan.collecting_count == 0
    plan.update([_rec('a')], now=1.0)
    assert not plan.locked
    assert plan.collecting_count == 1
    plan.update([_rec('a'), _rec('b', distance=0.5)], now=2.0)
    assert plan.collecting_count == 2
    # 未确认记录不进累积集（确认中不计入 collecting_count）
    plan.update([_rec('a'), _rec('b', distance=0.5),
                 _rec('c', confirmed=False)], now=3.0)
    assert plan.collecting_count == 2
    # 第 4 帧窗口关闭锁定：语义切换为锁定集大小（== target_count）
    plan.update([_rec('a'), _rec('b', distance=0.5)], now=4.0)
    assert plan.locked
    assert plan.collecting_count == plan.target_count == 2
    # 锁定后目标暂时丢失不影响 collecting_count
    plan.update([_rec('a')], now=5.0)
    assert plan.collecting_count == 2


def test_registry_pending_count_tracks_confirmation():
    """
    R-D8：pending_count=未转正记录数，累计命中满 confirm_frames 归零.

    节点把本属性填入 target_observations.pending_count 与 harvest_state
    同名键（锁定后/身份记忆禁用时节点侧直接给 0）。
    """
    registry = TargetRegistry(confirm_frames=2, tentative_ttl_frames=5)
    assert registry.pending_count == 0
    registry.begin_frame()
    tid, is_new = registry.match_or_register(
        [0.0, 0.0, 1.0], class_id=0, now=1.0)
    assert is_new
    assert registry.pending_count == 1
    # 第二次命中后 obs_count=2 满 confirm_frames → 转正，pending 归零
    registry.begin_frame()
    tid2, is_new2 = registry.match_or_register(
        [0.0, 0.0, 1.01], class_id=0, now=2.0)
    assert (tid2, is_new2) == (tid, False)
    assert registry.pending_count == 0


def test_timing_metrics_snapshot_keys_and_ema_monotonic():
    """
    推理耗时 EMA：快照含全部分段键与 fps，EMA 向新样本单调收敛.

    harvest_state JSON 的 timing 子对象即本快照，本测试锁定键存在性
    （detect/segment/geometry/total + fps）与 EMA 单调收敛性质。
    """
    metrics = TimingMetrics(alpha=0.5)
    # 无样本时仅含 fps 键（None 安全）
    assert metrics.snapshot() == {'fps': 0.0}
    for key in ('detect_ms', 'segment_ms', 'geometry_ms', 'total_ms'):
        metrics.record(key, 10.0)
    snap = metrics.snapshot(fps=5.0)
    for key in ('detect_ms', 'segment_ms', 'geometry_ms', 'total_ms'):
        assert snap[key] == 10.0
    assert snap['fps'] == 5.0
    # EMA 单调收敛：持续记录更大样本，估计值单调递增且不越过样本
    prev = snap['total_ms']
    for _ in range(5):
        metrics.record('total_ms', 100.0)
        current = metrics.snapshot()['total_ms']
        assert prev < current <= 100.0
        prev = current
    # 脏样本（nan/负值）不进 EMA，不污染估计
    metrics.record('total_ms', float('nan'))
    metrics.record('total_ms', -1.0)
    assert metrics.snapshot()['total_ms'] == prev
