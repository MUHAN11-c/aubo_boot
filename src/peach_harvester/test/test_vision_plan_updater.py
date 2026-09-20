"""
W3 PlanUpdater 纯核单测：帧级计划推进/观测材料/记忆回填/状态快照.

用真实 GlobalHarvestPlan / TargetRegistry / LightingMeter / RateEstimator
驱动，仅 payload（ROS msg 对象）用鸭子类型替身——plan_updater 只透传
不读 msg 字段（degenerate 判定除外，替身提供 x/y/z）。
"""
from __future__ import annotations

from types import SimpleNamespace as NS

import numpy as np
from peach_harvester.vision.common.runtime import ManualClock
from peach_harvester.vision.scene_perception.identity import (
    CollectLockPolicy,
    GlobalHarvestPlan,
    TargetRegistry,
)
from peach_harvester.vision.scene_perception.plan_updater import (
    degenerate_candidate,
    harvest_state_dict,
    PlanUpdater,
)
from peach_harvester.vision.scene_perception.stream_metrics import (
    AdaptiveTimeout,
    LightingMeter,
    RateEstimator,
)


class _ZeroPt:
    x = y = z = 0.0


class _Candidate:
    """payload['candidate'] 替身：degenerate 判定只读 bag_bottom/bag_neck."""

    bag_bottom = _ZeroPt()
    bag_neck = _ZeroPt()


def _updater():
    clock = ManualClock(100.0)
    plan = GlobalHarvestPlan(
        max_targets=10, prefer_lower_first=True,
        anchor_max_age_frames=150, anchor_drop_frames=600,
        lock_policy=CollectLockPolicy(
            min_collect_frames=2, lock_settle_frames=1, max_collect_s=25.0))
    registry = TargetRegistry(match_radius=0.06, max_targets=10)
    pipeline = NS(
        harvest_plan=plan, target_registry=registry,
        lighting=LightingMeter(), bbox_at_edge={}, frame_rate=RateEstimator(),
        collect_window_timeout=AdaptiveTimeout(
            lower=10.0, upper=float('inf'), factor=5),
        timing=NS(snapshot=lambda fps=None: {'fps': 0.0}))
    params = NS(
        target_memory=NS(anchor_max_age_s=30.0, anchor_drop_s=120.0),
        lighting=NS(min_depth_ratio=0.35, min_conf_mean=0.3, bad_frames=5),
        tool=NS(entry_standoff=0.03))
    return PlanUpdater(pipeline, params, clock), pipeline, clock


_RECORDS = [
    {'target_id': 'target_0', 'status': 0, 'confidence': 0.9,
     'camera_distance_m': 0.8, 'confirmed': True, 'diagnostic_flags': []},
    {'target_id': 'target_1', 'status': 0, 'confidence': 0.8,
     'camera_distance_m': 0.9, 'confirmed': True, 'diagnostic_flags': []},
]

_PAYLOADS = {'target_0': {
    'mask': np.ones((4, 4), bool), 'mask_depth_ratio': 0.9,
    'candidate': _Candidate(), 'candidate_2d': None, 'fitting': None}}


def test_update_locks_and_builds_observation_specs():
    updater, pipeline, clock = _updater()
    updater.update(_RECORDS, _PAYLOADS, 42, 'target_0', None)
    clock.advance(1.0)
    outcome = updater.update(_RECORDS, _PAYLOADS, 43, 'target_0', None)
    assert outcome.target_set_locked and outcome.locked_just_now
    assert [s.target_id for s in outcome.observations] == [
        'target_0', 'target_1']
    first = outcome.observations[0]
    assert first.selected and first.mask is not None
    assert first.tracking_token == 'OBSERVED'
    assert first.camera_distance_m == 0.8 and first.confidence == 0.9
    assert outcome.observed_ids == ['target_0']
    assert outcome.frame_event['observed_target_ids'] == ['target_0']
    assert outcome.stamp_ns == 43


def test_missing_payload_lost_token_and_no_anchor_without_entry():
    updater, pipeline, clock = _updater()
    updater.update(_RECORDS, _PAYLOADS, 42, 'target_0', None)
    clock.advance(1.0)
    outcome = updater.update(_RECORDS, _PAYLOADS, 43, 'target_0', None)
    spec = outcome.observations[1]
    assert spec.payload is None
    assert spec.tracking_token == 'LOST'
    assert 'target_temporarily_lost' in spec.diagnostic_flags
    assert spec.anchor_fields is None  # registry 无该 entry → 不回填


def test_memory_anchor_fields_from_registry_entry():
    updater, pipeline, clock = _updater()
    updater.update(_RECORDS, _PAYLOADS, 42, 'target_0', None)
    clock.advance(1.0)
    updater.update(_RECORDS, _PAYLOADS, 43, 'target_0', None)
    pipeline.target_registry._targets['target_1'] = {
        'position': np.array([0.1, 0.0, 0.5]), 'axis': np.array([0, 0, 1.0]),
        'diameter': 0.06, 'confirmed': True, 'last_seen': clock.now()}
    clock.advance(1.0)
    outcome = updater.update(_RECORDS, _PAYLOADS, 44, 'target_0', None)
    spec = [s for s in outcome.observations if s.target_id == 'target_1'][0]
    assert spec.anchor_fields is not None
    assert spec.anchor_fields['bottom'].shape == (3,)
    assert spec.anchor_fields['neck'].shape == (3,)
    assert spec.anchor_fields['orientation'].w is not None


def test_harvest_state_dict_shape_unchanged():
    updater, pipeline, clock = _updater()
    updater.update(_RECORDS, _PAYLOADS, 42, 'target_0', None)
    clock.advance(1.0)
    updater.update(_RECORDS, _PAYLOADS, 43, 'target_0', None)
    state = harvest_state_dict(
        pipeline, 'run_1', 3, 'target_0', {'run_dir': 'x'})
    assert set(state) == {
        'harvest_run_id', 'snapshot_id', 'target_set_locked', 'target_count',
        'collecting_count', 'pending_count', 'target_ids',
        'completed_target_ids', 'priorities', 'selected_target_id',
        'anchor_stale_target_ids', 'out_of_view_target_ids',
        'dropped_target_ids', 'lighting', 'low_light_quality', 'scene_epoch',
        'timing', 'data'}
    assert state['target_set_locked'] and state['target_count'] == 2
    assert state['selected_target_id'] == 'target_0'


def test_degenerate_candidate_zero_landmarks():
    assert degenerate_candidate(_Candidate())
    near = NS(bag_bottom=NS(x=1e-9, y=0.0, z=0.0), bag_neck=NS(x=1.0, y=0, z=0))
    assert degenerate_candidate(near)
