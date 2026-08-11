"""PeachPose ROS 编排层的三维时间戳契约回归测试."""
from pathlib import Path

import numpy as np

from peach_pose_ros2.harvest_data import HarvestDataStore
from peach_pose_ros2.harvest_plan import GlobalHarvestPlan


_NODE = (
    Path(__file__).resolve().parents[1]
    / 'peach_pose_ros2'
    / 'peach_pose_node.py'
)


def test_depth_geometry_uses_depth_stamp():
    """深度生成的三维结果必须用 Depth 时间戳查 TF 并发布."""
    source = _NODE.read_text(encoding='utf-8')
    assert 'geometry_stamp = depth_msg.header.stamp' in source
    assert 'cam_frame, geometry_stamp)' in source
    assert 'header.stamp = geometry_stamp' in source


def test_global_plan_locks_count_priority_and_selected_id():
    """首轮确认后数量和优先级固定，后续丢失不自动切换目标."""
    plan = GlobalHarvestPlan()
    records = [
        {'target_id': 'far', 'confirmed': True, 'status': 0,
         'camera_distance_m': 1.2, 'confidence': 0.9},
        {'target_id': 'near', 'confirmed': True, 'status': 0,
         'camera_distance_m': 0.7, 'confidence': 0.8},
    ]
    plan.update(records)
    assert plan.locked_ids == ('near', 'far')
    assert plan.target_count == 2
    assert plan.selected_target_id == 'near'
    plan.update([records[0]])
    assert plan.selected_target_id == 'near'
    assert plan.target_count == 2
    assert plan.complete_selected() == 'far'
    assert plan.harvest_status('near') == 'HARVESTED'
    assert plan.complete_selected() == ''
    assert plan.harvest_status('near') == 'HARVESTED'
    plan.update([
        {'target_id': 'far', 'confirmed': True, 'status': 0,
         'camera_distance_m': 1.2, 'confidence': 0.9},
        {'target_id': 'near', 'confirmed': True, 'status': 1,
         'camera_distance_m': 0.7, 'confidence': 0.8},
    ])


def test_rejected_target_waits_until_quality_recovers():
    """REJECT 目标计入总数，但恢复安全前不会自动成为 selected ID."""
    plan = GlobalHarvestPlan()
    safe = {'target_id': 'safe', 'confirmed': True, 'status': 0,
            'camera_distance_m': 0.8, 'confidence': 0.9}
    blocked = {'target_id': 'blocked', 'confirmed': True, 'status': 2,
               'camera_distance_m': 0.6, 'confidence': 0.95}
    plan.update([safe, blocked])
    assert plan.target_count == 2
    assert plan.complete_selected() == ''
    assert plan.harvest_status('blocked') == 'WAITING_QUALITY'
    blocked['status'] = 1
    plan.update([blocked])
    assert plan.selected_target_id == 'blocked'


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
