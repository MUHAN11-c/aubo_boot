"""账本序列化：elapsed 与 failure_code 附加字段."""

from peach_interfaces.msg import TargetOutcome

from peach_task_executor.ledger import (
    dict_to_outcome, outcome_to_dict, set_elapsed,
)


def test_outcome_roundtrip_with_telemetry():
    """附加字段写入 dict，恢复时不影响 TargetOutcome 核心字段."""
    item = TargetOutcome()
    item.target_id = 'target_1'
    item.outcome = TargetOutcome.SKIPPED_QUALITY
    item.reason = 'observe_build_view_race: views=0 min_views=4'
    set_elapsed(item, 3.25)
    extra = {
        'failure_code': 'observe_build_view_race',
        'build_view_count': 0,
        'timeout_source': None,
        'stage_names': ['prepare', 'observe'],
        'stage_durations': [0.2, 2.8],
    }
    data = outcome_to_dict(item, extra)
    assert data['elapsed_s'] == 3.25
    assert data['failure_code'] == 'observe_build_view_race'
    assert data['build_view_count'] == 0
    assert 'timeout_source' not in data
    restored = dict_to_outcome(data)
    assert restored.target_id == 'target_1'
    assert restored.outcome == TargetOutcome.SKIPPED_QUALITY
