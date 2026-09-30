from peach2_observability.snapshot import (
    batch_state_dict,
    SnapshotStore,
    summarize_diagnostics,
    summarize_observations,
)


def test_snapshot_store_thread_safe_copy():
    store = SnapshotStore()
    store.set_task(batch_state_dict(
        stamp_sec=1.0,
        request_id='batch_1',
        phase=2,
        current_target_id='target_a',
        blockers=[],
        attempted=0,
        succeeded=0,
        skipped=0,
        failed=0,
        recovery_required=False,
        message='',
    ))
    snap = store.snapshot()
    snap['task']['phase_name'] = 'MUTATED'
    again = store.snapshot()
    assert again['task']['phase_name'] == 'SELECTING'


def test_observation_summary_shape():
    payload = summarize_observations(
        scene_epoch=3,
        target_set_locked=True,
        locked_target_ids=['t1'],
        observations=[{'target_id': 't1'}],
        stamp_sec=2.0,
    )
    assert payload['count'] == 1
    assert payload['scene_epoch'] == 3


def test_diagnostics_worst_level():
    summary = summarize_diagnostics([{'level': 1}, {'level': 2}])
    assert summary['worst_level'] == 2
