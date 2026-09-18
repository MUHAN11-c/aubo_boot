"""Zero-ROS pipeline 纯核：阶段转移时间线 + 批次账本直播."""
import json
import time

from peach_observability.pipeline import (
    LedgerWatch,
    merge_job_landmarks,
    OUTCOME_NAMES,
    sanitize_request_id,
    StageTracker,
)


def _write_ledger(root, request_id, document):
    directory = root / request_id
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / 'ledger.json'
    path.write_text(json.dumps(document, ensure_ascii=False), encoding='utf-8')
    return path


class TestStageTracker:

    def test_records_transitions_and_durations(self):
        tracker = StageTracker(limit=10)
        assert tracker.feed(('a',), {'state': 'A'}, 100.0)
        assert not tracker.feed(('a',), {'state': 'A'}, 101.0)  # 心跳去重
        assert tracker.feed(('b',), {'state': 'B'}, 102.5)
        exported = tracker.export(103.0)
        assert len(exported) == 2
        assert exported[0]['dur_s'] == 2.5
        assert exported[0].get('current') is None
        assert exported[1]['current'] is True
        assert exported[1]['dur_s'] == 0.5

    def test_limit_trims_head(self):
        tracker = StageTracker(limit=3)
        for index in range(6):
            tracker.feed((index,), {'state': index}, float(index))
        exported = tracker.export(10.0)
        assert len(exported) == 3

    def test_reset_clears(self):
        tracker = StageTracker()
        tracker.feed(('a',), {'state': 'A'}, 1.0)
        tracker.reset()
        assert tracker.export(2.0) == []
        assert tracker.feed(('a',), {'state': 'A'}, 3.0)


class TestSanitizeRequestId:

    def test_accepts_plain(self):
        assert sanitize_request_id('field_pregrasp_20260918') == \
            'field_pregrasp_20260918'

    def test_rejects_traversal_and_empty(self):
        assert sanitize_request_id('') is None
        assert sanitize_request_id('..') is None
        assert sanitize_request_id('../evil') is None
        assert sanitize_request_id('a/b') is None
        assert sanitize_request_id('a\\b') is None


class TestLedgerWatch:

    def test_missing_file_yields_pending_error(self, tmp_path):
        watch = LedgerWatch(tmp_path)
        payload = watch.refresh('req_missing')
        assert payload['error'] is not None
        assert payload['rows'] == []
        # 同 id 未变化 → 不再刷
        assert watch.refresh('req_missing') is None

    def test_parses_rows_and_totals(self, tmp_path):
        _write_ledger(tmp_path, 'req_a', {
            'claimed': ['target_0', 'target_1'],
            'outcomes': [
                {'target_id': 'target_0', 'outcome': 0, 'reason': 'ok',
                 'quality_score': 0.8, 'elapsed_s': 12.34,
                 'stage_names': ['observe', 'approach', 'tool'],
                 'stage_durations': [3.0, 5.0, 4.0],
                 'build_view_count': 3, 'failure_code': None},
                {'target_id': 'target_1', 'outcome': 3,
                 'reason': 'full_failed', 'failure_code': 'cut_timeout',
                 'elapsed_s': 8.0},
            ],
        })
        watch = LedgerWatch(tmp_path)
        payload = watch.refresh('req_a')
        assert payload['error'] is None
        assert payload['request_id'] == 'req_a'
        assert payload['totals']['SUCCEEDED'] == 1
        assert payload['totals']['FAILED'] == 1
        assert payload['totals']['attempted'] == 2
        assert payload['totals']['claimed'] == 2
        first = payload['rows'][0]
        assert first['outcome_name'] == 'SUCCEEDED'
        assert first['stages'] == [
            {'name': 'observe', 'dur_s': 3.0},
            {'name': 'approach', 'dur_s': 5.0},
            {'name': 'tool', 'dur_s': 4.0},
        ]
        assert first['build_view_count'] == 3
        assert 'failure_code' not in first  # None 不透传
        second = payload['rows'][1]
        assert second['failure_code'] == 'cut_timeout'
        assert second['stages'] == []
        # mtime 未变 → 不刷
        assert watch.refresh('req_a') is None

    def test_request_id_change_forces_reload(self, tmp_path):
        _write_ledger(tmp_path, 'req_b',
                      {'claimed': [], 'outcomes': []})
        watch = LedgerWatch(tmp_path)
        assert watch.refresh('req_none')['error'] is not None
        payload = watch.refresh('req_b')
        assert payload['error'] is None

    def test_invalid_request_id_never_touches_disk(self, tmp_path):
        watch = LedgerWatch(tmp_path)
        assert watch.refresh('../etc') is None
        assert watch.refresh('') is None
        assert not list(tmp_path.iterdir())

    def test_corrupt_ledger_reports_error(self, tmp_path):
        directory = tmp_path / 'req_bad'
        directory.mkdir()
        (directory / 'ledger.json').write_text('{not json', encoding='utf-8')
        payload = LedgerWatch(tmp_path).refresh('req_bad')
        assert payload['error'] is not None
        assert payload['rows'] == []

    def test_empty_request_clears_once(self, tmp_path):
        _write_ledger(tmp_path, 'req_c', {'claimed': [], 'outcomes': []})
        watch = LedgerWatch(tmp_path)
        watch.refresh('req_c')
        cleared = watch.refresh('')
        assert cleared == {}
        assert watch.refresh('') is None


def test_outcome_names_match_idl():
    assert OUTCOME_NAMES == {
        0: 'SUCCEEDED', 1: 'SKIPPED_QUALITY', 2: 'SKIPPED_UNREACHABLE',
        3: 'FAILED', 4: 'CANCELED',
    }


def test_stage_tracker_wall_clock_compatible():
    """喂 time.time() 实数与 export 的 now 参数同源即可结算."""
    tracker = StageTracker()
    now = time.time()
    tracker.feed(('a',), {'state': 'A'}, now)
    exported = tracker.export(now + 0.25)
    assert abs(exported[0]['dur_s'] - 0.25) < 1e-6


class TestActionFallbackName:
    """DDS 侧 action 生成消息按 msg 命名空间登记的回退解析."""

    def test_feedback_message_maps_to_action(self):
        from peach_observability.bag_reader import action_fallback_name
        assert action_fallback_name(
            'peach_interfaces/msg/RunHarvest_FeedbackMessage') == \
            'peach_interfaces/action/RunHarvest_FeedbackMessage'

    def test_goal_and_result_suffixes(self):
        from peach_observability.bag_reader import action_fallback_name
        assert action_fallback_name(
            'peach_interfaces/msg/ExecuteTarget_Goal') == \
            'peach_interfaces/action/ExecuteTarget_Goal'
        assert action_fallback_name(
            'peach_interfaces/msg/SurveyScene_Result') == \
            'peach_interfaces/action/SurveyScene_Result'

    def test_plain_messages_untouched(self):
        from peach_observability.bag_reader import action_fallback_name
        assert action_fallback_name(
            'peach_interfaces/msg/HarvestState') is None
        assert action_fallback_name('std_msgs/msg/String') is None
        assert action_fallback_name('bad/string') is None


def test_merge_job_landmarks_prefers_job_coords():
    merged = merge_job_landmarks(
        {'perception_entry': [1.0, 2.0, 3.0], 'target_id': 'old'},
        {'target_id': 't1',
         'coords': {'perception_entry': [4.0, 5.0, 6.0]},
         'grasp': {'axis': [0.0, 0.0, 1.0]}})
    assert merged['perception_entry'] == [4.0, 5.0, 6.0]
    assert merged['axis'] == [0.0, 0.0, 1.0]
    assert merged['target_id'] == 't1'
