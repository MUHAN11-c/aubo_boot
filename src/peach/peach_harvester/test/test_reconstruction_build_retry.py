"""单机位不可观测只替换新曝光；总预算/质量拒绝保持不变."""
import threading
from types import SimpleNamespace

from peach_harvester.vision.target_reconstruction.target_reconstruction_node import (
    TargetReconstructionNode,
)
import pytest


def _host(reasons, budget=60.):
    now, waits, retries, events = [0.], [], [], []
    host = SimpleNamespace(
        _state_lock=threading.RLock(), _algo_clock=SimpleNamespace(now=lambda: now[0]),
        params=SimpleNamespace(capture=SimpleNamespace(min_views=1, build_timeout_s=budget)),
        collector=SimpleNamespace(frames=[]), _refined=None,
        _harvest_data=SimpleNamespace(append_event=events.append),
        get_logger=lambda: SimpleNamespace(warning=lambda *_args: None),
    )

    def wait(_handle, target, timeout_s):
        waits.append(timeout_s)
        if timeout_s < 2.:
            now[0] += max(0., timeout_s)
            return 'timeout', 0, target
        now[0] += 2.
        host.collector.frames = [SimpleNamespace(stamp=100. + len(waits))]
        return 'ready', 1, target

    def finalize():
        reason = reasons.pop(0)
        host._refined = SimpleNamespace(reason=reason, model_revision='r' + str(len(waits)))
        return not reason, reason or 'ok'

    host._wait_min_views, host._finalize_now = wait, finalize
    host._restart_single_view_build = lambda target, stamp: retries.append((target, stamp))
    return host, waits, retries, events


def test_transient_single_view_rejection_retries_new_exposure_then_succeeds():
    host, waits, retries, events = _host(['bag_axis_envelope_ill_conditioned', ''])
    status, count, ok, message = TargetReconstructionNode._collect_and_finalize_build(
        host, object(), 't1')
    assert status == 'ready' and count == 1 and ok
    assert waits == [60., 58.]
    assert retries == [('t1', 101.)]
    assert events[0]['reason'] == 'bag_axis_envelope_ill_conditioned'
    assert events[0]['attempt'] == 1 and events[0]['stamp_sec'] == 101.


@pytest.mark.parametrize('reason', ['bag_axis_envelope_ill_conditioned',
                                    'bag_axis_silhouette_unobservable'])
def test_permanent_unobservable_is_rejected_after_three_total_attempts(reason):
    host, waits, retries, events = _host([reason] * 3)
    status, count, ok, message = TargetReconstructionNode._collect_and_finalize_build(
        host, object(), 't1')
    assert status == 'quality_rejected' and not ok
    assert len(waits) == 3 and len(retries) == 2 and len(events) == 3
    assert reason in message and 'attempts=3' in message
    assert count == 1  # 三次同机位曝光不能冒充三个独立视角。


def test_single_deadline_is_not_reset_for_replacement_capture():
    host, waits, retries, events = _host(['bag_axis_envelope_ill_conditioned'], budget=3.)
    status, _count, ok, message = TargetReconstructionNode._collect_and_finalize_build(
        host, object(), 't1')
    assert status == 'timeout' and not ok
    assert waits == [3., 1.]
    assert len(retries) == 1 and len(events) == 1
    assert 'bag_axis_envelope_ill_conditioned' in message


def test_non_retryable_quality_rejection_stops_after_first_exposure():
    host, waits, retries, _events = _host(['fused_axis_conflict'])
    status, _count, ok, message = TargetReconstructionNode._collect_and_finalize_build(
        host, object(), 't1')
    assert status == 'quality_rejected' and not ok
    assert len(waits) == 1 and retries == [] and 'fused_axis_conflict' in message


def test_specific_geometry_failure_remains_visible_in_denied_decision():
    from peach_harvester.vision.target_reconstruction.reconstruction_core import ReconstructionCore
    from peach_harvester.vision.target_reconstruction.refine import RefitResult

    for reason in ('bag_axis_envelope_ill_conditioned', 'bag_axis_silhouette_unobservable'):
        host = SimpleNamespace(_harvest_run_id='r',
                               collector=SimpleNamespace(state='COLLECTING', target_id='t1'),
                               _refined=RefitResult(ok=False, reason=reason))
        decision = ReconstructionCore._grasp_decision(host)
        assert decision['allowed'] is False and not decision['geometry_valid']
        assert decision['reason'] == reason


def test_canceled_build_does_not_restart_capture():
    host, _waits, retries, _events = _host([])
    host._wait_min_views = lambda *_args, **_kwargs: ('canceled', 0, 't1')
    status, _count, ok, _message = TargetReconstructionNode._collect_and_finalize_build(
        host, object(), 't1')
    assert status == 'canceled' and not ok and retries == []


def test_replacement_preserves_failed_exposure_watermark_and_invalidates_generation():
    host = SimpleNamespace(
        collector=SimpleNamespace(target_center=None, frames=[object()]),
        _capture_generation=1, _target_masks={'old': object()}, _latest_candidates=None,
        _target_kind_memory=SimpleNamespace(bind=lambda *_args: None),
        _refined=object(), _bag_model=object(), _publish_all=lambda: None,
    )

    def reset():
        host._capture_generation += 1
        host.collector.frames.clear()
        host._refined = host._bag_model = None
        host._last_captured_stamp_sec = -1.

    host._reset_capture_session = reset
    host._reset_products = lambda **_kwargs: None
    host.collector.start = lambda target, _center: setattr(host.collector, 'target_id', target)
    TargetReconstructionNode._restart_single_view_build(host, 't1', 123.)
    assert host._capture_generation == 2
    assert host._last_captured_stamp_sec == 123.
    assert host.collector.frames == [] and host.collector.target_id == 't1'
    assert not host._target_masks and host._refined is None and host._bag_model is None
