"""Execution-window watchdog."""
from peach_harvester.supervisor.domain.watchdog import execution_window_ok, WatchdogSample


def test_fresh_status_allows_window():
    sample = WatchdogSample(
        now_mono_s=10.0, robot_status_mono_s=9.8, robot_status_seen=True,
        timeout_s=0.5)
    ok, why = execution_window_ok(sample)
    assert ok and why == 'ok'


def test_stale_status_trips():
    sample = WatchdogSample(
        now_mono_s=10.0, robot_status_mono_s=8.0, robot_status_seen=True,
        timeout_s=0.5)
    ok, why = execution_window_ok(sample)
    assert not ok and why == 'robot_status_stale'


def test_expired_model_trips():
    sample = WatchdogSample(
        now_mono_s=10.0, robot_status_mono_s=9.9, robot_status_seen=True,
        model_valid_until_s=9.0, timeout_s=0.5)
    ok, why = execution_window_ok(sample)
    assert not ok and why == 'model_expired'


def test_cancel_trips():
    sample = WatchdogSample(
        now_mono_s=1.0, robot_status_mono_s=1.0, robot_status_seen=True,
        cancel_requested=True)
    ok, why = execution_window_ok(sample)
    assert not ok and why == 'cancel_requested'
