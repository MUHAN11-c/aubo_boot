"""Lifecycle deactivate plan and heartbeat watchdog."""
from peach_harvester.supervisor.domain.lifecycle import (
    CMD_PAUSE,
    CMD_RESET,
    CMD_RESUME,
    CMD_SHUTDOWN,
    CMD_STARTUP,
    HeartbeatWatchdog,
    plan_deactivate,
    watchdog_armed_after,
)


def test_deactivate_clears_when_idle():
    plan = plan_deactivate(now_s=1.0, deadline_s=2.0, inflight=False)
    assert plan.refuse_new_goals
    assert plan.cancel_inflight
    assert plan.clear_permits
    assert not plan.recovery_required


def test_deactivate_inflight_timeout_latches_recovery():
    plan = plan_deactivate(now_s=3.0, deadline_s=2.0, inflight=True)
    assert plan.recovery_required
    assert plan.clear_permits


def test_heartbeat_missing():
    dog = HeartbeatWatchdog(timeout_s=1.0)
    dog.beat('peach_executor', 1.0)
    assert dog.missing(['peach_executor', 'peach_arm'], 1.5) == [
        'peach_arm']
    assert dog.missing(['peach_executor'], 3.0) == ['peach_executor']


def test_watchdog_rearms_after_resume_and_reset():
    assert watchdog_armed_after(CMD_STARTUP, True)
    assert not watchdog_armed_after(CMD_STARTUP, False)
    assert not watchdog_armed_after(CMD_PAUSE, True)
    assert watchdog_armed_after(CMD_RESUME, True)
    assert not watchdog_armed_after(CMD_RESUME, False)
    assert watchdog_armed_after(CMD_RESET, True)
    assert not watchdog_armed_after(CMD_RESET, False)
    assert not watchdog_armed_after(CMD_SHUTDOWN, True)
