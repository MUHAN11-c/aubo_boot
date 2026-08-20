"""HarvestSummary 计数：从 TargetOutcome 列表现算."""
from __future__ import annotations

from builtin_interfaces.msg import Duration

from peach_interfaces.msg import HarvestSummary, TargetOutcome


def elapsed_msg(seconds: float) -> Duration:
    """单调时钟秒数 → Duration."""
    msg = Duration()
    safe = max(0.0, float(seconds))
    msg.sec = int(safe)
    msg.nanosec = int(round((safe - msg.sec) * 1e9))
    return msg


def build_summary(run_id: str, outcomes, discovered: int,
                  elapsed_s: float) -> HarvestSummary:
    """填充 HarvestSummary 计数与账本."""
    summary = HarvestSummary()
    summary.run_id = run_id
    summary.outcomes = list(outcomes)
    summary.discovered = int(max(discovered, len(summary.outcomes)))
    summary.attempted = len(summary.outcomes)
    summary.elapsed = elapsed_msg(elapsed_s)
    for item in summary.outcomes:
        code = int(item.outcome)
        if code == TargetOutcome.SUCCEEDED:
            summary.succeeded += 1
        elif code == TargetOutcome.SKIPPED_QUALITY:
            summary.skipped_quality += 1
        elif code == TargetOutcome.SKIPPED_UNREACHABLE:
            summary.skipped_unreachable += 1
        elif code == TargetOutcome.FAILED:
            summary.failed += 1
        elif code == TargetOutcome.CANCELED:
            summary.canceled += 1
    return summary
