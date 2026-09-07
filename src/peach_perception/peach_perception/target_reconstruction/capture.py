"""Re-export split modules (behavior-preserving shim)."""
from __future__ import annotations

from peach_perception.target_reconstruction.auto_controller import AutoControllerMixin
from peach_perception.target_reconstruction.bind_holdoff import BindSwitchHoldoff
from peach_perception.target_reconstruction.capture_gate import (
    capture_gate,
    GATE_ALLOW,
    GATE_DENY,
    GATE_NEED_TF,
    GATE_SKIP,
    GateDecision,
)
from peach_perception.target_reconstruction.captured_frame import CapturedFrame
from peach_perception.target_reconstruction.frame_collector import (
    CollectorConfig,
    FrameCollector,
    STATE_COLLECTING,
    STATE_IDLE,
    STATE_READY,
)
from peach_perception.target_reconstruction.mask_gate import (
    GateResult,
    MaskContext,
    StrictMaskGate,
)
from peach_perception.target_reconstruction.skip_codes import classify_skip_reason
from peach_perception.target_reconstruction.timing import TimingStats

__all__ = [
    'AutoControllerMixin',
    'BindSwitchHoldoff',
    'CapturedFrame',
    'CollectorConfig',
    'FrameCollector',
    'GATE_ALLOW',
    'GATE_DENY',
    'GATE_NEED_TF',
    'GATE_SKIP',
    'GateDecision',
    'GateResult',
    'MaskContext',
    'STATE_COLLECTING',
    'STATE_IDLE',
    'STATE_READY',
    'StrictMaskGate',
    'TimingStats',
    'capture_gate',
    'classify_skip_reason',
]
