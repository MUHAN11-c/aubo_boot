"""peach_bringup attach 规则与 yaml 键对账（零 ROS）."""
from __future__ import annotations

from pathlib import Path

from peach_bringup.params import _validate
from peach_bringup.yaml_params import leaf_keys

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'config' / 'bringup.yaml'


def test_yaml_has_both_node_blocks():
    bridge = leaf_keys(BRINGUP, 'peach_lifecycle_flag_bridge')
    autostart = leaf_keys(BRINGUP, 'peach_autostart_client')
    assert {'is_active_service', 'poll_hz'} <= bridge
    assert {'scene_key', 'intent', 'wait_stack_timeout_s'} <= autostart


def test_validate_rejects_nonpositive_periods():
    assert _validate('poll_hz', 0.0) is not None
    assert _validate('poll_hz', 1.0) is None
    assert _validate('wait_stack_timeout_s', -1.0) is not None
    assert _validate('scene_key', 'default') is None
