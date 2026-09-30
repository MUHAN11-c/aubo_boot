"""In-memory observability snapshot (pure dicts, thread-safe store)."""
from __future__ import annotations

import threading
import time
from typing import Any

BATCH_PHASE_NAMES = {
    0: 'IDLE',
    1: 'SURVEYING',
    2: 'SELECTING',
    3: 'OBSERVING',
    4: 'HARVESTING',
    5: 'WAITING_ACK',
    6: 'PAUSED',
    7: 'COMPLETED',
    8: 'ABORTED',
}

TOOL_STATE_NAMES = {
    0: 'UNKNOWN',
    1: 'OPEN_CONFIRMED',
    2: 'CLOSING',
    3: 'CLOSED_CONFIRMED',
    4: 'OPENING',
    5: 'FAULT',
}


def stamp_to_sec(stamp) -> float | None:
    if stamp is None:
        return None
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None


def summarize_observations(
    *,
    scene_epoch: int,
    target_set_locked: bool,
    locked_target_ids: list[str],
    observations: list[dict],
    stamp_sec: float | None,
) -> dict:
    return {
        'stamp_s': stamp_sec,
        'scene_epoch': int(scene_epoch),
        'target_set_locked': bool(target_set_locked),
        'locked_target_ids': list(locked_target_ids),
        'count': len(observations),
        'observations': observations,
    }


def observation_row(
    *,
    target_id: str,
    category: int,
    confirmed: bool,
    diameter95_m: float,
    swing_known: bool,
    swing_amplitude_m: float,
    mask_quality: float,
    depth_coverage: float,
    edge_touch: bool,
    flags: list[str],
) -> dict:
    return {
        'target_id': target_id,
        'category': int(category),
        'confirmed': bool(confirmed),
        'diameter95_m': float(diameter95_m),
        'swing_known': bool(swing_known),
        'swing_amplitude_m': float(swing_amplitude_m),
        'mask_quality': float(mask_quality),
        'depth_coverage': float(depth_coverage),
        'edge_touch': bool(edge_touch),
        'flags': list(flags),
    }


def model_row(
    *,
    target_id: str,
    model_revision: int,
    converged: bool,
    n_views: int,
    d95_m: float,
    length_m: float,
    sigma_lateral95_m: float,
    swing_amplitude_m: float,
) -> dict:
    return {
        'target_id': target_id,
        'model_revision': int(model_revision),
        'converged': bool(converged),
        'n_views': int(n_views),
        'd95_m': float(d95_m),
        'length_m': float(length_m),
        'sigma_lateral95_m': float(sigma_lateral95_m),
        'swing_amplitude_m': float(swing_amplitude_m),
    }


def summarize_diagnostics(status_list: list[dict]) -> dict:
    worst = 0
    for item in status_list:
        try:
            level = int(item.get('level', 0))
        except (TypeError, ValueError):
            level = 0
        worst = max(worst, level)
    return {'status_count': len(status_list), 'worst_level': worst}


class SnapshotStore:
    """Thread-safe mirror updated by ROS callbacks."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data: dict[str, Any] = {
            'read_only': True,
            'task': None,
            'tool': None,
            'recovery_required': None,
            'enables': None,
            'models': None,
            'observations': None,
            'diagnostics': {'statuses': [], 'summary': summarize_diagnostics([])},
            'session_bag': {'enabled': False, 'running': False, 'directory': None},
            'topic_ages_s': {},
        }

    def set_session_bag_info(self, info: dict) -> None:
        with self._lock:
            self._data['session_bag'] = dict(info)

    def touch_topic(self, key: str, now: float | None = None) -> None:
        ts = float(now if now is not None else time.time())
        with self._lock:
            ages = dict(self._data.get('topic_ages_s') or {})
            ages[key] = 0.0
            self._data['topic_ages_s'] = ages
            self._data['updated_at'] = ts

    def set_task(self, payload: dict | None) -> None:
        with self._lock:
            self._data['task'] = payload
            self.touch_topic_unlocked('task')

    def set_tool(self, payload: dict | None) -> None:
        with self._lock:
            self._data['tool'] = payload
            self.touch_topic_unlocked('tool')

    def set_recovery_required(self, value: bool | None) -> None:
        with self._lock:
            self._data['recovery_required'] = value
            self.touch_topic_unlocked('recovery_required')

    def set_enables(self, payload: dict | None) -> None:
        with self._lock:
            self._data['enables'] = payload
            self.touch_topic_unlocked('enables')

    def set_models(self, payload: dict | None) -> None:
        with self._lock:
            self._data['models'] = payload
            self.touch_topic_unlocked('models')

    def set_observations(self, payload: dict | None) -> None:
        with self._lock:
            self._data['observations'] = payload
            self.touch_topic_unlocked('observations')

    def set_diagnostics(self, statuses: list[dict]) -> None:
        with self._lock:
            self._data['diagnostics'] = {
                'statuses': list(statuses),
                'summary': summarize_diagnostics(statuses),
            }
            self.touch_topic_unlocked('diagnostics')

    def touch_topic_unlocked(self, key: str) -> None:
        ts = time.time()
        ages = dict(self._data.get('topic_ages_s') or {})
        ages[key] = 0.0
        self._data['topic_ages_s'] = ages
        self._data['updated_at'] = ts

    def snapshot(self) -> dict:
        with self._lock:
            return _deep_copy(self._data)

    def diagnostics_payload(self) -> dict:
        with self._lock:
            diag = self._data.get('diagnostics') or {}
            return _deep_copy(diag)

    def advance_topic_ages(self, delta_s: float) -> None:
        with self._lock:
            ages = dict(self._data.get('topic_ages_s') or {})
            self._data['topic_ages_s'] = {
                key: round(float(age) + float(delta_s), 3) for key, age in ages.items()
            }


def _deep_copy(value):
    if isinstance(value, dict):
        return {k: _deep_copy(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_deep_copy(v) for v in value]
    return value


def batch_state_dict(
    *,
    stamp_sec: float | None,
    request_id: str,
    phase: int,
    current_target_id: str,
    blockers: list[str],
    attempted: int,
    succeeded: int,
    skipped: int,
    failed: int,
    recovery_required: bool,
    message: str,
) -> dict:
    return {
        'stamp_s': stamp_sec,
        'request_id': request_id,
        'phase': int(phase),
        'phase_name': BATCH_PHASE_NAMES.get(int(phase), str(phase)),
        'current_target_id': current_target_id,
        'blockers': list(blockers),
        'attempted': int(attempted),
        'succeeded': int(succeeded),
        'skipped': int(skipped),
        'failed': int(failed),
        'recovery_required': bool(recovery_required),
        'message': message,
    }


def tool_state_dict(
    *,
    stamp_sec: float | None,
    tool_id: str,
    state: int,
    command_closed: bool,
    feedback: int,
    suspected_loopback: bool,
    actuator_current_a: float,
    fault_reason: str,
) -> dict:
    return {
        'stamp_s': stamp_sec,
        'tool_id': tool_id,
        'state': int(state),
        'state_name': TOOL_STATE_NAMES.get(int(state), str(state)),
        'command_closed': bool(command_closed),
        'feedback': int(feedback),
        'suspected_loopback': bool(suspected_loopback),
        'actuator_current_a': float(actuator_current_a),
        'fault_reason': fault_reason,
    }


def enables_dict(
    *,
    stamp_sec: float | None,
    seq: int,
    execution: bool,
    grasp: bool,
    tool: bool,
) -> dict:
    return {
        'stamp_s': stamp_sec,
        'seq': int(seq),
        'execution': bool(execution),
        'grasp': bool(grasp),
        'tool': bool(tool),
    }


def diagnostic_status_to_dict(status) -> dict:
    """Convert diagnostic_msgs/DiagnosticStatus to a JSON-safe dict."""
    values = []
    for item in getattr(status, 'values', []) or []:
        values.append({
            'key': str(getattr(item, 'key', '')),
            'value': str(getattr(item, 'value', '')),
        })
    return {
        'level': int(getattr(status, 'level', 0)),
        'name': str(getattr(status, 'name', '')),
        'message': str(getattr(status, 'message', '')),
        'hardware_id': str(getattr(status, 'hardware_id', '')),
        'values': values,
    }
