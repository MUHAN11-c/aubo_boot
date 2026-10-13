"""采帧静止门必须看曝光时刻，而不是延迟处理时刻的关节速度."""
import threading
from types import SimpleNamespace

import message_filters
import numpy as np
from peach_harvester.vision.target_reconstruction.capture import (
    capture_gate, GATE_ALLOW,
)
from peach_harvester.vision.target_reconstruction.target_reconstruction_node import (
    TargetReconstructionNode,
)
import pytest
from rclpy.time import Time
from sensor_msgs.msg import JointState

JOINTS = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint']


def _node(stamp_ns, require_static=True):
    node = TargetReconstructionNode.__new__(TargetReconstructionNode)
    node._state_lock = threading.RLock()
    node._joint_state_cache = message_filters.Cache(
        message_filters.SimpleFilter(), cache_size=1024)
    node._joint_states_seen = False
    node._max_joint_vel = 0.0
    node.params = SimpleNamespace(
        capture=SimpleNamespace(
            max_views=24, max_frame_age_s=2.0,
            require_robot_static=require_static,
            static_joint_vel_thresh=0.03, joint_state_tolerance_s=0.1),
        frames=SimpleNamespace(base_frame='base_link'))
    node.collector = SimpleNamespace(frames=[])
    node._last_captured_stamp_sec = 0.0
    frame = (np.zeros((2, 2, 3)), np.ones((2, 2)), {},
             Time(nanoseconds=stamp_ns).to_msg(), stamp_ns / 1e9, 'camera_optical')
    node._select_cached_frame = lambda **kwargs: frame
    node._target_mask_for_frame = lambda *args: (None, '')
    node._algo_clock = SimpleNamespace(now=lambda: stamp_ns / 1e9 + 1.4)
    return node


def _joint(node, stamp_ns, velocities):
    msg = JointState()
    msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    msg.name = JOINTS[:len(velocities)]
    msg.velocity = velocities
    node._on_joint_states(msg)


def _decision(node):
    with node._state_lock:
        values, _ = node._collect_gate_values(automatic=True)
    return capture_gate(tf_available=True, **values)


def test_late_processing_cannot_admit_a_frame_taken_in_motion():
    # 16:36 实测帧晚约 1.4s 处理；曝光邻近关节速度 .798rad/s。
    stamp = 1791794190595777000
    node = _node(stamp)
    _joint(node, stamp - 5_000_000, [0.8] * 6)
    _joint(node, 1791794190602251000, [0.7983585] * 6)
    _joint(node, stamp + 1_400_000_000, [0.0] * 6)
    assert _decision(node).action != GATE_ALLOW


def test_later_movement_does_not_invalidate_a_stationary_exposure():
    node = _node(10_000_000_000)
    _joint(node, 9_995_000_000, [0.0] * 6)
    _joint(node, 10_005_000_000, [0.02] * 6)
    _joint(node, 11_400_000_000, [0.8] * 6)
    assert _decision(node).action == GATE_ALLOW


@pytest.mark.parametrize('samples', [
    [], [(9_995_000_000, [0.0] * 6)],
    [(10_005_000_000, [0.0] * 6)],
    [(9_800_000_000, [0.0] * 6), (10_200_000_000, [0.0] * 6)],
    [(10_000_000_000, [])],
    [(10_000_000_000, [float('nan')] * 6)],
    [(10_000_000_000, [float('inf')] * 6)],
    [(10_000_000_000, [0.0])],
])
def test_missing_stale_or_invalid_exposure_evidence_is_rejected(samples):
    node = _node(10_000_000_000)
    for stamp, velocities in samples:
        _joint(node, stamp, velocities)
    assert _decision(node).action != GATE_ALLOW


def test_exact_stamp_at_static_threshold_is_accepted():
    node = _node(10_000_000_000)
    _joint(node, 10_000_000_000, [0.03] * 6)
    assert _decision(node).action == GATE_ALLOW


def test_static_check_disabled_does_not_require_joint_history():
    assert _decision(_node(10_000_000_000, require_static=False)).action == GATE_ALLOW


def test_clock_rewind_cannot_reuse_old_joint_history():
    node = _node(10_000_000_000)
    _joint(node, 9_995_000_000, [0.0] * 6)
    _joint(node, 10_005_000_000, [0.0] * 6)
    _joint(node, 1_000_000_000, [0.0] * 6)
    assert _decision(node).action != GATE_ALLOW
