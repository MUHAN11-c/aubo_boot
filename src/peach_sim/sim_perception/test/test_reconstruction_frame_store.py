"""精确同戳RGB-D等待延迟掩膜：按新鲜度保留，按字节和帧数有界."""
from types import SimpleNamespace
import weakref

from builtin_interfaces.msg import Time
import numpy as np
from sim_perception.vision.target_reconstruction.reconstruction_core import ReconstructionCore


def _store(max_frame_age_s=2., mask_stamp_tolerance_s=0.):
    params = SimpleNamespace(
        capture=SimpleNamespace(
            max_frame_age_s=max_frame_age_s,
            mask_stamp_tolerance_s=mask_stamp_tolerance_s),
        local_volume=SimpleNamespace(size_x=.3, size_y=.3, size_z=.4))
    return ReconstructionCore(
        collector=None, mask_gate=None, kind_memory=None, params=params,
        algo_clock=None, logger=None, timing=None, throttle=None, icp_target_cache=None)


def _frame(t, size=8):
    ns = round(t * 1e9)
    return (np.zeros(size, dtype=np.uint8), np.zeros(size, dtype=np.uint16), {},
            Time(sec=ns // 1000000000, nanosec=ns % 1000000000), t, 'camera_optical')


def test_delayed_mask_retains_exact_frame_within_existing_freshness_window():
    store = _store()
    original = _frame(10.)
    store._push_frame_ring(original)
    for i in range(1, 18):
        store._push_frame_ring(_frame(10. + i / 11.))
    store._target_masks = {10000000000: object()}
    assert store._select_cached_frame() is original
    assert store._select_cached_frame(10., 'camera_optical') is original


def test_age_horizon_and_out_of_order_eviction_use_exposure_time():
    store = _store()
    for stamp in (10., 11.9, 11., 12.01, 9.):
        store._push_frame_ring(_frame(stamp))
    assert 10000000000 not in store._frame_ring
    assert 9000000000 not in store._frame_ring
    assert store._select_cached_frame()[4] == 12.01
    assert store._latest_frame[4] == 12.01


def test_byte_limit_and_duplicate_stamp_replacement_are_accounted():
    store = _store()
    store._frame_ring_max_bytes = 60
    for stamp in (10., 10.1, 10.2):
        store._push_frame_ring(_frame(stamp))  # each 24 bytes
    assert sorted(store._frame_ring) == [10100000000, 10200000000]
    assert store._frame_ring_bytes == 48
    store._push_frame_ring(_frame(10.2, size=4))
    assert store._frame_ring_bytes == 36
    store._push_frame_ring(_frame(10.3, size=100))
    assert store._frame_ring_bytes <= 60
    assert store._latest_frame is None or store._latest_frame[4] != 10.3


def test_frame_cap_and_clear_release_ring_and_latest_references():
    store = _store()
    store._frame_ring_max = 3
    for stamp in (10., 10.1, 10.2, 10.3):
        store._push_frame_ring(_frame(stamp))
    assert len(store._frame_ring) == 3
    reference = weakref.ref(store._latest_frame[0])
    store._clear_frame_ring()
    assert reference() is None
    assert store._frame_ring_bytes == 0
    assert store._select_cached_frame() is None


def test_low_rate_horizon_and_reset_accept_new_clock_origin():
    store = _store()
    store._push_frame_ring(_frame(10.))
    store._push_frame_ring(_frame(12.))
    assert store._select_cached_frame(10.) is not None  # 原2s新鲜度边界含等号。
    store._push_frame_ring(_frame(15.))
    assert store._select_cached_frame(12.) is None
    store._clear_frame_ring()
    restarted = _frame(1.)
    store._push_frame_ring(restarted)
    assert store._select_cached_frame() is restarted
    assert store._frame_ring_bytes == 24


def test_lagged_mask_kept_across_perception_latency():
    store = _store(max_frame_age_s=8.)
    original = _frame(10.)
    store._push_frame_ring(original)
    for i in range(1, 70):
        store._push_frame_ring(_frame(10. + i / 12.))
    store._target_masks = {10000000000: object()}
    assert store._select_cached_frame() is original


def test_select_cached_frame_uses_mask_tolerance_not_latest():
    store = _store(mask_stamp_tolerance_s=0.45)
    early = _frame(10.)
    store._push_frame_ring(early)
    store._push_frame_ring(_frame(10.5))
    store._target_masks = {10020000000: object()}
    assert store._select_cached_frame() is early


def test_oversized_frame_does_not_advance_time_watermark_or_evict_waiting_frame():
    store = _store()
    store._frame_ring_max_bytes = 60
    waiting = _frame(10.)
    store._push_frame_ring(waiting)
    store._push_frame_ring(_frame(13., size=30))  # 90B不能进入60B缓存。
    assert store._select_cached_frame(10.) is waiting
    store._push_frame_ring(_frame(10.1))
    assert len(store._frame_ring) == 2
