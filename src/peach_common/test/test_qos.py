"""QoS factory profile tests (rclpy required; skipped in pure-core shells)."""
from peach_common.qos import latched, reliable, stream
import pytest

qos = pytest.importorskip('rclpy.qos')


def test_latched_is_reliable_transient_depth1():
    profile = latched()
    assert profile.reliability == qos.ReliabilityPolicy.RELIABLE
    assert profile.durability == qos.DurabilityPolicy.TRANSIENT_LOCAL
    assert profile.depth == 1


def test_stream_is_reliable_volatile_keep_last():
    profile = stream()
    assert profile.reliability == qos.ReliabilityPolicy.RELIABLE
    assert profile.durability == qos.DurabilityPolicy.VOLATILE
    assert profile.history == qos.HistoryPolicy.KEEP_LAST
    assert profile.depth == 10


def test_reliable_alias_and_depth_override():
    profile = reliable()
    assert profile.reliability == qos.ReliabilityPolicy.RELIABLE
    assert profile.durability == qos.DurabilityPolicy.VOLATILE
    assert profile.depth == 10
    assert stream(depth=2).depth == 2
    assert reliable(depth=3).depth == 3
