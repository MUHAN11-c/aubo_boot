"""Unit tests for the zero-copy Image view helpers."""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from peach_scene_perception.image_codec import (
    bgr8_view,
    depth16_view,
)
import pytest


def _message(data, *, encoding, width, height, step, bigendian=False):
    return SimpleNamespace(
        data=data, encoding=encoding, width=width, height=height,
        step=step, is_bigendian=bigendian)


def test_bgr8_view_shares_payload_and_keeps_padding():
    """BGR view must share memory and skip row padding."""
    payload = bytearray(range(8))
    msg = _message(payload, encoding='bgr8', width=1, height=2, step=4)
    view = bgr8_view(msg)
    assert view.shape == (2, 1, 3)
    assert np.shares_memory(view, np.frombuffer(payload, dtype=np.uint8))
    assert view[0, 0, 0] == 0
    assert view[1, 0, 2] == 6


def test_depth16_view_shares_payload():
    """Depth view must share memory for native little-endian input."""
    payload = bytearray(np.array([[1, 2], [3, 4]], dtype=np.uint16).tobytes())
    msg = _message(payload, encoding='16UC1', width=2, height=2, step=4)
    view = depth16_view(msg)
    assert view.shape == (2, 2)
    assert np.shares_memory(view, np.frombuffer(payload, dtype=np.uint16))
    assert view.tolist() == [[1, 2], [3, 4]]


def test_views_reject_unsupported_or_short_payloads():
    """Unsupported encodings and truncated data must fail closed."""
    msg = _message(bytearray(3), encoding='rgb8', width=2, height=2, step=6)
    with pytest.raises(ValueError):
        bgr8_view(msg)
    msg = _message(bytearray(3), encoding='bgr8', width=2, height=2, step=6)
    with pytest.raises(ValueError):
        bgr8_view(msg)


def test_depth16_view_handles_big_endian():
    """Network-order depth must be swapped into native values."""
    payload = bytearray(np.array([[1, 2]], dtype='>u2').tobytes())
    msg = _message(
        payload, encoding='16UC1', width=2, height=1, step=4, bigendian=True)
    view = depth16_view(msg)
    assert view.tolist() == [[1, 2]]
