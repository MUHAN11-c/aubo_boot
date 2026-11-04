"""ROS Image → NumPy view helpers for the hot RGB-D path (pure NumPy)."""
from __future__ import annotations

import numpy as np


def _base_bytes(message) -> np.ndarray:
    """Return the message payload as a read-only uint8 view."""
    data = message.data
    if isinstance(data, list):
        data = bytes(data)
    return np.frombuffer(data, dtype=np.uint8)


def _check_layout(message, itemsize: int, channels: int) -> int:
    """Validate tight-enough message dimensions; return row stride in pixels."""
    if message.height <= 0 or message.width <= 0 or message.step <= 0:
        raise ValueError('invalid image dimensions')
    if message.step % itemsize:
        raise ValueError('step is not aligned to pixel size')
    row_pixels = message.step // itemsize
    needed = (message.height - 1) * message.step + message.width * itemsize
    if message.data is None or len(message.data) < needed:
        raise ValueError('image payload is shorter than height/step')
    if row_pixels < message.width * channels:
        raise ValueError('row step is narrower than width')
    return row_pixels


def bgr8_view(message) -> np.ndarray:
    """Return a read-only (H, W, 3) BGR view without copying the payload."""
    if message.encoding != 'bgr8':
        raise ValueError(f'unsupported color encoding: {message.encoding}')
    _check_layout(message, 1, 3)
    base = _base_bytes(message)
    # ndarray with explicit strides preserves the ROS row padding without a copy.
    view = np.ndarray(
        shape=(message.height, message.width, 3), dtype=np.uint8,
        buffer=base, strides=(message.step, 3, 1))
    view.setflags(write=False)
    return view


def depth16_view(message) -> np.ndarray:
    """Return a read-only (H, W) uint16 depth view without copying."""
    if message.encoding not in ('16UC1', 'mono16'):
        raise ValueError(f'unsupported depth encoding: {message.encoding}')
    _check_layout(message, 2, 1)
    base = np.frombuffer(message.data, dtype=np.uint16)
    view = np.ndarray(
        shape=(message.height, message.width), dtype=np.uint16,
        buffer=base, strides=(message.step, 2))
    if message.is_bigendian:
        # ROS stores network byte order; a swapped copy is unavoidable for math.
        view = view.byteswap()
    view.setflags(write=False)
    return view
