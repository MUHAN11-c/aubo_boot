"""Capture simulation window drawables without sampling the desktop behind them."""

from pathlib import Path
import re
import struct
import subprocess

import numpy as np
from PIL import Image


def _read_xwd(path):
    """Decode the X11 TrueColor ZPixmap written by the standard xwd utility."""
    data = path.read_bytes()
    header = struct.unpack('>25I', data[:100])
    size, version, form, _, width, height = header[:6]
    order, bits, stride, red, green, blue = (
        header[7], header[11], header[12], header[14], header[15], header[16])
    if version != 7 or form != 2 or bits != 32:
        raise ValueError('Unsupported XWD window format')
    pixels = np.frombuffer(data, dtype='<u4' if order == 0 else '>u4',
                           offset=size + 12 * header[19], count=height * stride // 4)
    pixels = pixels.reshape(height, stride // 4)[:, :width]
    channels = []
    for mask in (red, green, blue):
        shift = (mask & -mask).bit_length() - 1
        channels.append((((pixels & mask) >> shift) * (255 // (mask >> shift))).astype('uint8'))
    return Image.fromarray(np.stack(channels, axis=-1))


def capture(directory):
    """Read the app's window ID itself, including when another window covers it."""
    directory = Path(directory)
    output = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True,
                                     stderr=subprocess.DEVNULL, timeout=4)
    captured = []
    for label, pattern in [('gazebo', 'Gazebo'), ('rviz', 'RViz')]:
        for line in output.splitlines():
            if pattern.lower() not in line.lower() or '1x1' in line:
                continue
            match = re.search(r'\b(0x[0-9a-f]+)\b', line)
            if not match:
                continue
            raw = directory / (label + '.xwd')
            try:
                subprocess.run(['xwd', '-silent', '-id', match.group(1), '-out', str(raw)],
                               check=True, capture_output=True, timeout=4)
                frame = _read_xwd(raw)
                if frame.width < 300 or frame.height < 200:
                    continue
                target = directory / (label + '.png')
                frame.save(target)
                captured.append(str(target))
                break
            except (OSError, ValueError, struct.error, subprocess.SubprocessError) as exc:
                (directory / (label + '-capture-error.txt')).write_text(str(exc))
            finally:
                raw.unlink(missing_ok=True)
    return captured
