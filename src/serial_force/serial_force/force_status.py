"""Per-channel status for the 0xA5 force frame (no ROS).

The STM32 uses mm=6.0 (0x3C) for *both* the full-scale 370 g point and the
open/over-range sentinel.  The two cannot be told apart from mm alone; the
raw AD can: a real full-scale pad is roughly AD 1700..2200, an open channel
sits near 3900..4095.  This module classifies a decoded frame into
OK / SATURATED / OPEN / INVALID.
"""
from __future__ import annotations

import math

OK = 0
SATURATED = 1
OPEN = 2
INVALID = 3

NAMES = {
    OK: 'OK',
    SATURATED: 'SAT',
    OPEN: 'OPEN',
    INVALID: 'INVALID',
}


def classify(ad: float, travel_mm: float,
             open_ad_threshold: float = 3000.0,
             full_travel_mm: float = 5.95) -> int:
    """Classify one channel.

    - ad >= open_ad_threshold: OPEN (wire open / floating input)
    - travel >= full_travel_mm: SATURATED (370 g / 6.0 mm full scale)
    - otherwise: OK
    - non-finite values: INVALID
    """
    if not math.isfinite(ad) or not math.isfinite(travel_mm):
        return INVALID
    if ad >= open_ad_threshold:
        return OPEN
    if travel_mm >= full_travel_mm:
        return SATURATED
    return OK
