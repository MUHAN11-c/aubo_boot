"""
五点力通道状态分类（纯 Python，不依赖 ROS）.

STM32 用 ``mm=6.0``（``0x3C``）同时表示满量程 370 g 与断线/超量程哨兵；
仅凭 mm 无法区分，原始 AD 可以：有效满量程约 AD 1700–2200，断线约 3900–4095。
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
    """
    分类单个通道.

    - AD >= open_ad_threshold：断线/悬空（OPEN）；
    - 行程 >= full_travel_mm：370 g/6.0 mm 满量程（SATURATED）；
    - 非有限值：INVALID；其余 OK。
    """
    if not math.isfinite(ad) or not math.isfinite(travel_mm):
        return INVALID
    if ad >= open_ad_threshold:
        return OPEN
    if travel_mm >= full_travel_mm:
        return SATURATED
    return OK
