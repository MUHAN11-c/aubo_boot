"""零 ROS：0xA5 帧通道状态分类（OK/SAT/OPEN/INVALID）."""
from serial_force.force_status import (
    OK, SATURATED, OPEN, INVALID, NAMES, classify)


def test_ok():
    assert classify(1000, 3.0) == OK
    assert classify(1730, 5.6) == OK
    assert NAMES[OK] == 'OK'


def test_saturated_at_full_scale():
    assert classify(1730, 6.0) == SATURATED
    assert classify(2142, 6.0) == SATURATED
    assert NAMES[SATURATED] == 'SAT'


def test_open_by_ad():
    assert classify(3900, 6.0) == OPEN
    assert classify(4095, 6.0) == OPEN
    assert classify(4095, 0.0) == OPEN
    assert NAMES[OPEN] == 'OPEN'


def test_invalid():
    assert classify(float('nan'), 0.0) == INVALID
    assert classify(1000, float('nan')) == INVALID
    assert NAMES[INVALID] == 'INVALID'


def test_threshold_is_configurable():
    assert classify(2500, 0.0, open_ad_threshold=3000) == OK
    assert classify(2500, 0.0, open_ad_threshold=2000) == OPEN
