import os
import re

from peach2_core import codes
import pytest

MSG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'peach2_interfaces',
                   'msg', 'FailureCode.msg')


def test_codes_mirror_failure_code_msg():
    if not os.path.exists(MSG):
        pytest.skip('peach2_interfaces sources not available')
    pattern = re.compile(r'^uint32\s+([A-Z_0-9]+)\s*=\s*(\d+)')
    expected = {}
    with open(MSG, 'r', encoding='utf-8') as f:
        for line in f:
            m = pattern.match(line.strip())
            if m:
                expected[m.group(1)] = int(m.group(2))
    assert expected, 'no constants parsed'
    assert codes.all_codes() == expected


def test_change_01_codes():
    assert codes.NECK_REMEASURE_PENDING == 28
    assert codes.TARGET_TIMEOUT == 45
    assert codes.DEPENDENCY_UNAVAILABLE == 65
    values = list(codes.all_codes().values())
    assert len(values) == len(set(values)), 'duplicate code values'
