# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""浏览器数据转换工具测试."""

import math

from peach_perception_web.codec import (
    finite_or_none,
    parse_json_text,
)


def test_parse_json_text_preserves_object_and_plain_text():
    """JSON 对象按对象返回，普通状态文本不丢失."""
    assert parse_json_text('{"state":"READY"}') == {'state': 'READY'}
    assert parse_json_text('COLLECTING', 'state') == {'state': 'COLLECTING'}


def test_finite_or_none_recurses():
    """浏览器 JSON 中不出现 NaN 或 Infinity."""
    value = {'ok': [1.0, math.nan, {'bad': math.inf}]}
    assert finite_or_none(value) == {'ok': [1.0, None, {'bad': None}]}
