# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""浏览器数据转换工具测试."""

import math
from types import SimpleNamespace

from peach_perception_web.codec import (
    finite_or_none,
    harvest_event,
    parse_json_text,
    robot_status,
)


def test_parse_json_text_preserves_object_and_plain_text():
    """JSON 对象按对象返回，普通状态文本不丢失."""
    assert parse_json_text('{"state":"READY"}') == {'state': 'READY'}
    assert parse_json_text('COLLECTING', 'state') == {'state': 'COLLECTING'}


def test_finite_or_none_recurses():
    """浏览器 JSON 中不出现 NaN 或 Infinity."""
    value = {'ok': [1.0, math.nan, {'bad': math.inf}]}
    assert finite_or_none(value) == {'ok': [1.0, None, {'bad': None}]}


def test_harvest_event_converts_fields_and_details():
    """编排器事件转换为时间线条目（含严重级别名与 details 字典）."""
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=500000000)),
        sequence=42, severity=1, code='round_started',
        message='第2轮开始：感知重置完成', request_id='', run_id='run-1',
        cycle_id='cycle-7', target_id='peach_3',
        details=[SimpleNamespace(key='round', value='2')])
    event = harvest_event(message)
    assert event['stamp'] == 12.5
    assert event['sequence'] == 42
    assert event['severity'] == 1
    assert event['severity_name'] == 'WARNING'
    assert event['code'] == 'round_started'
    assert event['target_id'] == 'peach_3'
    assert event['details'] == {'round': '2'}


def test_robot_status_converts_int_fields():
    """机械臂状态各标志位原样转为 int."""
    message = SimpleNamespace(
        mode=2, e_stopped=0, drives_powered=1, motion_possible=0,
        in_motion=1, in_error=0, error_code=0)
    status = robot_status(message)
    assert status == {
        'mode': 2, 'e_stopped': 0, 'drives_powered': 1,
        'motion_possible': 0, 'in_motion': 1, 'in_error': 0, 'error_code': 0,
    }
