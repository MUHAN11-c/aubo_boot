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
    grasp_decision,
    harvest_event,
    parse_json_text,
    reconstruction_status,
    robot_status,
    target_observations,
)


def test_parse_json_text_preserves_object_and_plain_text():
    """JSON 对象按对象返回，普通状态文本不丢失."""
    assert parse_json_text('{"state":"READY"}') == {'state': 'READY'}
    assert parse_json_text('COLLECTING', 'state') == {'state': 'COLLECTING'}


def test_target_observations_tracking_names_include_d1_statuses():
    """跟踪状态名映射含阶段 D1 新常量（4 出画/5 深度空洞），未知值回退原文."""
    def _item(target_id, tracking_status):
        return SimpleNamespace(
            target_id=target_id, priority=0, confirmed=True, selected=False,
            harvest_status='PLANNED', tracking_status=tracking_status,
            camera_distance_m=0.0, confidence=0.0,
            candidate=SimpleNamespace(
                target_id=target_id,
                entry_pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
                bag_bottom=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                bag_neck=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                translation_direction=SimpleNamespace(x=0.0, y=0.0, z=1.0),
                bag_diameter_upper_m=0.0, suggested_travel_m=0.0,
                confidence=0.0, status=0, diagnostic_flags=[],
                strategy_id='', model_version='', calibration_version='',
                tool_version=''),
            fitting=SimpleNamespace(
                target_id=target_id, target_kind='', status=0,
                axis_confidence=0.0, valid_depth_ratio=0.0, n_points=0,
                error_budget_mm=0.0, radial_clearance_mm=0.0,
                bag_diameter_upper_m=0.0, fruit_radius_m=0.0,
                cylinder_rms_m=0.0, sphere_rms_m=0.0,
                cylinder_inlier_ratio=0.0, sphere_inlier_ratio=0.0,
                diagnostic_flags=[]),
            mask=SimpleNamespace(
                width=0, height=0,
                header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))),
            diagnostic_flags=[])

    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=0), frame_id='base_link'),
        snapshot_id=1, harvest_run_id='run-1', target_set_locked=True,
        target_count=4, selected_target_id='',
        observations=[
            _item('peach_1', 0), _item('peach_2', 4),
            _item('peach_3', 5), _item('peach_4', 9)])
    value = target_observations(message)
    names = [item['tracking_status'] for item in value['observations']]
    assert names == ['OBSERVED', 'OUT_OF_VIEW', 'DEPTH_VOID', '9']


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


def _recon_message(**overrides):
    """构造 ReconstructionStatus 形状的消息替身（SimpleNamespace 属性访问）."""
    fields = {
        'header': SimpleNamespace(
            stamp=SimpleNamespace(sec=3, nanosec=250000000)),
        'harvest_run_id': 'run-1',
        'selected_target_id': 'peach_1',
        'state': 'COLLECTING',
        'target_id': 'peach_1',
        'target_center_base': [0.4, -0.1, 0.3],
        'captured_views': 3,
        'rejected_views': 1,
        'tf_failures': 2,
        'tf_latency_ms': 1.5,
        'valid_depth_ratio': 0.7,
        'max_baseline_deg': 22.0,
        'mean_nearest_baseline_deg': 18.0,
        'view_directions': [SimpleNamespace(x=1.0, y=0.0, z=0.0)],
    }
    fields.update(overrides)
    return SimpleNamespace(**fields)


def test_reconstruction_status_maps_fields_and_coverage():
    """结构化诊断镜像：键名沿用旧 JSON 契约，方向转列表."""
    value = reconstruction_status(_recon_message())
    assert value['stamp'] == 3.25
    assert value['state'] == 'COLLECTING'
    assert value['target_id'] == 'peach_1'
    assert value['target_center_base'] == [0.4, -0.1, 0.3]
    assert value['captured_views'] == 3
    assert value['tf_latency_ms'] == 1.5
    assert value['view_coverage'] == {
        'max_baseline_deg': 22.0,
        'mean_nearest_baseline_deg': 18.0,
        'valid_depth_ratio_mean': 0.7,
    }
    assert value['view_directions'] == [[1.0, 0.0, 0.0]]


def test_reconstruction_status_invalid_scalars_become_none():
    """无效约定（标量 -1、中心全 -1）折回 None，前端按无数据渲染."""
    value = reconstruction_status(_recon_message(
        target_center_base=[-1.0, -1.0, -1.0],
        tf_latency_ms=-1.0, valid_depth_ratio=-1.0,
        max_baseline_deg=-1.0, mean_nearest_baseline_deg=-1.0,
        view_directions=[]))
    assert value['target_center_base'] is None
    assert value['tf_latency_ms'] is None
    assert value['valid_depth_ratio'] is None
    assert value['view_coverage']['max_baseline_deg'] is None
    assert value['view_directions'] == []


def test_grasp_decision_denied_drops_placeholder_geometry():
    """allowed=false：镜像只留 allowed/reason，不透出占位几何."""
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        harvest_run_id='run-1', target_id='peach_2',
        allowed=False, reason='refined_geometry_unavailable',
        entry=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        axis=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        diameter_m=0.0, rmse_m=-1.0, inlier_ratio=-1.0)
    value = grasp_decision(message)
    assert value['allowed'] is False
    assert value['reason'] == 'refined_geometry_unavailable'
    assert 'entry' not in value
    assert 'rmse_m' not in value


def test_grasp_decision_allowed_keeps_geometry():
    """allowed=true：镜像保留 entry/axis/直径/RMSE/内点比."""
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        harvest_run_id='run-1', target_id='peach_2',
        allowed=True, reason='refined_geometry_accept',
        entry=SimpleNamespace(x=0.4, y=0.1, z=0.2),
        axis=SimpleNamespace(x=0.0, y=0.0, z=1.0),
        diameter_m=0.08, rmse_m=0.004, inlier_ratio=0.9)
    value = grasp_decision(message)
    assert value['allowed'] is True
    assert value['entry'] == [0.4, 0.1, 0.2]
    assert value['axis'] == [0.0, 0.0, 1.0]
    assert value['diameter_m'] == 0.08
    assert value['rmse_m'] == 0.004
    assert value['inlier_ratio'] == 0.9
