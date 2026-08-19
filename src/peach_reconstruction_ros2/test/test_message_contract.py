"""
诊断/许可 dict → 类型化消息组装的契约测试（status_messages 纯函数）.

覆盖的契约面：
- 结构化字段映射（视角覆盖有效时取机位聚类口径与逐机位方向）；
- 无效值约定（标量 -1、target_center_base 全 -1、方向留空）；
- 闩锁覆盖语义：allowed=false 的 GraspDecision 几何/质量字段为零/-1
  占位且 reason 明确——消费方不得把这些占位当真实几何使用（与
  refined_pose 发空数组覆盖的惯例等价）。
"""

from peach_reconstruction_ros2.status_messages import (
    diagnostics_to_status_msg,
    grasp_decision_to_msg,
    INVALID_SCALAR,
)

from std_msgs.msg import Header


def _header() -> Header:
    header = Header()
    header.frame_id = 'base_link'
    return header


def test_status_msg_maps_core_fields_when_coverage_valid():
    """覆盖有效：摘要指标取机位均值，方向逐机位填入，ID/计数透传."""
    diag = {
        'harvest_run_id': 'run-1',
        'selected_target_id': 'peach_1',
        'state': 'COLLECTING',
        'target_id': 'peach_1',
        'target_center_base': [0.4, -0.1, 0.3],
        'captured_views': 3,
        'rejected_views': 1,
        'tf_failures': 2,
        'tf_latency_ms': 1.5,
        'valid_depth_ratio': 0.6,  # 最近帧值（覆盖有效时不用）
        'view_coverage': {
            'valid': True,
            'view_count': 3,
            'max_baseline_deg': 22.0,
            'mean_nearest_baseline_deg': 18.0,
            'valid_depth_ratio_mean': 0.7,
            'views': [
                {'direction_target_to_camera': [1.0, 0.0, 0.0]},
                {'direction_target_to_camera': [0.0, 1.0, 0.0]},
                {'direction_target_to_camera': [0.0, 0.0]},  # 退化长度跳过
            ],
        },
    }
    msg = diagnostics_to_status_msg(diag, _header())
    assert msg.header.frame_id == 'base_link'
    assert msg.harvest_run_id == 'run-1'
    assert msg.selected_target_id == 'peach_1'
    assert msg.state == 'COLLECTING'
    assert msg.target_id == 'peach_1'
    assert list(msg.target_center_base) == [0.4, -0.1, 0.3]
    assert msg.captured_views == 3
    assert msg.rejected_views == 1
    assert msg.tf_failures == 2
    assert msg.tf_latency_ms == 1.5
    assert msg.valid_depth_ratio == 0.7  # 机位均值，非最近帧值
    assert msg.max_baseline_deg == 22.0
    assert msg.mean_nearest_baseline_deg == 18.0
    assert len(msg.view_directions) == 2
    assert (msg.view_directions[0].x, msg.view_directions[0].y,
            msg.view_directions[0].z) == (1.0, 0.0, 0.0)


def test_status_msg_invalid_conventions():
    """未绑定/无帧：中心全 -1、标量 -1、方向留空（闩锁覆盖不得留陈旧值）."""
    diag = {
        'harvest_run_id': '',
        'selected_target_id': '',
        'state': 'IDLE',
        'target_id': '',
        'target_center_base': None,
        'captured_views': 0,
        'rejected_views': 0,
        'tf_failures': 0,
        'tf_latency_ms': None,
        'valid_depth_ratio': None,
        'view_coverage': {'valid': False, 'views': []},
    }
    msg = diagnostics_to_status_msg(diag, _header())
    assert list(msg.target_center_base) == [-1.0, -1.0, -1.0]
    assert msg.tf_latency_ms == INVALID_SCALAR
    assert msg.valid_depth_ratio == INVALID_SCALAR
    assert msg.max_baseline_deg == INVALID_SCALAR
    assert msg.mean_nearest_baseline_deg == INVALID_SCALAR
    assert len(msg.view_directions) == 0


def test_status_msg_depth_ratio_falls_back_to_last_frame():
    """覆盖无效但有帧：深度比回退最近帧值，基线仍 -1（区分无帧/覆盖无效）."""
    diag = {
        'target_center_base': [0.4, 0.0, 0.3],
        'captured_views': 1,
        'valid_depth_ratio': 0.55,
        'view_coverage': {'valid': False, 'views': []},
    }
    msg = diagnostics_to_status_msg(diag, _header())
    assert msg.valid_depth_ratio == 0.55
    assert msg.max_baseline_deg == INVALID_SCALAR
    assert msg.mean_nearest_baseline_deg == INVALID_SCALAR
    assert len(msg.view_directions) == 0


def test_grasp_decision_latch_overwrite_semantics():
    """闩锁覆盖契约：allowed=false 几何/质量全为占位（零/-1），reason 明确."""
    msg = grasp_decision_to_msg({
        'harvest_run_id': 'run-1',
        'target_id': 'peach_2',
        'allowed': False,
        'reason': 'refined_geometry_unavailable',
    }, _header())
    assert msg.allowed is False
    assert msg.reason == 'refined_geometry_unavailable'
    assert msg.target_id == 'peach_2'
    # 占位值不得被误用为真实几何
    assert (msg.entry.x, msg.entry.y, msg.entry.z) == (0.0, 0.0, 0.0)
    assert (msg.axis.x, msg.axis.y, msg.axis.z) == (0.0, 0.0, 0.0)
    assert msg.diameter_m == 0.0
    assert msg.rmse_m == INVALID_SCALAR
    assert msg.inlier_ratio == INVALID_SCALAR


def test_grasp_decision_accept_populates_geometry():
    """allowed=true：entry/axis/直径/RMSE/内点比按精化结果填充."""
    msg = grasp_decision_to_msg({
        'harvest_run_id': 'run-1',
        'target_id': 'peach_2',
        'allowed': True,
        'reason': 'refined_geometry_accept',
        'entry': [0.4, 0.1, 0.2],
        'axis': [0.0, 0.0, 1.0],
        'diameter_m': 0.08,
        'rmse_m': 0.004,
        'inlier_ratio': 0.9,
    }, _header())
    assert msg.allowed is True
    assert (msg.entry.x, msg.entry.y, msg.entry.z) == (0.4, 0.1, 0.2)
    assert (msg.axis.x, msg.axis.y, msg.axis.z) == (0.0, 0.0, 1.0)
    assert msg.diameter_m == 0.08
    assert msg.rmse_m == 0.004
    assert msg.inlier_ratio == 0.9
