"""
Zero-ROS tests for G1/M11: decision validity window + dict-side allowed.

G1（2026-09-20）：``_lock_decision_validity`` 的窗口走参数
``decision.validity_s``（部署默认 120s），冻结语义（0022：同 revision
心跳不续签）不变；缺键快照回退 ``MODEL_VALIDITY_S``。
M11：``_grasp_decision`` dict 侧 ``allowed`` 与 publish 消息侧同源派生。

reconstruction_core/publish 的 import 链含 ROS msg（rclpy 等）；
零 ROS 环境按仓库惯例跳过（同 test_vision_import_guard 的
msg_builders 门），colcon test（已 source 工作区）全量执行。
"""
from __future__ import annotations

import numpy as np
from peach_common.yaml_params import dict_to_ns
from peach_harvester.vision.domain.model_contract import allowed_from_decision
from peach_harvester.vision.target_reconstruction.params import (
    TargetReconstructionParams,
)
from peach_harvester.vision.target_reconstruction.refine import (
    RefitResult,
    STATUS_ACCEPT,
)
import pytest

try:
    from peach_harvester.vision.target_reconstruction.publish import (
        MODEL_VALIDITY_S,
    )
    from peach_harvester.vision.target_reconstruction.reconstruction_core import (
        ReconstructionCore,
    )
    _IMPORT_ERROR = ''
except ImportError as exc:  # 零 ROS 环境跳过整文件（见模块 docstring）
    _IMPORT_ERROR = str(exc)

pytestmark = pytest.mark.skipif(
    _IMPORT_ERROR != '', reason=f'需 ROS msg 环境: {_IMPORT_ERROR or "未加载"}')


class _Stamp:
    """builtin_interfaces/Time 形状替身（sec/nanosec 整数）."""

    def __init__(self, sec: int = 0, nanosec: int = 0):
        self.sec = int(sec)
        self.nanosec = int(nanosec)


class _FakeRosTime:
    """某时刻的 ROS Time 替身（to_msg → _Stamp）."""

    def __init__(self, t: float):
        self._t = float(t)

    def to_msg(self) -> _Stamp:
        sec = int(self._t)
        return _Stamp(sec, round((self._t - sec) * 1e9))


class _FakeClock:
    """节点时钟替身：t 可前移，模拟心跳间隔."""

    def __init__(self, t: float = 1000.0):
        self.t = float(t)

    def now(self) -> _FakeRosTime:
        return _FakeRosTime(self.t)


class _CoreHost(ReconstructionCore):
    """零 ROS 测试宿主：注入可控时钟替代 LifecycleNode.get_clock."""

    def __init__(self, params, clock: _FakeClock):
        ReconstructionCore.__init__(
            self, collector=None, mask_gate=None, kind_memory=None,
            params=params, algo_clock=None, logger=None, timing=None,
            throttle=None, icp_target_cache=None)
        self._host_clock = clock

    def get_clock(self) -> _FakeClock:
        return self._host_clock


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def _params(validity_s=None) -> TargetReconstructionParams:
    spec = {
        'frames': {'base_frame': 'base_link'},
        'session': {'root_dir': ''},
        'local_volume': {'size_x': 0.3, 'size_y': 0.3, 'size_z': 0.4},
        'refit': {'max_axis_angle_deg': 35.0},
        'tool': {'profile_id': 'hollow_cylinder_v1'},
    }
    if validity_s is not None:
        spec['decision'] = {'validity_s': validity_s}
    return TargetReconstructionParams.from_params(dict_to_ns(spec))


class _Collector:
    """_grasp_decision 只读 state/target_id 的采集器替身."""

    def __init__(self, state: str, target_id: str):
        self.state = state
        self.target_id = target_id


def _refined(sleeve: int, cut: int, revision: str = 't1:2:5') -> RefitResult:
    return RefitResult(
        ok=True, kind='cylinder', status=STATUS_ACCEPT, n_points=100,
        center=np.zeros(3), axis=np.array([0.0, 0.0, 1.0]),
        bottom=np.zeros(3), neck=np.array([0.0, 0.0, 0.2]),
        entry=np.zeros(3), diameter=0.07, d95_m=0.07, span_m=0.2,
        rmse=0.002, inlier_ratio=0.9, cut_travel_m=0.2,
        radial_margin_m=0.01, axial_margin_m=0.02, corridor_clear=True,
        budget={'sleeve_capability': sleeve, 'cut_capability': cut,
                'reason': 'ok', 'failure_code': 0},
        model_revision=revision)


# ---- G1：窗口参数化 + 冻结语义（0022）不变 ----

def test_lock_window_comes_from_decision_validity_s_param():
    clock = _FakeClock(1000.0)
    core = _CoreHost(_params(validity_s=30.0), clock)
    decision = {'model_revision': 't1:2:1'}
    core._lock_decision_validity(decision)
    assert _stamp_s(decision['valid_until']) - _stamp_s(
        decision['generated_at']) == 30.0


def test_lock_freezes_per_revision_no_heartbeat_renewal():
    """同 revision 不续签：时钟前进后 valid_until 沿用首次冻结时刻."""
    clock = _FakeClock(1000.0)
    core = _CoreHost(_params(validity_s=30.0), clock)
    first = {'model_revision': 't1:2:1'}
    core._lock_decision_validity(first)
    clock.t += 10.0
    second = {'model_revision': 't1:2:1'}
    core._lock_decision_validity(second)
    assert _stamp_s(second['valid_until']) == _stamp_s(first['valid_until'])
    assert _stamp_s(second['generated_at']) == _stamp_s(first['generated_at'])
    assert _stamp_s(first['valid_until']) == 1030.0


def test_lock_refreezes_on_new_revision_with_current_clock():
    clock = _FakeClock(1000.0)
    core = _CoreHost(_params(validity_s=30.0), clock)
    core._lock_decision_validity({'model_revision': 'a:2:1'})
    clock.t += 8.0
    fresh = {'model_revision': 'a:2:2'}
    core._lock_decision_validity(fresh)
    assert _stamp_s(fresh['generated_at']) == 1008.0
    assert _stamp_s(fresh['valid_until']) == 1038.0


def test_lock_window_falls_back_to_120s_constant_when_param_absent():
    """旧快照缺 decision 组 → 常量 fallback（与部署默认同值，G1）."""
    clock = _FakeClock(0.0)
    core = _CoreHost(_params(), clock)
    decision = {'model_revision': 't1:2:1'}
    core._lock_decision_validity(decision)
    assert _stamp_s(decision['valid_until']) - _stamp_s(
        decision['generated_at']) == 120.0
    assert MODEL_VALIDITY_S == 120.0


# ---- M11：dict 侧 allowed 与派生单源一致 ----

def test_grasp_decision_dict_allowed_matches_derivation():
    core = _CoreHost(_params(), _FakeClock())
    core.collector = _Collector('READY', 't1')
    core._refined = _refined(sleeve=0, cut=0)
    allowed_case = core._grasp_decision()
    assert allowed_case['allowed'] is True
    assert allowed_case['allowed'] == allowed_from_decision(allowed_case)

    core._refined = _refined(sleeve=1, cut=0)
    denied_case = core._grasp_decision()
    assert denied_case['allowed'] is False
    assert denied_case['allowed'] == allowed_from_decision(denied_case)


def test_grasp_decision_dict_allowed_false_on_unknown_budget():
    core = _CoreHost(_params(), _FakeClock())
    core.collector = _Collector('READY', 't1')
    refined = _refined(sleeve=0, cut=0)
    refined.budget = {}  # 无预算 → sleeve/cut UNKNOWN
    core._refined = refined
    decision = core._grasp_decision()
    assert decision['allowed'] is False
    assert decision['reason'] == 'bag_model_unavailable'


def test_grasp_decision_dict_allowed_false_when_not_ready():
    core = _CoreHost(_params(), _FakeClock())
    core.collector = _Collector('COLLECTING', 't1')
    core._refined = _refined(sleeve=0, cut=0)
    decision = core._grasp_decision()
    assert decision['allowed'] is False
    assert decision['reason'] == 'reconstruction_not_ready'
