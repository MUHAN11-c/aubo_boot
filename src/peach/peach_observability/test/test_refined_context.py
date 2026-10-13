"""原始诊断只读身份来源不得被类型化心跳或新世代污染."""
import json

from peach_interfaces.msg import (
    PeachTargetObservation, PeachTargetObservationArray, ReconstructionStatus)
from peach_observability.observability_node import ObservabilityNode
from peach_observability.state import ObservabilityState
from std_msgs.msg import String


def node():
    """实际状态转换器与消息字段，纯回调夹具不启动 ROS."""
    value = ObservabilityNode.__new__(ObservabilityNode)
    value._state = ObservabilityState()
    value._recon_debug_extra = {}
    value._recon_decision_value = None
    value._recon_typed_value = {}
    value._recon_debug_context = None
    value._traj_landmarks = {}
    value._record_raw = lambda *_args: None
    return value


def targets(value, run='run1', epoch=3):
    """生产感知数组确实拥有 scene_epoch，原始诊断不造该字段."""
    message = PeachTargetObservationArray(harvest_run_id=run, scene_epoch=epoch)
    message.observations = [PeachTargetObservation(target_id='t1')]
    value._targets_callback(message)


def raw(value, run='run1', target='t1'):
    """沿实际 diagnostics_debug 字段传送身份与无 epoch 的 refined."""
    value._recon_debug_callback(String(data=json.dumps({
        'target_id': target, 'harvest_run_id': run,
        'refined': {'approach_axis': [0, 1, 0], 'accepted': False}})))


def diagnostics(value):
    """读取实际节点诊断镜像."""
    return value._state.snapshot()['reconstruction']['diagnostics']


def test_matching_raw_identity_has_readonly_context():
    """明确身份匹配时注记原始诊断来源，raw 到达立即更新镜像."""
    value = node()
    targets(value)
    raw(value)
    assert diagnostics(value)['refined_context'] == {
        'target_id': 't1', 'harvest_run_id': 'run1', 'scene_epoch': 3}


def test_old_run_and_missing_identity_have_no_context():
    """旧批次或原始身份缺失不能借当前世代显示旧轴."""
    value = node()
    targets(value)
    for run, target in [('old', 't1'), ('', 't1'), ('run1', ''), ('run1', 't2')]:
        raw(value, run, target)
        assert 'refined_context' not in diagnostics(value)


def test_typed_target_does_not_relabel_raw_context():
    """类型化心跳可更新当前目标，但原始 refined 仍标原始身份."""
    value = node()
    targets(value)
    raw(value)
    value._recon_diagnostics_callback(ReconstructionStatus(
        harvest_run_id='run1', target_id='t2'))
    assert diagnostics(value)['target_id'] == 't2'
    assert diagnostics(value)['refined_context']['target_id'] == 't1'


def test_new_epoch_and_initial_unknown_do_not_relabel_old_raw():
    """原始诊断没有 epoch，后到的新感知世代不能追溯贴上去."""
    value = node()
    raw(value)
    targets(value)
    typed = ReconstructionStatus(harvest_run_id='run1', target_id='t1')
    value._recon_diagnostics_callback(typed)
    assert 'refined_context' not in diagnostics(value)
    raw(value)
    assert diagnostics(value)['refined_context']['scene_epoch'] == 3
    targets(value, epoch=4)
    value._recon_diagnostics_callback(typed)
    assert 'refined_context' not in diagnostics(value)
