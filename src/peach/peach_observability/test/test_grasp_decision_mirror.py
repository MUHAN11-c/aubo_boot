"""Web diagnostics preserve the typed capability contract without granting motion."""
from peach_interfaces.msg import GraspDecision, TargetModel
from peach_observability.state import to_grasp_decision


def test_capability_mirror_preserves_independent_levels_and_frame():
    """Summary permission never replaces independent contact capability facts."""
    message = GraspDecision()
    message.header.frame_id = 'base_link'
    message.geometry_capability = TargetModel.CAPABILITY_VALID
    message.pregrasp_capability = TargetModel.CAPABILITY_VALID
    message.sleeve_capability = TargetModel.CAPABILITY_INVALID
    message.cut_capability = TargetModel.CAPABILITY_UNKNOWN
    message.corridor_clear = False
    message.allowed = False
    result = to_grasp_decision(message)
    assert result['frame_id'] == 'base_link'
    assert result['geometry_capability'] == 0
    assert result['pregrasp_capability'] == 0
    assert result['sleeve_capability'] == 1
    assert result['cut_capability'] == 2
    assert result['corridor_clear'] is False
    assert result['allowed'] is False


def test_model_identity_and_zero_valid_capabilities_are_preserved():
    """零是VALID而非缺省未知；身份版本按生产消息原样镜像."""
    message = GraspDecision(
        harvest_run_id='run1', target_id='t1', scene_epoch=7,
        model_revision='t1:1:2', tool_profile_id='bite_shear_v1',
        calibration_revision='cal3', config_revision='cfg4',
        geometry_capability=0, pregrasp_capability=0,
        sleeve_capability=0, cut_capability=0, allowed=True)
    result = to_grasp_decision(message)
    assert result['scene_epoch'] == 7
    assert result['calibration_revision'] == 'cal3'
    assert result['config_revision'] == 'cfg4'
    assert result['harvest_run_id'] == 'run1'
    for name in ('geometry', 'pregrasp', 'sleeve', 'cut'):
        assert result[name + '_capability'] == TargetModel.CAPABILITY_VALID
