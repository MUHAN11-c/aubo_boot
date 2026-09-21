"""Model identity tuple and derived allowed."""
from peach_harvester.vision.domain.model_contract import (
    allowed_from_capabilities,
    allowed_from_decision,
    capabilities_from_decision,
    CAPABILITY_INVALID,
    CAPABILITY_UNKNOWN,
    CAPABILITY_VALID,
    identities_match,
    identity_complete,
    model_executable,
    ModelIdentity,
)


def _full(**overrides):
    data = {
        'run_id': 'run-a', 'scene_epoch': 1, 'target_id': 't1',
        'model_revision': 'm1', 'tool_profile_id': 'hollow_cylinder_v1',
        'calibration_revision': 'cal-1', 'config_revision': 'cfg-1'}
    data.update(overrides)
    return ModelIdentity(**data)


def test_empty_revision_not_complete():
    assert not identity_complete(_full(model_revision=''))
    assert identity_complete(_full())


def test_mismatch_wrong_tool_and_batch():
    assert not identities_match(_full(), _full(tool_profile_id='other'))
    assert not identities_match(_full(), _full(run_id='run-b'))


def test_allowed_only_from_strict_combo():
    assert allowed_from_capabilities(
        CAPABILITY_VALID, CAPABILITY_VALID, CAPABILITY_VALID, CAPABILITY_VALID)
    assert not allowed_from_capabilities(
        CAPABILITY_VALID, CAPABILITY_VALID, CAPABILITY_VALID, CAPABILITY_INVALID)
    assert not allowed_from_capabilities(
        CAPABILITY_VALID, CAPABILITY_VALID, CAPABILITY_UNKNOWN, CAPABILITY_VALID)
    assert not allowed_from_capabilities(
        CAPABILITY_INVALID, CAPABILITY_VALID, CAPABILITY_VALID, CAPABILITY_VALID)
    # pregrasp INVALID must not block allowed (does not gate sleeve/cut)
    assert allowed_from_capabilities(
        CAPABILITY_VALID, CAPABILITY_INVALID, CAPABILITY_VALID, CAPABILITY_VALID)


def test_expired_and_preview():
    ident = _full()
    ok, why = model_executable(ident, 1.0, 2.0, now_s=3.0)
    assert not ok and why == 'model_expired'
    ok, why = model_executable(ident, 1.0, 2.0, now_s=3.0, preview=True)
    assert ok and why == 'preview'
    ok, why = model_executable(_full(model_revision=''), 1.0, 2.0, now_s=1.5)
    assert not ok and why == 'identity_incomplete'


def _decision_dict(**capabilities):
    data = {
        'geometry_capability': CAPABILITY_VALID,
        'pregrasp_capability': CAPABILITY_VALID,
        'sleeve_capability': CAPABILITY_VALID,
        'cut_capability': CAPABILITY_VALID}
    data.update(capabilities)
    return data


def test_allowed_from_decision_allow_and_deny():
    """M11：dict 侧 allowed 单源派生（与 publish 消息侧同函数）."""
    assert allowed_from_decision(_decision_dict())
    assert not allowed_from_decision(
        _decision_dict(sleeve_capability=CAPABILITY_INVALID))
    assert not allowed_from_decision(
        _decision_dict(cut_capability=CAPABILITY_UNKNOWN))
    assert not allowed_from_decision(
        _decision_dict(geometry_capability=CAPABILITY_INVALID))
    # pregrasp 不进门（INVALID 不拦）
    assert allowed_from_decision(
        _decision_dict(pregrasp_capability=CAPABILITY_INVALID))


def test_capabilities_from_decision_missing_keys_default_unknown():
    # 缺键（_grasp_decision 早期返回路径）按 UNKNOWN：不许
    assert capabilities_from_decision({}) == (
        CAPABILITY_UNKNOWN, CAPABILITY_UNKNOWN,
        CAPABILITY_UNKNOWN, CAPABILITY_UNKNOWN)
    assert not allowed_from_decision({})
    # pregrasp 缺省回落 geometry 值
    got = capabilities_from_decision(
        {'geometry_capability': 0, 'sleeve_capability': 0, 'cut_capability': 0})
    assert got == (0, 0, 0, 0)
