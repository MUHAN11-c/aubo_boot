"""Model identity tuple and derived allowed."""
from peach_perception.domain.model_contract import (
    allowed_from_capabilities,
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
