"""模型身份元组与许可派生（零 ROS）。空版本不能执行."""
from __future__ import annotations

from dataclasses import dataclass

CAPABILITY_VALID = 0
CAPABILITY_INVALID = 1
CAPABILITY_UNKNOWN = 2


@dataclass(frozen=True)
class ModelIdentity:
    """执行绑定身份：缺任一非空字段则不完整."""

    run_id: str = ''
    scene_epoch: int = 0
    target_id: str = ''
    model_revision: str = ''
    tool_profile_id: str = ''
    calibration_revision: str = ''
    config_revision: str = ''


def identity_complete(identity: ModelIdentity) -> bool:
    """预览以外的执行要求完整元组."""
    return bool(
        identity.run_id
        and identity.target_id
        and identity.model_revision
        and identity.tool_profile_id
        and identity.calibration_revision
        and identity.config_revision)


def identities_match(expected: ModelIdentity, actual: ModelIdentity) -> bool:
    """字段逐项相等（含 scene_epoch）."""
    return (
        expected.run_id == actual.run_id
        and int(expected.scene_epoch) == int(actual.scene_epoch)
        and expected.target_id == actual.target_id
        and expected.model_revision == actual.model_revision
        and expected.tool_profile_id == actual.tool_profile_id
        and expected.calibration_revision == actual.calibration_revision
        and expected.config_revision == actual.config_revision)


def allowed_from_capabilities(
        geometry: int, pregrasp: int, sleeve: int, cut: int) -> bool:
    """Derive allowed from geometry, sleeve, and cut VALID."""
    del pregrasp
    return (
        geometry == CAPABILITY_VALID
        and sleeve == CAPABILITY_VALID
        and cut == CAPABILITY_VALID)


def capabilities_from_decision(decision: dict) -> tuple:
    """
    Extract the four capabilities from the GraspDecision precursor dict.

    M11（2026-09-20）：``_grasp_decision()`` 的 dict 侧与
    ``publish.grasp_decision_to_msg`` 的消息侧共用本提取与
    :func:`allowed_from_capabilities`，events.jsonl / diagnostics_debug
    的 ``allowed`` 不再与类型化消息各执一词。缺能力键按 UNKNOWN；
    pregrasp 缺省回落 geometry（与 publish 旧缺省一致）。
    """
    geometry = int(decision.get('geometry_capability', CAPABILITY_UNKNOWN))
    pregrasp = int(decision.get('pregrasp_capability', geometry))
    sleeve = int(decision.get('sleeve_capability', CAPABILITY_UNKNOWN))
    cut = int(decision.get('cut_capability', CAPABILITY_UNKNOWN))
    return geometry, pregrasp, sleeve, cut


def allowed_from_decision(decision: dict) -> bool:
    """
    Derive allowed from the decision dict（dict 侧单源，M11）.

    与 ``publish.grasp_decision_to_msg`` 的消息侧共用同一提取与判定；
    见 :func:`capabilities_from_decision`。
    """
    geometry, pregrasp, sleeve, cut = capabilities_from_decision(decision)
    return allowed_from_capabilities(geometry, pregrasp, sleeve, cut)


def model_executable(
        identity: ModelIdentity, generated_s: float, valid_until_s: float,
        now_s: float, preview: bool = False) -> tuple[bool, str]:
    """空版本/过期/错时拒执行；preview 显式非执行模式除外."""
    if preview:
        return True, 'preview'
    if not identity_complete(identity):
        return False, 'identity_incomplete'
    if valid_until_s <= generated_s:
        return False, 'valid_until_not_after_generated'
    if now_s > valid_until_s:
        return False, 'model_expired'
    return True, 'ok'
