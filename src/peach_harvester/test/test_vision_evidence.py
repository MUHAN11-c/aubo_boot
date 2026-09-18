"""Sealed-replay evidence gates (stale TF, corridor, occlusion)."""
from peach_harvester.vision.domain.evidence import (
    CORRIDOR_BLOCKED,
    corridor_clear,
    corridor_status,
    CORRIDOR_UNKNOWN,
    may_commit_identity,
    occlusion_class,
    OCCLUSION_CLEAR,
    OCCLUSION_UNKNOWN,
)


def test_stale_tf_does_not_update_identity():
    assert may_commit_identity(True)
    assert not may_commit_identity(False)


def test_empty_corridor_is_not_clear():
    status = corridor_status(points_present=False, blocked=False)
    assert status == CORRIDOR_UNKNOWN
    assert not corridor_clear(status)
    assert corridor_status(points_present=True, blocked=True) == CORRIDOR_BLOCKED
    assert corridor_clear(corridor_status(points_present=True, blocked=False))


def test_missing_occlusion_is_unknown():
    assert occlusion_class(inputs_available=False) == OCCLUSION_UNKNOWN
    assert occlusion_class(
        inputs_available=True, classified=OCCLUSION_CLEAR) == OCCLUSION_CLEAR
