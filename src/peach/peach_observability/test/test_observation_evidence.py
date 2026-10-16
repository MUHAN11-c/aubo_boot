"""Missing nested ROS messages must not become successful Web evidence."""
from peach_interfaces.msg import (
    BagFitting, BagGraspCandidate, PeachTargetObservation,
    PeachTargetObservationArray,
)

from peach_observability.state import to_fitting, to_target_observations


def project(item):
    """Project one typed observation through the actual HTTP codec."""
    message = PeachTargetObservationArray(observations=[item])
    return to_target_observations(message)['observations'][0]


def test_missing_nested_messages_do_not_advertise_accept_or_zero_geometry():
    item = PeachTargetObservation(target_id='bag1', tracking_status=2)
    result = project(item)
    assert result['fitting'] is None
    assert result['candidate'] is None


def test_nested_evidence_must_belong_to_its_observation():
    item = PeachTargetObservation(
        target_id='bag1', fitting=BagFitting(target_id='other'),
        candidate=BagGraspCandidate(target_id='other'))
    result = project(item)
    assert result['fitting'] is None
    assert result['candidate'] is None


def test_real_zero_measurements_and_accept_enum_are_preserved():
    fitting = BagFitting(target_id='bag1', status=BagFitting.ACCEPT,
                         n_points=0, valid_depth_ratio=0.0,
                         axis_confidence=0.0)
    result = to_fitting(fitting)
    assert result['status'] == 'ACCEPT'
    assert result['n_points'] == 0
    assert result['valid_depth_ratio'] == 0.0
    assert result['axis_confidence'] == 0.0


def test_invalid_fitting_sentinels_are_null_without_changing_reject():
    fitting = BagFitting(target_id='bag1', status=BagFitting.REJECT,
                         n_points=-1, valid_depth_ratio=-1.0,
                         cylinder_rms_m=-1.0, sphere_rms_m=-1.0,
                         cylinder_inlier_ratio=-1.0, sphere_inlier_ratio=-1.0,
                         bag_diameter_upper_m=-1.0, fruit_radius_m=-1.0)
    result = to_fitting(fitting)
    assert result['status'] == 'REJECT'
    for field in ('n_points', 'valid_depth_ratio', 'diameter_m',
                  'cylinder_rms_m', 'sphere_rms_m', 'inlier_ratio'):
        assert result[field] is None


def test_memory_anchor_preserves_positions_without_fabricating_measurements():
    item = PeachTargetObservation(
        target_id='bag1', tracking_status=2,
        diagnostic_flags=['anchor_from_memory'])
    item.candidate.target_id = 'bag1'
    item.candidate.status = BagGraspCandidate.REOBSERVE
    item.candidate.entry_pose.position.x = 0.4
    result = project(item)
    assert result['candidate']['entry_position'][0] == 0.4
    assert result['candidate']['status'] == 'REOBSERVE'
    assert result['fitting'] is None
    for field in ('confidence', 'camera_distance_m'):
        assert result[field] is None
    for field in ('confidence', 'diameter_m', 'travel_m'):
        assert result['candidate'][field] is None
