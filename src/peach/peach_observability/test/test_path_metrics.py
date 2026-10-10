"""Zero-ROS path_metrics (no geometry_msgs)."""
from peach_observability.path_metrics import path_metrics


def test_empty_path():
    metrics = path_metrics([])
    assert metrics['count'] == 0
    assert metrics['path_length_m'] == 0.0
    assert metrics['detour_ratio'] is None


def test_straight_line_detour_near_one():
    xyz = [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.2, 0.0, 0.0)]
    metrics = path_metrics(xyz)
    assert metrics['count'] == 3
    assert metrics['path_length_m'] == 0.2
    assert metrics['chord_m'] == 0.2
    assert metrics['detour_ratio'] == 1.0
