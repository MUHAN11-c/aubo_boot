"""TCP markers: sampled path only, no unreached chord."""
from peach_observability.tcp_trajectory import build_tcp_marker_dicts


def test_tcp_markers_draw_sampled_path_not_unreached_chord():
    """LINE_LIST adjacent samples; no tcp_chord first-to-last strip."""
    xyz = [
        0.0, 0.0, 0.0,
        0.1, 0.0, 0.0,
        0.1, 0.2, 0.0,
    ]
    markers = build_tcp_marker_dicts(xyz, [5, 5, 5], {}, 'base_link')
    namespaces = {item['ns'] for item in markers if item.get('action') == 0}
    assert 'tcp_path' in namespaces
    assert 'tcp_now' in namespaces
    assert 'tcp_chord' not in namespaces
    path = next(item for item in markers if item['ns'] == 'tcp_path')
    # LINE_LIST 只连相邻采样点，不把首末点直连成未走过的弦。
    assert path['type'] == 5
    assert path['points'] == [
        [0.0, 0.0, 0.0], [0.1, 0.0, 0.0],
        [0.1, 0.0, 0.0], [0.1, 0.2, 0.0],
    ]


def test_tcp_markers_axis_is_pregrasp_to_entry_not_extruded():
    """Sleeve arrow is the commanded pregrasp→entry segment, not ±0.12/0.22 m."""
    xyz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
    pregrasp = [0.0, 0.0, 0.10]
    entry = [0.0, 0.0, 0.20]
    markers = build_tcp_marker_dicts(
        xyz, [5, 5],
        {
            'grasp_pregrasp': pregrasp,
            'grasp_entry': entry,
            'axis': [0.0, 0.0, 1.0],
        },
        'base_link')
    axis = next(item for item in markers if item['ns'] == 'grasp_axis')
    assert axis['points'] == [pregrasp, entry]


def test_tcp_markers_skip_axis_without_pregrasp():
    """Axis+entry alone must not paint a 0.34 m ray through unreached space."""
    markers = build_tcp_marker_dicts(
        [0.0, 0.0, 0.0], [0],
        {'grasp_entry': [0.4, -0.6, 0.6], 'axis': [0.0, 0.0, 1.0]},
        'base_link')
    assert all(item.get('ns') != 'grasp_axis' for item in markers)
