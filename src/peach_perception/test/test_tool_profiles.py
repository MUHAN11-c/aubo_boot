"""零 ROS：工具档案解析纯核（结构校验/字段提取/非法输入；对错以实机为准）."""
from peach_perception.tool_profiles import parse_tool_profile
import pytest


def _archive(profile_id='adaptive_cylinder_v1', d_inner=0.116, **extra):
    data = {
        'profile_id': profile_id,
        'frames': {'tool_axis': {'xyz': [0.0, 0.047, 0.16866]}},
        'geometry_m': {'D_inner': d_inner, 'D_outer': 0.120, 'L_insert': 0.200},
    }
    data.update(extra)
    return data


def test_parse_extracts_fields():
    parsed = parse_tool_profile(_archive(), 'adaptive_cylinder_v1')
    assert parsed == {'profile_id': 'adaptive_cylinder_v1', 'd_inner': 0.116}


def test_parse_hollow_baseline():
    parsed = parse_tool_profile(
        _archive(profile_id='hollow_cylinder_v1', d_inner=0.104),
        'hollow_cylinder_v1')
    assert parsed['d_inner'] == 0.104


def test_parse_rejects_id_mismatch():
    with pytest.raises(ValueError, match='mismatch'):
        parse_tool_profile(_archive(profile_id='hollow_cylinder_v1'),
                           'adaptive_cylinder_v1')


def test_parse_rejects_missing_inner():
    data = _archive()
    del data['geometry_m']
    with pytest.raises(ValueError, match='D_inner'):
        parse_tool_profile(data, 'adaptive_cylinder_v1')


def test_parse_rejects_out_of_range_inner():
    with pytest.raises(ValueError, match='out of range'):
        parse_tool_profile(_archive(d_inner=0.6), 'adaptive_cylinder_v1')
    with pytest.raises(ValueError, match='out of range'):
        parse_tool_profile(_archive(d_inner=0.0), 'adaptive_cylinder_v1')


def test_parse_rejects_non_mapping():
    with pytest.raises(ValueError, match='mapping'):
        parse_tool_profile(['not', 'a', 'dict'], 'adaptive_cylinder_v1')
