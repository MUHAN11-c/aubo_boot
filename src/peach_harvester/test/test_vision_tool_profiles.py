"""零 ROS：工具档案解析纯核（结构校验/字段提取/非法输入；对错以实机为准）."""
from peach_harvester.vision.tool_profiles import parse_tool_profile
import pytest


def _archive(profile_id='adaptive_shear_v1', d_inner=0.120, **extra):
    data = {
        'profile_id': profile_id,
        'frames': {'tool_axis': {'xyz': [0.0, -0.05, 0.17]}},
        'geometry_m': {
            'D_inner': d_inner, 'D_outer': 0.215, 'L_insert': 0.060,
            'L_blade': 0.079, 'body_length': 0.09, 'body_radius': 0.11},
    }
    data.update(extra)
    return data


def test_parse_extracts_fields():
    parsed = parse_tool_profile(_archive(), 'adaptive_shear_v1')
    assert parsed == {'profile_id': 'adaptive_shear_v1', 'd_inner': 0.120,
                      'l_insert': 0.060, 'l_blade': 0.079,
                      'body_length': 0.09, 'body_radius': 0.11}


def test_parse_shear_baseline():
    # 基础剪切手基线：镜像 aubo_description/config/shear_v1.yaml 几何档
    # （三把里唯一与 bite 同代码路径的档：无 imu_follow，套入/撤退走 MTC LIN）
    data = _archive(profile_id='shear_v1', d_inner=0.080)
    data['geometry_m'].update(L_insert=0.030, L_blade=0.030,
                              body_length=0.120, body_radius=0.100)
    parsed = parse_tool_profile(data, 'shear_v1')
    assert parsed == {'profile_id': 'shear_v1', 'd_inner': 0.080,
                      'l_insert': 0.030, 'l_blade': 0.030,
                      'body_length': 0.120, 'body_radius': 0.100}


def test_parse_bite_baseline():
    parsed = parse_tool_profile(
        _archive(profile_id='bite_shear_v1', d_inner=0.030),
        'bite_shear_v1')
    assert parsed['d_inner'] == 0.030


def test_parse_rejects_id_mismatch():
    with pytest.raises(ValueError, match='mismatch'):
        parse_tool_profile(_archive(profile_id='bite_shear_v1'),
                           'adaptive_shear_v1')


def test_parse_rejects_missing_inner():
    data = _archive()
    del data['geometry_m']
    with pytest.raises(ValueError, match='D_inner'):
        parse_tool_profile(data, 'adaptive_shear_v1')


def test_parse_rejects_missing_body_envelope():
    data = _archive()
    del data['geometry_m']['body_radius']
    with pytest.raises(ValueError, match='body_length/body_radius'):
        parse_tool_profile(data, 'adaptive_shear_v1')


def test_parse_rejects_out_of_range_inner():
    with pytest.raises(ValueError, match='out of range'):
        parse_tool_profile(_archive(d_inner=0.6), 'adaptive_shear_v1')
    with pytest.raises(ValueError, match='out of range'):
        parse_tool_profile(_archive(d_inner=0.0), 'adaptive_shear_v1')


def test_parse_rejects_missing_insert_length():
    data = _archive()
    del data['geometry_m']['L_insert']
    with pytest.raises(ValueError, match='L_insert/L_blade'):
        parse_tool_profile(data, 'adaptive_shear_v1')


def test_parse_rejects_out_of_range_insert_length():
    with pytest.raises(ValueError, match='out of range'):
        parse_tool_profile(_archive() | {
            'geometry_m': {'D_inner': 0.12, 'L_insert': 0.6, 'L_blade': 0.02,
                           'body_length': 0.09, 'body_radius': 0.11}},
                           'adaptive_shear_v1')
    with pytest.raises(ValueError, match='out of range'):
        parse_tool_profile(_archive() | {
            'geometry_m': {'D_inner': 0.12, 'L_insert': 0.06, 'L_blade': 0.7,
                           'body_length': 0.09, 'body_radius': 0.11}},
                           'adaptive_shear_v1')


def test_parse_rejects_out_of_range_body_envelope():
    with pytest.raises(ValueError, match='envelope'):
        parse_tool_profile(
            _archive() | {'geometry_m': {
                'D_inner': 0.12, 'L_insert': 0.06, 'L_blade': 0.079,
                'body_length': 2.0, 'body_radius': 0.11}},
            'adaptive_shear_v1')
    with pytest.raises(ValueError, match='envelope'):
        parse_tool_profile(
            _archive() | {'geometry_m': {
                'D_inner': 0.12, 'L_insert': 0.06, 'L_blade': 0.079,
                'body_length': 0.09, 'body_radius': 0.9}},
            'adaptive_shear_v1')


def test_parse_rejects_non_mapping():
    with pytest.raises(ValueError, match='mapping'):
        parse_tool_profile(['not', 'a', 'dict'], 'adaptive_shear_v1')
