import os

from peach2_core.tool import load_tool_calibration, load_tool_geometry
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILES = os.path.join(HERE, '..', '..', '..', 'aubo_description', 'config')
CALIB = os.path.join(HERE, '..', '..', 'peach2_calibration', 'results')

EXPECTED = {
    'shear_v1': (0.080, 0.191, 0.030, 0.030),
    'bite_shear_v1': (0.104, 0.180, 0.030, 0.037),
    'adaptive_shear_v1': (0.120, 0.215, 0.090, 0.079),
}


@pytest.mark.parametrize('tool_id', sorted(EXPECTED))
def test_load_geometry_matches_profile(tool_id):
    tool = load_tool_geometry(tool_id, PROFILES)
    d_in, d_out, l_ins, l_blade = EXPECTED[tool_id]
    assert tool.tool_id == tool_id
    assert tool.d_inner_m == pytest.approx(d_in)
    assert tool.d_outer_m == pytest.approx(d_out)
    assert tool.l_insert_m == pytest.approx(l_ins)
    assert tool.l_blade_m == pytest.approx(l_blade)
    assert tool.wall_clearance_m == pytest.approx(0.002)


@pytest.mark.parametrize('tool_id', sorted(EXPECTED))
def test_load_calibration(tool_id):
    calib = load_tool_calibration(tool_id, CALIB)
    assert calib.status == 'design_reference'
    assert calib.w_capture_m == pytest.approx(0.008)


def test_rejects_bad_tool_id_missing_key_and_mismatch(tmp_path):
    with pytest.raises(ValueError):
        load_tool_geometry('../etc/passwd', PROFILES)
    with pytest.raises(OSError):
        load_tool_geometry('no_such_tool', PROFILES)
    (tmp_path / 'x_tool.yaml').write_text(
        'profile_id: x_tool\ngeometry_m: {D_inner: 0.1, D_outer: 0.2}\n')
    with pytest.raises(ValueError, match='L_insert'):
        load_tool_geometry('x_tool', str(tmp_path))
    (tmp_path / 'y_tool.yaml').write_text('profile_id: other\ngeometry_m: {}\n')
    with pytest.raises(ValueError, match='profile_id'):
        load_tool_geometry('y_tool', str(tmp_path))
    (tmp_path / 'z_tool.yaml').write_text(
        'status: calibrated\nw_capture_m: -0.01\ne_blade_m: 0\ne_robot_m: 0\ne_tcp_m: 0\n'
        'e_handeye_m: 0\ne_runout_m: 0\nc_wall_m: 0\nfruit_clearance_m: 0\n')
    with pytest.raises(ValueError, match='w_capture_m'):
        load_tool_calibration('z_tool', str(tmp_path))
