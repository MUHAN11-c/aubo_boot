"""
Bag and overlay checks for the v2 frozen-camera grasp path.

Does not start peach2_system or require a GPU. Skips when the capture
directory is not on this machine.
"""
from __future__ import annotations

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import frozen_camera_replay as replay  # noqa: E402, I100


def _bag() -> str | None:
    env = os.environ.get('PEACH_FROZEN_BAG', '').strip()
    if env:
        return env if (Path(env) / 'metadata.yaml').is_file() else None
    return replay.discover_bag()


def test_frozen_target_model_overlay_relaxes_remeasure_and_swing():
    share = get_package_share_directory('peach2_system_tests')
    path = Path(share) / 'config' / 'frozen_target_model.yaml'
    if not path.is_file():
        path = Path(__file__).resolve().parents[1] / 'config' / 'frozen_target_model.yaml'
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert data['cut_requires_remeasure'] is False
    assert data['require_known_swing'] is False
    assert data['min_views'] == 1
    assert data['default_max_views'] == 1
    assert data['observe_timeout_s'] <= 20.0
    assert data['converge_sigma_lateral95_m'] >= 0.05
    assert data['approach_radial_slack_m'] >= 0.04
    assert 'peach2_system_tests' in data['calibration_dir']
    assert 'frozen_calibration' in data['calibration_dir']
    assert 'frozen_tools' in data['description_config_dir']
    root = path.parent
    calib = yaml.safe_load(
        (root / 'frozen_calibration' / 'adaptive_shear_v1.yaml')
        .read_text(encoding='utf-8'))
    tool = yaml.safe_load(
        (root / 'frozen_tools' / 'adaptive_shear_v1.yaml')
        .read_text(encoding='utf-8'))
    assert calib['status'] == 'calibrated'
    assert tool['geometry_m']['L_insert'] >= 0.15
    scene = yaml.safe_load(
        (root / 'frozen_scene.yaml').read_text(encoding='utf-8'))
    assert scene['min_points_per_voxel'] >= 1000
    assert scene['hard_min_thickness_m'] >= 0.10
    assert scene['frame_timeout_s'] >= 4.0
    perception = yaml.safe_load(
        (root / 'frozen_perception.yaml').read_text(encoding='utf-8'))
    assert perception['lock']['min_stationary_frames'] <= 3
    assert perception['lock']['stable_s'] <= 1.0
    assert perception['tracker']['confirm_hits'] <= 2
    assert perception['stationary']['max_gap_s'] >= 0.4
    assert perception['sensor_qos_reliable'] is True


def test_optical_static_tfs_cover_color_and_depth():
    from geometry_msgs.msg import TransformStamped
    stamp = TransformStamped().header.stamp
    children = {tf.child_frame_id for tf in replay.optical_static_tfs(stamp)}
    assert 'camera_color_optical_frame' in children
    assert 'camera_depth_optical_frame' in children


@pytest.mark.skipif(_bag() is None, reason='frozen camera bag not present')
def test_stereo_bag_has_frozen_rgbd_frame():
    uri = _bag()
    assert uri is not None
    frame = replay.load_frame(uri, 0)
    assert set(frame) == set(replay.TOPICS)
    color = frame['/camera/color/image_raw']
    depth = frame['/camera/depth/image_raw']
    assert color.height > 0 and color.width > 0
    assert depth.height == color.height and depth.width == color.width
    cloud = frame['/camera/depth_registered/points']
    assert cloud.width > 0 or cloud.height > 0
