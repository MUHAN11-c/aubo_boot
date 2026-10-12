"""Bag and overlay checks for the v1 frozen-camera grasp path.

Does not start the harvest stack or require a GPU. Skips when the capture
directory is not on this machine (CI without the 10 s stereo bag).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import frozen_camera_replay as replay  # noqa: E402, I100


def _bag() -> str | None:
    env = os.environ.get('PEACH_FROZEN_BAG', '').strip()
    if env:
        return env if (Path(env) / 'metadata.yaml').is_file() else None
    return replay.discover_bag()


def test_frozen_one_view_overlay_relaxes_min_views_and_baseline():
    path = Path(__file__).resolve().parents[1] / 'config' / 'frozen_one_view.yaml'
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    recon = data['peach_target_reconstruction_node']['ros__parameters']
    capture = recon['capture']
    supervisor = data['peach_supervisor']['ros__parameters']
    scene = data['peach_scene_perception_node']['ros__parameters']
    assert capture['min_views'] == 1
    assert capture['minimum_baseline_deg'] == 0.0
    assert capture['minimum_mean_nearest_baseline_deg'] == 0.0
    assert recon['tool.budget.d_inner'] >= 0.20
    assert recon['tool.budget.blade_capture_half_width'] >= 0.02
    assert scene['harvest.min_collect_frames'] <= 3
    assert scene['harvest.lock_settle_frames'] <= 2
    assert supervisor['reconstruction_min_views'] == 1
    assert supervisor['execute_pregrasp_only'] is False
    assert supervisor['empty_survey_limit'] == 2
    assert supervisor['reconstruct_in_trajectory'] is True
    assert supervisor['survey_wait_s'] >= 15.0


def test_replay_pause_and_photo_pose_spawn():
    import inspect
    init = inspect.getsource(replay.FrozenCamera.__init__)
    tick = inspect.getsource(replay.FrozenCamera._tick)
    assert '~/pause' in init
    assert 'self._paused' in tick
    launch = Path(__file__).resolve().parents[1] / 'launch' / 'frozen_camera_grasp.launch.py'
    text = launch.read_text(encoding='utf-8')
    assert '0.425175,0.194274,1.679546,1.462579,-0.279948,0.038859' in text
    harvest = (
        Path(__file__).resolve().parents[2] / 'peach_bringup' / 'launch'
        / 'harvest_system.launch.py')
    forwarded = harvest.read_text(encoding='utf-8')
    assert "'mock_initial_positions': LaunchConfiguration(" in forwarded


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
