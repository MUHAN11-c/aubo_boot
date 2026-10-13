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

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import frozen_camera_replay as replay  # noqa: E402, I100


COLOR = '/camera/color/image_raw'
DEPTH = '/camera/depth/image_raw'
POINTS = '/camera/depth_registered/points'
COLOR_INFO = '/camera/color/camera_info'
DEPTH_INFO = '/camera/depth/camera_info'


def _message(topic, stamp_ns):
    msg = replay.TOPICS[topic]()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    msg.header.frame_id = (
        'camera_color_optical_frame' if 'color' in topic
        else 'camera_depth_optical_frame')
    msg.width = msg.height = 2
    if isinstance(msg, CameraInfo):
        msg.k = [100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]
    elif isinstance(msg, Image):
        msg.encoding = 'bgr8' if topic == COLOR else '16UC1'
        msg.step = 6 if topic == COLOR else 4
        msg.data = bytes(msg.height * msg.step)
    elif isinstance(msg, PointCloud2):
        msg.point_step = 12
        msg.row_step = msg.width * msg.point_step
        msg.data = bytes(msg.height * msg.row_step)
    return msg


def _write_bag(tmp_path, samples):
    """Write tiny real CDR messages, without initializing a ROS graph."""
    uri = str(tmp_path / 'bag')
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    try:
        for topic, kind in replay.TOPICS.items():
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0, name=topic, type=f'sensor_msgs/msg/{kind.__name__}',
                serialization_format='cdr'))
        for i, (topic, msg) in enumerate(samples):
            writer.write(topic, serialize_message(msg), (i + 1) * 10**9)
    finally:
        writer.close()
    return uri


def _calibration():
    return [(topic, _message(topic, 1_000_000_000))
            for topic in (COLOR_INFO, DEPTH_INFO)]


def test_load_frame_matches_header_stamp_instead_of_topic_index(tmp_path):
    samples = _calibration() + [
        (DEPTH, _message(DEPTH, 9_000_000_000)),
        (POINTS, _message(POINTS, 2_000_000_000)),
        (COLOR, _message(COLOR, 10_000_000_000)),
        (DEPTH, _message(DEPTH, 10_020_000_000)),
        (POINTS, _message(POINTS, 10_015_000_000)),
    ]
    frame = replay.load_frame(_write_bag(tmp_path, samples), 0)
    assert frame[DEPTH].header.stamp.nanosec == 20_000_000
    assert frame[POINTS].header.stamp.sec == 10


@pytest.mark.parametrize('depth_stamp,cloud_stamp', [
    (9_800_000_000, 78_438_000_000),  # Recorded 200 ms / 68 s mismatch.
    (9_960_000_000, 10_040_000_000),  # Each near RGB, total span is 80 ms.
    (10_050_000_000, 10_000_000_000),  # Jazzy ATS requires span < slop.
])
def test_load_frame_rejects_unsynchronized_original_exposures(
        tmp_path, depth_stamp, cloud_stamp):
    samples = _calibration() + [
        (COLOR, _message(COLOR, 10_000_000_000)),
        (DEPTH, _message(DEPTH, depth_stamp)),
        (POINTS, _message(POINTS, cloud_stamp)),
    ]
    with pytest.raises(RuntimeError, match='synchronized'):
        replay.load_frame(_write_bag(tmp_path, samples), 0)


@pytest.mark.parametrize('fault', ['size', 'intrinsic', 'unregistered', 'frame'])
def test_load_frame_rejects_incompatible_static_calibration(tmp_path, fault):
    samples = _calibration()
    if fault == 'size':
        samples[0][1].width = 4
    elif fault == 'intrinsic':
        samples[0][1].k[0] = 0.0
    elif fault == 'unregistered':
        samples[1][1].k[0] = 120.0
    else:
        samples[0][1].header.frame_id = 'another_optical_frame'
    samples += [(topic, _message(topic, 10_000_000_000))
                for topic in (COLOR, DEPTH, POINTS)]
    with pytest.raises(RuntimeError, match='calibration'):
        replay.load_frame(_write_bag(tmp_path, samples), 0)


def test_load_frame_does_not_replace_requested_rgb_with_another_valid_group(tmp_path):
    samples = _calibration() + [
        (COLOR, _message(COLOR, 10_000_000_000)),
        (COLOR, _message(COLOR, 20_000_000_000)),
        (DEPTH, _message(DEPTH, 20_000_000_000)),
        (POINTS, _message(POINTS, 20_000_000_000)),
    ]
    uri = _write_bag(tmp_path, samples)
    with pytest.raises(RuntimeError, match='synchronized'):
        replay.load_frame(uri, 0)
    frame = replay.load_frame(uri, 1)
    assert frame[COLOR].header.stamp.sec == 20
    assert frame[DEPTH].header.stamp.sec == 20


def test_load_frame_rejects_cloud_in_another_coordinate_frame(tmp_path):
    samples = _calibration() + [(topic, _message(topic, 10_000_000_000))
                                for topic in (COLOR, DEPTH, POINTS)]
    samples[-1][1].header.frame_id = 'base_link'
    with pytest.raises(RuntimeError, match='calibration'):
        replay.load_frame(_write_bag(tmp_path, samples), 0)


def test_retimestamp_preserves_original_sensor_skews():
    frame = {topic: _message(topic, stamp) for topic, stamp in [
        (COLOR, 10_000_000_000), (DEPTH, 10_020_000_000),
        (POINTS, 10_015_000_000),
        (COLOR_INFO, 1_000_000_000), (DEPTH_INFO, 1_000_000_000),
    ]}
    stamp = _message(COLOR, 100_000_000_000).header.stamp
    emitted = replay.retimestamp_frame(frame, stamp)
    assert emitted[COLOR].header.stamp.sec == 100
    assert emitted[DEPTH].header.stamp.nanosec == 20_000_000
    assert emitted[POINTS].header.stamp.nanosec == 15_000_000
    assert emitted[COLOR_INFO].header.stamp == emitted[COLOR].header.stamp
    assert emitted[DEPTH_INFO].header.stamp == emitted[DEPTH].header.stamp
    assert frame[COLOR].header.stamp.sec == 10


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
