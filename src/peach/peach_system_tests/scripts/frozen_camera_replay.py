#!/usr/bin/env python3
"""Republish one RGB-D frame from a camera bag with live timestamps.

The recorded image is fixed (photo-pose simulation). A synchronized RGB-D/cloud
group is selected using its original exposure stamps, then shifted to live time
without erasing sensor skews. Does not start a camera driver.
"""
from __future__ import annotations

import argparse
import copy
import math
import os
import sys
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, SimpleFilter
import rclpy
import rosbag2_py
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster

# peach_stereo publishStaticTf: RPY(-π/2, 0, -π/2) → optical (REP-103).
_OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)

BAG_RELATIVE = 'runs/peach_camera_capture_20260930/stereo_10s'
TOPICS = {
    '/camera/color/image_raw': Image,
    '/camera/depth/image_raw': Image,
    '/camera/color/camera_info': CameraInfo,
    '/camera/depth/camera_info': CameraInfo,
    '/camera/depth_registered/points': PointCloud2,
}
_COLOR = '/camera/color/image_raw'
_DEPTH = '/camera/depth/image_raw'
_POINTS = '/camera/depth_registered/points'
_INFO_IMAGES = {
    '/camera/color/camera_info': _COLOR,
    '/camera/depth/camera_info': _DEPTH,
}
_SYNC_SLOP_NS = 50_000_000  # Same strict <50 ms span as the Jazzy RGB-D intake.


def _stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def _open_reader(uri: str, topics: list[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def _validate_calibration(frame: dict) -> None:
    """Require the aligned pixel geometry assumed by this frozen fixture."""
    color, depth, cloud = (frame[n] for n in (_COLOR, _DEPTH, _POINTS))
    if ((color.width, color.height) != (depth.width, depth.height)
            or color.width <= 0 or color.height <= 0):
        raise RuntimeError('calibration: RGB/depth image dimensions do not match')
    for topic, image_topic in _INFO_IMAGES.items():
        info, image = frame[topic], frame[image_topic]
        if ((info.width, info.height) != (image.width, image.height)
                or not info.header.frame_id
                or info.header.frame_id != image.header.frame_id):
            raise RuntimeError(f'calibration: {topic} size/frame differs from image')
        if (not all(math.isfinite(v) for v in info.k)
                or info.k[0] <= 0.0 or info.k[4] <= 0.0):
            raise RuntimeError(f'calibration: {topic} has invalid intrinsic K')
    color_info = frame['/camera/color/camera_info']
    depth_info = frame['/camera/depth/camera_info']
    if not all(math.isclose(a, b, rel_tol=0.0, abs_tol=1e-6)
               for a, b in zip(color_info.k, depth_info.k)):
        raise RuntimeError('calibration: depth must be registered to color intrinsic K')
    if (not cloud.header.frame_id
            or cloud.header.frame_id != depth.header.frame_id
            or cloud.width * cloud.height <= 0):
        raise RuntimeError('calibration: registered cloud is empty or in another frame')


def discover_bag() -> str | None:
    """Return the default frozen-camera bag directory, or None if absent."""
    env = os.environ.get('PEACH_FROZEN_BAG', '').strip()
    if env:
        return env if (Path(env) / 'metadata.yaml').is_file() else None
    here = Path(__file__).resolve()
    for base in (Path.cwd(), *Path.cwd().parents, *here.parents):
        cand = base / BAG_RELATIVE
        if (cand / 'metadata.yaml').is_file():
            return str(cand)
    return None


def load_frame(uri: str, index: int) -> dict:
    """Bind RGB index, then use original header stamps with bounded Jazzy ATS.

    CameraInfo is calibration, selected independently (latest before RGB, or
    earliest after if no earlier sample exists). It is not exposure evidence.
    Two streaming passes retain one RGB and at most ten depth/cloud candidates,
    rather than accumulating full-resolution images from the entire bag.
    """
    if index < 0:
        raise RuntimeError('RGB index must be non-negative')
    color = None
    reader = _open_reader(uri, [_COLOR])
    try:
        for _ in range(index + 1):
            if not reader.has_next():
                raise RuntimeError(f'bag {uri} missing RGB frame index {index}')
            _name, data, _record_stamp = reader.read_next()
        color = deserialize_message(data, Image)
    finally:
        reader.close()
    anchor_ns = _stamp_ns(color)
    if anchor_ns <= 0:
        raise RuntimeError('RGB exposure stamp must be positive')

    filters = [SimpleFilter() for _ in range(3)]
    sync = ApproximateTimeSynchronizer(filters, queue_size=10, slop=0.05)
    chosen = {}

    def on_synced(rgb, depth, cloud):
        chosen.update({_COLOR: rgb, _DEPTH: depth, _POINTS: cloud})

    sync.registerCallback(on_synced)
    filters[0].signalMessage(color)  # Only the requested RGB ever enters ATS.
    nearest = {_DEPTH: None, _POINTS: None}
    infos = {}
    reader = _open_reader(uri, [_DEPTH, _POINTS, *_INFO_IMAGES])
    try:
        while reader.has_next():
            name, data, _record_stamp = reader.read_next()
            if chosen and name not in _INFO_IMAGES:
                continue
            msg = deserialize_message(data, TOPICS[name])
            stamp_ns = _stamp_ns(msg)
            if name in _INFO_IMAGES:
                rank = (stamp_ns > anchor_ns, abs(stamp_ns - anchor_ns))
                if name not in infos or rank < infos[name][0]:
                    infos[name] = (rank, msg)
                continue
            skew_ns = stamp_ns - anchor_ns
            if nearest[name] is None or abs(skew_ns) < abs(nearest[name]):
                nearest[name] = skew_ns
            if abs(skew_ns) < _SYNC_SLOP_NS:
                filters[1 if name == _DEPTH else 2].signalMessage(msg)
    finally:
        reader.close()
    if not chosen:
        nearest_ms = {name: None if skew is None else skew / 1e6
                      for name, skew in nearest.items()}
        raise RuntimeError(
            f'bag {uri} has no synchronized RGB-D/cloud group for RGB index '
            f'{index}, original RGB stamp_ns={anchor_ns}; span must be <50 ms; '
            f'nearest original skews_ms={nearest_ms}')
    missing = [name for name in _INFO_IMAGES if name not in infos]
    if missing:
        raise RuntimeError(f'calibration: bag {uri} missing {missing}')
    chosen.update({name: entry[1] for name, entry in infos.items()})
    _validate_calibration(chosen)
    original_stamps = {name: _stamp_ns(msg) for name, msg in chosen.items()}
    original_skews_ms = {name: (stamp - anchor_ns) / 1e6
                         for name, stamp in original_stamps.items()}
    print(
        f'loaded RGB index {index} from {uri}: '
        f'original stamps_ns={original_stamps}; original skews_ms='
        f'{original_skews_ms}; CameraInfo is static calibration, not an exposure',
        flush=True)
    return chosen


def retimestamp_frame(frame: dict, stamp) -> dict:
    """Shift a validated sensor group to live time, preserving exposure offsets."""
    anchor_ns = _stamp_ns(frame[_COLOR])
    live_ns = stamp.sec * 10**9 + stamp.nanosec
    emitted = {}
    for name, proto in frame.items():
        msg = copy.deepcopy(proto)
        # Calibration accompanies its selected image at that image's new stamp.
        source = frame[_INFO_IMAGES.get(name, name)]
        shifted_ns = live_ns + _stamp_ns(source) - anchor_ns
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(shifted_ns, 10**9)
        emitted[name] = msg
    return emitted


def optical_static_tfs(stamp) -> list[TransformStamped]:
    """camera_link → {color,depth}_frame → optical; same chain as peach_stereo."""
    msgs = []
    pairs = (
        ('camera_link', 'camera_color_frame', (0.0, 0.0, 0.0, 1.0)),
        ('camera_color_frame', 'camera_color_optical_frame', _OPTICAL_Q),
        ('camera_link', 'camera_depth_frame', (0.0, 0.0, 0.0, 1.0)),
        ('camera_depth_frame', 'camera_depth_optical_frame', _OPTICAL_Q),
    )
    for parent, child, rot in pairs:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.rotation.x, tf.transform.rotation.y = rot[0], rot[1]
        tf.transform.rotation.z, tf.transform.rotation.w = rot[2], rot[3]
        msgs.append(tf)
    return msgs


class FrozenCamera(Node):
    """Timer publisher of one bag frame with rewritten stamps."""

    def __init__(self, frame: dict, hz: float, start_paused: bool = False):
        super().__init__('frozen_camera_replay')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE)
        self._frame = frame
        self._paused = start_paused
        self._pubs = {
            name: self.create_publisher(type(msg), name, qos)
            for name, msg in frame.items()}
        self.create_service(
            SetBool, '~/pause', self._on_pause,
            callback_group=ReentrantCallbackGroup())
        self._static_tf = StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(
            optical_static_tfs(self.get_clock().now().to_msg()))
        period = 1.0 / max(hz, 0.1)
        self.create_timer(period, self._tick)

    def _on_pause(self, request, response):
        self._paused = bool(request.data)
        response.success = True
        response.message = 'paused' if self._paused else 'streaming'
        return response

    def _tick(self) -> None:
        if self._paused:
            return
        stamp = self.get_clock().now().to_msg()
        for name, msg in retimestamp_frame(self._frame, stamp).items():
            self._pubs[name].publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--bag', default='',
        help='ros2 bag directory (mcap); empty = PEACH_FROZEN_BAG or workspace capture')
    parser.add_argument('--index', type=int, default=70,
                        help='0-based RGB frame index; requires original synchronized depth/cloud')
    parser.add_argument('--hz', type=float, default=2.0)
    parser.add_argument(
        '--start-paused', default='false',
        help='true = publish nothing until ~/pause is set false')
    args, _unknown = parser.parse_known_args()
    uri = args.bag.strip() or discover_bag()
    if not uri:
        print('frozen camera bag not found; set --bag or PEACH_FROZEN_BAG',
              file=sys.stderr)
        return 2
    try:
        frame = load_frame(uri, args.index)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    rclpy.init()
    node = FrozenCamera(
        frame, args.hz, start_paused=args.start_paused.strip().lower() in
        ('1', 'true', 'yes', 'on'))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
