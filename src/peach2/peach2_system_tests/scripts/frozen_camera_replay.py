#!/usr/bin/env python3
"""
Republish one RGB-D frame from a camera bag with live timestamps.

The recorded image is fixed (photo-pose simulation). Stamps are rewritten to
now() so perception looks up the current wrist TF. Does not start a camera
driver. Topics match peach_stereo / percipio.
"""
from __future__ import annotations

import argparse
import copy
import os
from pathlib import Path
import sys

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
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
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    counts = {name: 0 for name in TOPICS}
    chosen = {}
    while reader.has_next():
        name, data, _stamp = reader.read_next()
        if name not in TOPICS:
            continue
        if counts[name] == index:
            chosen[name] = deserialize_message(data, TOPICS[name])
        counts[name] += 1
        if len(chosen) == len(TOPICS):
            break
    missing = [n for n in TOPICS if n not in chosen]
    if missing:
        raise RuntimeError(
            f'bag {uri} missing frame {index} for {missing}; counts={counts}')
    print(
        f'loaded frame {index} from {uri}: ' +
        ', '.join(f'{n}={counts[n]}' for n in TOPICS),
        flush=True)
    return chosen


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
            return  # freeze the world: no frames, no TF refresh, no fusion drift
        stamp = self.get_clock().now().to_msg()
        for name, proto in self._frame.items():
            msg = copy.deepcopy(proto)
            msg.header.stamp = stamp
            self._pubs[name].publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--bag', default='',
        help='ros2 bag directory (mcap); empty = PEACH_FROZEN_BAG or workspace capture')
    parser.add_argument('--index', type=int, default=70,
                        help='0-based frame index to freeze')
    parser.add_argument('--hz', type=float, default=2.0)
    parser.add_argument(
        '--start-paused', default='false',
        help='true = publish nothing until ~/pause is set false (blind photo '
             'pose: no frames, no fusion, no stale tracks while the arm moves)')
    args, _unknown = parser.parse_known_args()
    uri = args.bag.strip() or discover_bag()
    if not uri:
        print('frozen camera bag not found; set --bag or PEACH_FROZEN_BAG',
              file=sys.stderr)
        return 2
    frame = load_frame(uri, args.index)
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
