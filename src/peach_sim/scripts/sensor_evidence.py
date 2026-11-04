"""Record synchronized simulator sensor and contact evidence inside a run directory."""

import json
import time

from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
from peach_interfaces.msg import PeachTargetObservationArray
import numpy as np
from PIL import Image as PillowImage
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(header):
    """Return the acquisition time without substituting arrival time."""
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


class SensorEvidence:
    """Observe data only; never command the robot or alter success criteria."""

    def __init__(self, node, run):
        self.run = run
        self.directory = run / 'sensors'
        self.directory.mkdir()
        self.bridge = CvBridge()
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.counts = {'synchronized_frames': 0, 'snapshots': 0, 'contacts': 0}
        self.streams = {}
        self.observation_count = 0
        node.create_subscription(PeachTargetObservationArray,
                                 '/peach/perception/target_observations',
                                 self.on_observations, 10)
        self.pending, self.last_saved, self.stage = None, 0., 'startup'
        self.subscribers = [Subscriber(node, kind, topic, qos_profile=qos_profile_sensor_data)
                            for kind, topic in (
                                (Image, '/camera/color/image_raw'),
                                (Image, '/camera/depth/image_raw'),
                                (CameraInfo, '/camera/color/camera_info'),
                                (PointCloud2, '/camera/depth_registered/points'))]
        self.sync = TimeSynchronizer(self.subscribers, 8)
        self.sync.registerCallback(self.on_frame)
        for subscriber in self.subscribers:
            subscriber.registerCallback(self.on_raw, subscriber.topic)
        node.create_subscription(TFMessage, '/sim_harvest/entity_poses', self.on_poses, 100)
        node.create_subscription(Contacts, '/sim_harvest/basket_contacts', self.on_contacts, 100)
        node.create_timer(.2, self.save_pending)

    def on_observations(self, msg):
        """Record perception progress even when no target satisfies the model gate."""
        self.observation_count += 1
        payload = {'messages': self.observation_count, 'epoch': msg.scene_epoch,
                   'locked': msg.target_set_locked, 'run': msg.harvest_run_id,
                   'collecting': msg.collecting_count, 'pending': msg.pending_count,
                   'targets': [{'id': item.target_id, 'tracking': item.tracking_status,
                                'status': item.candidate.status,
                                'flags': list(item.diagnostic_flags)
                                + list(item.candidate.diagnostic_flags)}
                               for item in msg.observations]}
        (self.run / 'perception_progress.json').write_text(json.dumps(payload))

    def on_frame(self, *messages):
        self.counts['synchronized_frames'] += 1
        self.pending = messages

    def on_raw(self, msg, topic):
        previous = self.streams.get(topic, {})
        self.streams[topic] = {'count': previous.get('count', 0) + 1,
                               'stamp_ns': stamp_ns(msg.header), 'frame': msg.header.frame_id}

    def on_poses(self, msg):
        """Keep actual Gazebo poses with simulation timestamps for dynamics analysis."""
        rows = []
        for item in msg.transforms:
            if item.child_frame_id != 'harvester' and not item.child_frame_id.endswith('_picked'):
                continue
            t, q = item.transform.translation, item.transform.rotation
            rows.append({'name': item.child_frame_id, 'stamp_ns': stamp_ns(item.header),
                         'xyz': [t.x, t.y, t.z], 'xyzw': [q.x, q.y, q.z, q.w]})
        if rows:
            with (self.run / 'entity_poses.jsonl').open('a') as output:
                output.write(json.dumps(rows) + '\n')

    def on_contacts(self, msg):
        """Retain native contact identities, penetration and solver forces."""
        self.counts['contacts'] += len(msg.contacts)
        rows = [{'stamp_ns': stamp_ns(msg.header),
                 'collision1': c.collision1.name, 'collision2': c.collision2.name,
                 'depths_m': list(c.depths),
                 'positions': [[p.x, p.y, p.z] for p in c.positions],
                 'forces': [[w.body_1_wrench.force.x, w.body_1_wrench.force.y,
                             w.body_1_wrench.force.z] for w in c.wrenches]}
                for c in msg.contacts]
        if rows:
            with (self.run / 'contacts.jsonl').open('a') as output:
                output.write(json.dumps(rows) + '\n')

    def save_pending(self):
        """Save bounded samples; every TF lookup uses the exact exposure stamp."""
        (self.directory / 'streams.json').write_text(json.dumps(self.streams))
        if self.pending is None or time.monotonic() - self.last_saved < 4:
            return
        rgb, depth, info, cloud = self.pending
        try:
            transform = self.buffer.lookup_transform(
                'platform_base_link', rgb.header.frame_id, Time.from_msg(rgb.header.stamp))
        except TransformException:
            return
        try:
            color = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
            distance = self.bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
            points = read_points_numpy(cloud, field_names=['x', 'y', 'z']).reshape(
                depth.height, depth.width, 3)
            cloud_transform = self.buffer.lookup_transform(
                rgb.header.frame_id, cloud.header.frame_id, Time.from_msg(cloud.header.stamp))
            cq, ct = cloud_transform.transform.rotation, cloud_transform.transform.translation
            with np.errstate(invalid='ignore'):
                optical = Rotation.from_quat([cq.x, cq.y, cq.z, cq.w]).apply(
                    points.reshape(-1, 3)).reshape(points.shape) + [ct.x, ct.y, ct.z]
            valid = np.isfinite(distance) & (distance > 0)
            compare = valid & np.isfinite(optical).all(axis=2)
            number = self.counts['snapshots']
            prefix = f'{number:03d}_{self.stage}'
            PillowImage.fromarray(color).save(self.directory / f'{prefix}_rgb.png')
            np.save(self.directory / f'{prefix}_depth.npy', distance)
            np.savez_compressed(self.directory / f'{prefix}_cloud.npz', xyz=points)
            display = np.zeros((*distance.shape, 3), dtype=np.uint8)
            display[valid, 1] = np.clip(distance[valid] * 255, 0, 255).astype(np.uint8)
            PillowImage.fromarray(display).save(self.directory / f'{prefix}_depth.png')
            t, q = transform.transform.translation, transform.transform.rotation
            record = {
                'prefix': prefix, 'stage': self.stage, 'stamp_ns': stamp_ns(rgb.header),
                'frames': [m.header.frame_id for m in (rgb, depth, info, cloud)],
                'encodings': [rgb.encoding, depth.encoding], 'dimensions': [rgb.width, rgb.height],
                'K': list(info.k), 'valid_depth_fraction': float(valid.mean()),
                'depth_min_m': float(distance[valid].min()) if valid.any() else None,
                'depth_max_m': float(distance[valid].max()) if valid.any() else None,
                'cloud_z_depth_max_error_m': float(np.max(np.abs(
                    optical[..., 2][compare] - distance[compare]))) if compare.any() else None,
                'base_from_optical': {'xyz': [t.x, t.y, t.z], 'xyzw': [q.x, q.y, q.z, q.w]},
            }
            with (self.directory / 'frames.jsonl').open('a') as output:
                output.write(json.dumps(record) + '\n')
            self.counts['snapshots'] += 1
        except Exception as exc:
            with (self.directory / 'errors.log').open('a') as output:
                output.write(str(exc) + '\n')
        self.pending = None
        self.last_saved = time.monotonic()
