# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Single-authority wrist-to-camera-root static TF publisher."""

from pathlib import Path

from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import yaml

from .storage import default_storage_directory


class ExtrinsicsPublisher(Node):
    def __init__(self):
        super().__init__('hand_eye_extrinsics_publisher')
        self.declare_parameter(
            'parent_frame', 'wrist3_Link',
            ParameterDescriptor(description='静态 TF 父坐标系 (腕部)'))
        self.declare_parameter(
            'child_frame', 'camera_link',
            ParameterDescriptor(description='静态 TF 子坐标系 (相机安装座)'))
        self.declare_parameter(
            'active_file', '',
            ParameterDescriptor(
                description='激活标定结果文件路径, 空串时按默认存储目录定位'))
        self.declare_parameter(
            'nominal_xyz_m', [0.0, 0.0, 0.020],
            ParameterDescriptor(
                description='无激活结果时发布的标称外参平移 (m)'))
        self.declare_parameter(
            'nominal_quaternion_xyzw', [0.0, 0.0, 0.0, 1.0],
            ParameterDescriptor(
                description='无激活结果时发布的标称外参四元数 (xyzw)'))
        # Own /tf_static publisher: tf2_ros StaticTransformBroadcaster will not
        # replace a child_frame_id it already sent, so ~/reload would keep the
        # first (often nominal) pose latched.
        self._tf_pub = self.create_publisher(
            TFMessage,
            '/tf_static',
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self._reload_service = self.create_service(
            Trigger, '~/reload', self._reload)
        try:
            self._publish()
        except (OSError, KeyError, TypeError, ValueError,
                yaml.YAMLError) as error:
            # active.yaml 损坏或 frame 不匹配不应导致节点启动崩溃,
            # 回退到 nominal 外参并告警
            self.get_logger().error(
                f'failed to load active calibration ({error}); '
                'falling back to nominal extrinsic')
            self._publish_nominal()

    def _publish_nominal(self):
        xyz = self.get_parameter('nominal_xyz_m').value
        quaternion = self.get_parameter('nominal_quaternion_xyzw').value
        self._send(xyz, quaternion)
        self.get_logger().warning(
            'No active hand-eye result; publishing nominal camera extrinsic')

    def _active_path(self):
        configured = self.get_parameter('active_file').value
        return Path(configured) if configured else (
            default_storage_directory() / 'active.yaml')

    def _load_transform(self):
        active_path = self._active_path()
        if active_path.is_file():
            with active_path.open(encoding='utf-8') as stream:
                data = yaml.safe_load(stream)
            frames = data['frames']
            if (
                frames['wrist'] != self.get_parameter('parent_frame').value
                or frames['camera_root']
                != self.get_parameter('child_frame').value
            ):
                raise ValueError('active calibration frame ids do not match')
            transform = data['transforms']['wrist_from_camera_root']
            return transform['xyz_m'], transform['quaternion_xyzw'], False
        return (
            self.get_parameter('nominal_xyz_m').value,
            self.get_parameter('nominal_quaternion_xyzw').value,
            True,
        )

    def _send(self, xyz, quaternion):
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('parent_frame').value
        message.child_frame_id = self.get_parameter('child_frame').value
        message.transform.translation.x = float(xyz[0])
        message.transform.translation.y = float(xyz[1])
        message.transform.translation.z = float(xyz[2])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])
        self._tf_pub.publish(TFMessage(transforms=[message]))

    def _publish(self):
        xyz, quaternion, nominal = self._load_transform()
        self._send(xyz, quaternion)
        if nominal:
            self.get_logger().warning(
                'No active hand-eye result; publishing nominal camera extrinsic')
        else:
            self.get_logger().info(
                f'Published active camera extrinsic from {self._active_path()}')

    def _reload(self, _request, response):
        try:
            self._publish()
            response.success = True
            response.message = 'extrinsic reloaded'
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            response.success = False
            response.message = str(error)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
