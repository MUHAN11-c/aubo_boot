#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Deterministic ROS examples for the web client; never connects to hardware."""

import math
from pathlib import Path
import struct
import time
import zlib

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as RosPath
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import CompressedImage, Image, JointState, PointCloud2, PointField
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


def png_image(tick):
    """Build a tiny changing PNG with the standard library, without numpy/ABI."""
    width, height = 96, 48
    rows = b''.join(b'\x00' + bytes(
        channel for x in range(width)
        for channel in ((x * 2 + tick * 7) % 256, y * 5, 110))
        for y in range(height))

    def chunk(tag, data):
        return struct.pack('!I', len(data)) + tag + data + struct.pack(
            '!I', zlib.crc32(tag + data) & 0xffffffff)

    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack(
        '!2I5B', width, height, 8, 2, 0, 0, 0)) + chunk(
        b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b'')


def depth_millimeters(x, y, tick):
    """Return explicit uint16 millimeters for the synthetic optical surface."""
    return round(850 + 80 * math.sin((x + tick) / 16) + 60 * math.cos(y / 10))


class Demo(Node):
    """Exercise real DDS QoS, standard services, typed parameters and actions."""

    def __init__(self):
        """Keep every endpoint under a dedicated demonstration namespace."""
        super().__init__('demo', namespace='/web_link_demo')
        self.group = ReentrantCallbackGroup()
        self.declare_parameter('gain', 1.0, ParameterDescriptor(
            description='Demo gain, allowed range 0..10; rejection is server-side.'))
        for key, value in [('enabled', False), ('count', 3), ('label', 'demo'),
                           ('bools', [True, False]), ('ints', [1, 2]),
                           ('doubles', [0.1, 0.2]), ('strings', ['a', 'b']),
                           ('bytes', [b'\x00', b'\xff'])]:
            self.declare_parameter(key, value)
        self.declare_parameter('read_only', 'fixed', ParameterDescriptor(read_only=True))
        self.add_on_set_parameters_callback(self.validate)
        self.echo = self.create_publisher(String, 'echo', 10)
        self.create_subscription(String, 'input', self.echo.publish, 10)
        self.ticks = self.create_publisher(String, 'ticks', 10)
        self.sensor = self.create_publisher(Float64, 'sensor', qos_profile_sensor_data)
        retained_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.retained = self.create_publisher(String, 'retained', retained_qos)
        self.retained.publish(String(data='retained-once'))
        self.description = self.create_publisher(String, 'robot_description', retained_qos)
        self.tf_static = self.create_publisher(TFMessage, 'tf_static', retained_qos)
        self.diagnostics = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.image = self.create_publisher(CompressedImage, 'image/compressed',
                                           qos_profile_sensor_data)
        self.joints = self.create_publisher(JointState, 'joint_states', 10)
        self.tf = self.create_publisher(TFMessage, 'tf', 10)
        self.cloud = self.create_publisher(PointCloud2, 'points', qos_profile_sensor_data)
        self.depth = self.create_publisher(Image, 'depth', qos_profile_sensor_data)
        self.markers = self.create_publisher(MarkerArray, 'markers', 10)
        self.path = self.create_publisher(RosPath, 'path', 10)
        self.publish_static()
        self.create_service(AddTwoInts, 'add_two_ints', self.add,
                            callback_group=self.group)
        self.create_service(Trigger, 'slow', self.slow, callback_group=self.group)
        self.action = ActionServer(self, Fibonacci, 'fibonacci',
                                   execute_callback=self.execute,
                                   goal_callback=self.goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.tick = 0
        self.create_timer(0.2, self.publish_samples)

    def publish_static(self):
        """Retain the URDF and both fixed edges on dedicated demo topics."""
        description = Path(get_package_share_directory('ros2_web_link')) / 'description/demo.urdf'
        self.description.publish(String(data=description.read_text(encoding='utf-8')))
        stamp = self.get_clock().now().to_msg()
        world = TransformStamped()
        world.header.stamp, world.header.frame_id = stamp, 'demo_world'
        world.child_frame_id = 'demo_base'
        world.transform.rotation.w = 1.0
        camera = TransformStamped()
        camera.header.stamp, camera.header.frame_id = stamp, 'demo_base'
        camera.child_frame_id = 'demo_camera_optical'
        camera.transform.translation.x = -0.2
        camera.transform.translation.z = 0.5
        # URDF rpy=(-pi/2, 0, -pi/2): optical +z forward, +x right, +y down.
        camera.transform.rotation.x = -0.5
        camera.transform.rotation.y = 0.5
        camera.transform.rotation.z = -0.5
        camera.transform.rotation.w = 0.5
        self.tf_static.publish(TFMessage(transforms=[world, camera]))

    @staticmethod
    def validate(parameters):
        """Reject invalid gain atomically in the node, not just the web form."""
        for parameter in parameters:
            if parameter.name == 'gain' and not 0 <= parameter.value <= 10:
                return SetParametersResult(successful=False, reason='gain must be 0..10')
        return SetParametersResult(successful=True)

    @staticmethod
    def add(request, response):
        """Complete a short standard service."""
        response.sum = request.a + request.b
        return response

    @staticmethod
    def slow(request, response):
        """Demonstrate that a local timeout does not cancel ROS service work."""
        time.sleep(1.2)
        response.success = True
        response.message = 'Completed remotely even if browser stopped waiting'
        return response

    @staticmethod
    def goal(request):
        """Reject unreasonable orders; 13 intentionally demonstrates abort."""
        return GoalResponse.ACCEPT if 1 <= request.order <= 30 else GoalResponse.REJECT

    def execute(self, handle):
        """Publish feedback and cooperate with cancellation via another thread."""
        result = Fibonacci.Result(sequence=[0, 1])
        for _ in range(2, handle.request.order + 1):
            if not rclpy.ok():
                return result
            if handle.is_cancel_requested:
                handle.canceled()
                return result
            if handle.request.order == 13 and len(result.sequence) == 4:
                handle.abort()
                return result
            result.sequence.append(result.sequence[-1] + result.sequence[-2])
            handle.publish_feedback(Fibonacci.Feedback(sequence=result.sequence))
            time.sleep(0.2)
        handle.succeed()
        return result

    def publish_samples(self):
        """Provide low-bandwidth samples including binary fields and timestamps."""
        self.tick += 1
        stamp = self.get_clock().now().to_msg()
        self.ticks.publish(String(data=str(self.tick)))
        self.sensor.publish(Float64(data=math.sin(self.tick / 5)))
        joint = JointState()
        joint.header.stamp = stamp
        rotation = 0.5 * math.sin(self.tick / 5)
        slide = 0.1 + 0.06 * math.sin(self.tick / 7)
        joint.name, joint.position = ['demo_joint', 'demo_slide'], [rotation, slide]
        self.joints.publish(joint)
        arm = TransformStamped()
        arm.header.stamp, arm.header.frame_id = stamp, 'demo_base'
        arm.child_frame_id = 'demo_arm'
        arm.transform.translation.z = 0.13
        arm.transform.rotation.z = math.sin(rotation / 2)
        arm.transform.rotation.w = math.cos(rotation / 2)
        tool = TransformStamped()
        tool.header.stamp, tool.header.frame_id = stamp, 'demo_arm'
        tool.child_frame_id = 'demo_tool'
        tool.transform.translation.x = 0.6
        tool.transform.translation.z = slide
        tool.transform.rotation.w = 1.0
        self.tf.publish(TFMessage(transforms=[arm, tool]))
        if self.tick % 5:
            return
        image = CompressedImage(format='png', data=png_image(self.tick))
        image.header.stamp, image.header.frame_id = stamp, 'demo_camera_optical'
        self.image.publish(image)
        depth = Image(height=48, width=96, encoding='16UC1', is_bigendian=0, step=192)
        depth.header.stamp, depth.header.frame_id = stamp, 'demo_camera_optical'
        depth.data = struct.pack('<4608H', *(
            depth_millimeters(x, y, self.tick) for y in range(48) for x in range(96)))
        self.depth.publish(depth)
        diagnostic = DiagnosticArray(status=[DiagnosticStatus(
            level=DiagnosticStatus.OK, name='web_link_demo', message='No hardware',
            hardware_id='simulated', values=[KeyValue(key='tick', value=str(self.tick))])])
        diagnostic.header.stamp = stamp
        self.diagnostics.publish(diagnostic)
        cloud = PointCloud2(height=20, width=20, is_bigendian=False,
                            point_step=16, row_step=320, is_dense=True)
        cloud.header.stamp, cloud.header.frame_id = stamp, 'demo_camera_optical'
        cloud.fields = [PointField(name=axis, offset=i * 4,
                                   datatype=PointField.FLOAT32, count=1)
                        for i, axis in enumerate(('x', 'y', 'z'))]
        # PCL-compatible rgb: FLOAT32 field whose raw bits hold 0x00RRGGBB.
        cloud.fields.append(PointField(name='rgb', offset=12,
                                       datatype=PointField.FLOAT32, count=1))
        points = []
        for row in range(20):
            for column in range(20):
                u, v = round(column * 95 / 19), round(row * 47 / 19)
                z = depth_millimeters(u, v, self.tick) * 0.001
                x, y = (u - 47.5) * z / 120, (v - 23.5) * z / 120
                rgb = (column * 255 // 19 << 16) | (row * 255 // 19 << 8) | 160
                points.append(struct.pack('<fffI', x, y, z, rgb))
        cloud.data = b''.join(points)
        self.cloud.publish(cloud)
        self.publish_overlays(stamp)

    def publish_overlays(self, stamp):
        """Exercise marker resources/frame locking and a 20-pose standard path."""
        sphere = Marker(ns='demo_observations', id=0, type=Marker.SPHERE, action=Marker.ADD)
        sphere.header.stamp, sphere.header.frame_id = stamp, 'demo_base'
        sphere.pose.orientation.w = 1.0
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = 0.55, 0.3, 0.25
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.14
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.45, 0.1, 1.0
        arrow = Marker(ns='demo_observations', id=1, type=Marker.ARROW,
                       action=Marker.ADD, frame_locked=True)
        arrow.header.stamp, arrow.header.frame_id = stamp, 'demo_arm'
        arrow.pose.orientation.w = 1.0
        arrow.pose.position.z = 0.12
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.35, 0.025, 0.045
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.15, 0.8, 0.55, 1.0
        mesh = Marker(ns='demo_observations', id=2, type=Marker.MESH_RESOURCE,
                      action=Marker.ADD,
                      mesh_resource='package://ros2_web_link/description/tool.stl')
        mesh.header.stamp, mesh.header.frame_id = stamp, 'demo_base'
        mesh.pose.orientation.w = 1.0
        mesh.pose.position.x, mesh.pose.position.y, mesh.pose.position.z = 0.7, -0.28, 0.08
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
        mesh.color.r, mesh.color.g, mesh.color.b, mesh.color.a = 0.2, 0.55, 0.95, 1.0
        self.markers.publish(MarkerArray(markers=[sphere, arrow, mesh]))
        path = RosPath()
        path.header.stamp, path.header.frame_id = stamp, 'demo_base'
        for index in range(20):
            angle = 2 * math.pi * index / 19
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = stamp, 'demo_base'
            pose.pose.position.x = 0.35 + 0.32 * math.cos(angle)
            pose.pose.position.y = 0.35 * math.sin(angle)
            pose.pose.position.z = 0.3 + 0.04 * math.sin(2 * angle)
            pose.pose.orientation.z = math.sin((angle + math.pi / 2) / 2)
            pose.pose.orientation.w = math.cos((angle + math.pi / 2) / 2)
            path.poses.append(pose)
        self.path.publish(path)


def main():
    """Run demo and native lifecycle node; launch SIGINT shuts down both."""
    rclpy.init()
    demo = Demo()
    lifecycle = LifecycleNode('lifecycle', namespace='/web_link_demo')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(demo)
    executor.add_node(lifecycle)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        demo.destroy_node()
        lifecycle.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
