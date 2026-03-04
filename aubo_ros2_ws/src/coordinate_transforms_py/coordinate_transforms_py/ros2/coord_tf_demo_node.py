#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from visualization_msgs.msg import Marker
from coordinate_transforms_py.ros2 import rotation_conversion_ros2 as rcr

def main(args=None):
    rclpy.init(args=args)
    node = Node('coord_tf_demo_node')
    tf_broadcaster = TransformBroadcaster(node)
    marker_pub = node.create_publisher(Marker, 'coord_tf_marker', 10)

    t = TransformStamped()
    t.header.frame_id = 'world'
    t.child_frame_id = 'camera_optical_frame'
    t.transform.translation.x = 0.5
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.3
    t.transform.rotation = rcr.rpy_to_ros_quaternion(0.0, 0.0, 0.0)

    def timer_cb():
        now = node.get_clock().now().to_msg()
        t.header.stamp = now
        tf_broadcaster.sendTransform(t)
        # Publish 3 ARROW markers for X/Y/Z axes (ROS2 Marker has no AXIS type)
        ax = 0.2
        orients = [
            Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),   # X
            Quaternion(x=0.0, y=0.0, z=0.7071, w=0.7071),   # Y (90 about Z)
            Quaternion(x=0.0, y=-0.7071, z=0.0, w=0.7071),   # Z (-90 about Y)
        ]
        for i in range(3):
            m = Marker()
            m.header.frame_id = 'camera_optical_frame'
            m.header.stamp = now
            m.ns = 'coord_tf'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = 0.0
            m.pose.position.y = 0.0
            m.pose.position.z = 0.0
            m.pose.orientation = orients[i]
            m.scale.x = ax
            m.scale.y = 0.02
            m.scale.z = 0.02
            m.color.r = 1.0 if i == 0 else 0.0
            m.color.g = 1.0 if i == 1 else 0.0
            m.color.b = 1.0 if i == 2 else 0.0
            m.color.a = 1.0
            marker_pub.publish(m)

    timer = node.create_timer(0.1, timer_cb)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
