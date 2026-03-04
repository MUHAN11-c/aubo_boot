#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "coordinate_transforms/ros2/rotation_conversion_ros2.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("coord_tf_demo_node");

  tf2_ros::TransformBroadcaster tf_broadcaster(node);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("coord_tf_marker", 10);

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = "camera_optical_frame";
  t.transform.translation.x = 0.5;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.3;
  t.transform.rotation = coordinate_transforms::ros2::rpy_to_ros_quaternion(0.0, 0.0, 0.0);

  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    auto now = node->get_clock()->now();
    t.header.stamp = now;
    tf_broadcaster.sendTransform(t);

    // Publish 3 ARROW markers for X/Y/Z axes (ROS2 Marker has no AXIS type)
    const double ax = 0.2;
    for (int i = 0; i < 3; i++) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "camera_optical_frame";
      m.header.stamp = now;
      m.ns = "coord_tf";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = 0.0;
      m.pose.position.y = 0.0;
      m.pose.position.z = 0.0;
      if (i == 0) { m.pose.orientation.w = 1.0; m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0.0; }  // X
      else if (i == 1) { m.pose.orientation.w = 0.7071; m.pose.orientation.z = 0.7071; m.pose.orientation.x = m.pose.orientation.y = 0.0; }  // Y (90 about Z)
      else { m.pose.orientation.w = 0.7071; m.pose.orientation.y = -0.7071; m.pose.orientation.x = m.pose.orientation.z = 0.0; }  // Z (-90 about Y)
      m.scale.x = ax;
      m.scale.y = 0.02;
      m.scale.z = 0.02;
      m.color.r = (i == 0) ? 1.0f : 0.0f;
      m.color.g = (i == 1) ? 1.0f : 0.0f;
      m.color.b = (i == 2) ? 1.0f : 0.0f;
      m.color.a = 1.0f;
      marker_pub->publish(m);
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
