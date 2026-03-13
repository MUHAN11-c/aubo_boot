/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_MOVEIT_ARCLINE_DEMO_H_
#define DEMO_DRIVER_MOVEIT_ARCLINE_DEMO_H_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <aubo_msgs/srv/set_io.hpp>
#include <demo_interface/srv/move_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace demo_driver
{

struct CartesianSegment
{
  char axis;
  double offset;
};

class MoveitArclineDemo : public rclcpp::Node
{
public:
  explicit MoveitArclineDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MoveitArclineDemo() override = default;

  static std::shared_ptr<MoveitArclineDemo> create(const rclcpp::NodeOptions& options);

  bool waitForServices(std::chrono::seconds timeout);
  virtual bool run();

protected:
  void initMoveGroup();

  bool moveToJoints(const std::array<double, 6>& joint_positions_rad, float velocity_factor = 0.5f,
                    float acceleration_factor = 0.5f);
  bool moveToPose(double x, double y, double z, double qx, double qy, double qz, double qw, bool use_joints = false,
                  float velocity_factor = 0.5f, float acceleration_factor = 0.5f);

  bool moveToHome();
  bool moveToArcStart();
  bool runArcPath(double z_offset = 0.2);
  bool runArcPath(char axis, double offset);
  bool runArcPathSequence(const std::vector<CartesianSegment>& segments);
  bool setDigitalOutput(int32_t io_index, bool high);

private:
  rclcpp::Client<demo_interface::srv::MoveToPose>::SharedPtr move_to_pose_client_;
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr aubo_set_io_client_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  static const std::array<double, 6> kHomeJointsRad1;
  static const std::string kMoveToPoseService;
  static const std::string kAuboSetIOService;
  static constexpr int kCallTimeoutSeconds = 60;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_MOVEIT_ARCLINE_DEMO_H_
