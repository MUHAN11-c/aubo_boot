/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_MOVEL_SERVER_H_
#define DEMO_DRIVER_MOVEL_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/srv/movel.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cstdint>
#include <string>
#include <memory>
#include <mutex>

namespace demo_driver
{

/** Movel 服务错误码（应用层）；MoveIt 返回的错误码直接透传为 int32_t */
enum class MovelErrorCode : int32_t
{
  SUCCESS = 0,
  MOVEIT_NOT_INITIALIZED = -100,
  GET_CURRENT_STATE_FAILED = -101,
  JOINT_MODEL_GROUP_FAILED = -102,
  END_EFFECTOR_LINK_EMPTY = -103,
  CARTESIAN_PATH_SERVICE_UNAVAILABLE = -104,
  CARTESIAN_PATH_CALL_TIMEOUT = -105,
  CARTESIAN_PATH_FRACTION_INCOMPLETE = -106,
  CARTESIAN_PATH_EMPTY_TRAJECTORY = -107,
  EXCEPTION = -200,
};

/**
 * @brief 笛卡尔直线运动服务服务器类
 * 与 RViz2 中勾选「使用笛卡尔空间规划执行」行为一致，通过 move_group 的 GetCartesianPath 服务计算直线路径并执行
 */
class MovelServer : public rclcpp::Node
{
public:
  explicit MovelServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MovelServer();

  bool initialize(int max_retries = 10, int retry_delay_seconds = 2);
  void spin();

private:
  bool wait_for_robot_description(int timeout_seconds = 30);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  rclcpp::Service<demo_interface::srv::Movel>::SharedPtr movel_service_;
  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr compute_cartesian_path_client_;

  void movelCallback(
    const std::shared_ptr<demo_interface::srv::Movel::Request> req,
    std::shared_ptr<demo_interface::srv::Movel::Response> res);

  bool movel(
    const geometry_msgs::msg::Pose& target_pose,
    float velocity_factor,
    float acceleration_factor,
    int32_t& error_code,
    std::string& message);

  std::string planning_group_name_;
  std::string base_frame_;
  std::string end_effector_link_;
  std::string compute_cartesian_path_service_name_;
  double cartesian_max_step_;

  std::mutex movel_mutex_;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_MOVEL_SERVER_H_
