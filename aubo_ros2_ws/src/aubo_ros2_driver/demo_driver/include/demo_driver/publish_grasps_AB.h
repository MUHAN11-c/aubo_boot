/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_PUBLISH_GRASPS_AB_H_
#define DEMO_DRIVER_PUBLISH_GRASPS_AB_H_

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <demo_interface/srv/set_robot_io.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace demo_driver
{

struct CartesianSegment
{
  char axis;
  double offset;
};

/**
 * @brief A/B 工位交替 GraspNet 抓取 Worker（与 publish_grasps_client_worker 源码独立）
 *
 * 流程：A 识别 → 抓取 → B 放置 → 回 A；A 无抓取则 B 识别 → 抓取 → A 放置；均无则回 A 下一周期。
 * 发布最终选用 gripper_tip 位姿到 final_grasp_poses_topic（与 grasp_poses_base 语义一致）。
 */
class PublishGraspsABWorker : public rclcpp::Node
{
public:
  explicit PublishGraspsABWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PublishGraspsABWorker() = default;

  static std::shared_ptr<PublishGraspsABWorker> create(const rclcpp::NodeOptions& options);

  bool waitForServices(std::chrono::seconds timeout);

  bool run();

  bool runOneCycle();

  void clearGraspWindow();

  /** @param log_timeout_as_error 为 false 时超时仅 INFO（用于 A 侧尝试后切 B，不误报 ERROR） */
  bool waitForGraspWindowReady(bool log_timeout_as_error = true);

  bool requestGraspCapture(bool enable);
  void handleLoopControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  std::optional<std::pair<std::string, geometry_msgs::msg::Pose>> selectBestFromWindow();

  geometry_msgs::msg::Pose applyTransformationToPose(const geometry_msgs::msg::Pose& pose,
                                                     const Eigen::Matrix4d& transform_local);

  Eigen::Matrix4d buildGraspToEndEffectorTransform();

  bool runGraspMotion(const geometry_msgs::msg::Pose& target_pose);

  void onShutdown();

  void requestShutdown();
  bool isShutdownRequested() const { return shutdown_requested_.load(); }

private:
  bool runGraspApproach(const geometry_msgs::msg::Pose& pose_ee, double height_above, float vel, float acc);

  geometry_msgs::msg::Pose applyGraspZFlip180(const geometry_msgs::msg::Pose& pose);

  double verticalityScore(const geometry_msgs::msg::Pose& pose);

  void preparePlanningState(float velocity_factor, float acceleration_factor);

  void initMoveGroup();
  bool setGripperIo(int32_t io_index, bool high);
  bool moveToHome(float velocity_factor, float acceleration_factor);
  bool moveToPlacePointB(float velocity_factor, float acceleration_factor);
  bool runArcPath(char axis, double offset, float velocity_factor, float acceleration_factor);
  bool runArcPathSequence(const std::vector<CartesianSegment>& segments, float velocity_factor,
                          float acceleration_factor);

  void graspPosesCallback(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);

  void logCurrentEefPoseAndJoints(const char* stage_label);

  /** 发布与 grasp_poses_base 同语义的最终一条 tip 位姿（供 web 投影） */
  void publishFinalGraspPoseArray(const geometry_msgs::msg::Pose& grasp_tip_pose);

  void publishStatus(int cycle_count, int success_count, int fail_count, int last_step, bool is_running,
                     const std::string& pick_station_json = std::string());

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr grasp_poses_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr final_grasp_poses_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasp_capture_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr loop_control_service_;

  std::deque<geometry_msgs::msg::PoseArray> grasp_groups_window_;
  geometry_msgs::msg::PoseArray latest_grasp_poses_;
  /** 最近非空 PoseArray 的 header.frame_id，用于最终发布与感知一致 */
  std::string grasp_poses_reference_frame_;
  std::mutex window_mutex_;

  bool prefer_vertical_;
  double grasp_z_offset_;
  double height_above_;
  float joint_velocity_scaling_;
  float joint_acceleration_scaling_;
  std::string grasp_poses_topic_;
  double wait_poses_timeout_sec_;
  std::string grasp_capture_service_name_;
  double grasp_capture_service_timeout_sec_;
  std::string loop_control_service_name_;
  bool auto_start_loop_{ false };
  size_t grasp_window_size_;
  size_t min_groups_before_pick_;
  int32_t gripper_io_index_;
  double lift_offset_;
  double place_offset_y_;
  double place_offset_z_;
  double place_at_a_offset_z_{ -0.15 };
  double joint_cartesian_switch_delay_sec_;
  double cycle_delay_sec_;
  double fail_retry_delay_sec_;
  int max_cycles_;
  std::string status_topic_;
  int cartesian_max_points_;

  std::string final_grasp_poses_topic_;
  bool publish_final_grasp_poses_{ true };

  std::atomic<bool> shutdown_requested_{ false };
  std::atomic<bool> loop_enabled_{ false };
  std::atomic<bool> stop_after_cycle_{ false };
  std::atomic<bool> cycle_in_progress_{ false };
  int cycle_count_{ 0 };
  int success_count_{ 0 };
  int fail_count_{ 0 };
  int empty_grasp_cycles_{ 0 };

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<demo_interface::srv::SetRobotIO>::SharedPtr aubo_set_io_client_;

  static constexpr double kZMinLimit = 0.2;
  static const int32_t kGripperIoIndex;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_PUBLISH_GRASPS_AB_H_
