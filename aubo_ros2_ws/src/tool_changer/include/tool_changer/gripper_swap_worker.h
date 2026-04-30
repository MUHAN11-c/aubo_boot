/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef TOOL_CHANGER_GRIPPER_SWAP_WORKER_H_
#define TOOL_CHANGER_GRIPPER_SWAP_WORKER_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <demo_interface/srv/set_robot_io.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tool_changer_interface/msg/tool_changer_status.hpp>
#include <tool_changer_interface/srv/change_tool.hpp>
#include <tool_changer_interface/srv/get_current_tool.hpp>
#include <tool_changer_interface/srv/run_gripper_swap.hpp>

namespace tool_changer
{

struct CartesianSegment
{
  char axis;
  double offset;
};

/**
 * @brief 夹爪快换 Worker（物理运动 + IO）
 *
 * 只负责物理快换动作，PlanningScene 附着/脱离由 scene_attach_worker 独立管理。
 * 快换完成后发布 /tool_changer_status，scene_attach_worker 订阅后自动更新场景显示。
 */
class GripperSwapWorker : public rclcpp::Node
{
public:
  explicit GripperSwapWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GripperSwapWorker() override = default;

  static std::shared_ptr<GripperSwapWorker> create(const rclcpp::NodeOptions& options);

  bool waitForServices(std::chrono::seconds timeout);
  void run();
  void onShutdown();
  void requestShutdown();
  bool isShutdownRequested() const { return shutdown_requested_.load(); }

private:
  struct ToolInfo {
    std::string id;
    std::string name;
    std::string type;
    std::string parameters;
  };

  static const ToolInfo kToolGripper0;
  static const ToolInfo kToolGripper2;
  static const ToolInfo kToolNone;

  // 快换原语
  bool releaseGripper(const std::vector<CartesianSegment>& approach,
                      const std::vector<CartesianSegment>& depart,
                      double settle_release_sec, double settle_close_sec);
  bool pickGripper(const std::vector<CartesianSegment>& approach,
                   const std::vector<CartesianSegment>& depart,
                   double settle_lock_sec);

  // 流程函数
  bool swapToGripper0();
  bool swapToGripper2();
  bool switchToGripper2();
  bool switchToGripper0();
  bool changeToTool(const std::string& target_id);

  // 运动原语
  void initMoveGroup();
  bool moveToJoints(const std::array<double, 6>& joints, float vel, float acc);
  bool moveToHome(float vel, float acc);
  bool moveToTargetXYZ(double x, double y, double z, float vel, float acc);
  bool runCartesianPath(const std::vector<CartesianSegment>& segments, float vel, float acc);
  bool runCartesianPath(char axis, double offset, float vel, float acc);

  // IO
  bool setGripperIoSafe(bool open_gripper);
  bool setGripperIo(int32_t io_index, bool high);

  // 状态
  ToolInfo current_tool_;
  void publishToolStatus(bool connected);

  // 服务回调
  enum class SwapResult { Success, Failed, Busy };
  SwapResult runSwapOperation(std::function<bool()> operation);

  void onChangeTool(const std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> req,
                    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> resp);
  void onGetCurrentTool(const std::shared_ptr<tool_changer_interface::srv::GetCurrentTool::Request> req,
                        std::shared_ptr<tool_changer_interface::srv::GetCurrentTool::Response> resp);
  void onGripperSwapRequest(const std::shared_ptr<tool_changer_interface::srv::RunGripperSwap::Request> req,
                            std::shared_ptr<tool_changer_interface::srv::RunGripperSwap::Response> resp);

  bool sleepJointCartesianSwitchDelay(const char* where);

  // 成员
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<demo_interface::srv::SetRobotIO>::SharedPtr set_io_client_;
  int32_t gripper_io_index_{7};
  bool simulation_skip_io_{false};

  float joint_velocity_scaling_{0.7f};
  float joint_acceleration_scaling_{0.3f};
  float home_velocity_scaling_{0.7f};
  float home_acceleration_scaling_{0.3f};
  double joint_cartesian_switch_delay_sec_{0.05};

  rclcpp::Publisher<tool_changer_interface::msg::ToolChangerStatus>::SharedPtr tool_status_pub_;

  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Service<tool_changer_interface::srv::RunGripperSwap>::SharedPtr gripper_swap_srv_;
  rclcpp::Service<tool_changer_interface::srv::ChangeTool>::SharedPtr change_tool_srv_;
  rclcpp::Service<tool_changer_interface::srv::GetCurrentTool>::SharedPtr get_tool_srv_;

  std::atomic<bool> swap_in_progress_{false};
  std::atomic<bool> shutdown_requested_{false};
};

}  // namespace tool_changer

#endif  // TOOL_CHANGER_GRIPPER_SWAP_WORKER_H_
