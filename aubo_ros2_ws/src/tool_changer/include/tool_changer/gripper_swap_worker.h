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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "demo_driver/robot_controller.h"
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ivg_interfaces/srv/move_to_pose.hpp>
#include <ivg_interfaces/msg/tool_changer_status.hpp>
#include <ivg_interfaces/srv/change_tool.hpp>
#include <ivg_interfaces/srv/get_current_tool.hpp>
#include <ivg_interfaces/srv/run_gripper_swap.hpp>

namespace tool_changer
{

using demo_driver::CartesianSegment;

// ── 轨迹策略 ──
enum class TrajectoryStrategy { kVertical, kSlide };

struct VerticalStrategyParams
{
  double depth      = 0.210;
  double lift       = 0.210;
  double settle_sec = 0.5;
};

struct SlideStrategyParams
{
  double depth       = 0.210;
  double seat        = 0.012;
  double slide_y     = 0.100;
  double lift        = 0.210;
  double settle_sec  = 0.5;
  double release_sec = 0.3;
  double lock_sec    = 0.5;
};

// ── 从 tools.yaml 加载的运行时轨迹配置 ──
struct ToolConfig
{
  std::string id;
  std::string name;
  std::string type;
  std::string parameters;

  TrajectoryStrategy strategy = TrajectoryStrategy::kVertical;
  std::array<double, 6> dock_approach_joints{};

  // 笛卡尔直线接近（dock_above 的 XYZ，姿态保持当前值）— 关节角缺失时的替代方案喵~
  bool has_dock_approach_xyz = false;
  std::array<double, 3> dock_approach_xyz{};

  VerticalStrategyParams vertical;
  SlideStrategyParams slide;
};

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
  struct ToolInfo
  {
    std::string id;
    std::string name;
    std::string type;
    std::string parameters;
  };

  // ═══════════════════════════════════════════════════════════════
  // 轨迹原语（四类 — 已泛化为数据驱动）
  // ═══════════════════════════════════════════════════════════════

  // ── 0. 配置加载 ──
  void loadToolConfig();

  // ── 1. 回 home ──
  bool moveToHome(float vel, float acc);

  // ── 2. 到固定点位 ──
  bool moveToJoints(const std::array<double, 6>& joints, float vel, float acc);
  bool moveToTargetXYZ(double x, double y, double z, float vel, float acc);
  bool moveToDockApproach(const ToolConfig& tool);

  // ── 3. 取轨迹（泛型，按 strategy 分发） ──
  bool pickTool(const ToolConfig& tool);

  // ── 4. 放轨迹（泛型，按 strategy 分发） ──
  bool releaseTool(const ToolConfig& tool, bool* tool_released = nullptr);

  // ═══════════════════════════════════════════════════════════════
  // 笛卡尔路径（底层）
  // ═══════════════════════════════════════════════════════════════

  bool runCartesianPath(const std::vector<CartesianSegment>& segments, float vel, float acc);
  bool runCartesianPath(char axis, double offset, float vel, float acc);

  // ═══════════════════════════════════════════════════════════════
  // IO
  // ═══════════════════════════════════════════════════════════════

  bool setGripperIoSafe(bool open_gripper);
  bool setGripperIo(int32_t io_index, bool high);
  bool updateSceneAttachment(const std::string& tool_id, bool attached);

  // ═══════════════════════════════════════════════════════════════
  // 综合流程（数据驱动：释放当前 → 取目标 → 回 home）
  // ═══════════════════════════════════════════════════════════════

  bool changeToTool(const std::string& target_id);

  // ═══════════════════════════════════════════════════════════════
  // 工具状态 & 辅助
  // ═══════════════════════════════════════════════════════════════

  void initMoveGroup();
  ToolInfo current_tool_;
  void publishToolStatus(bool connected);
  bool sleepJointCartesianSwitchDelay(const char* where);

  // ═══════════════════════════════════════════════════════════════
  // 服务回调
  // ═══════════════════════════════════════════════════════════════

  void onChangeTool(const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
                    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp);
  void onGetCurrentTool(const std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Request> req,
                        std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Response> resp);
  void onGripperSwapRequest(const std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Request> req,
                            std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Response> resp);
  void onDebugMoveToXYZ(const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> req,
                        std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> resp);

  // ═══════════════════════════════════════════════════════════════
  // 成员变量
  // ═══════════════════════════════════════════════════════════════

  std::map<std::string, ToolConfig> tool_configs_;

  std::shared_ptr<demo_driver::RobotController> robot_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<ivg_interfaces::srv::SetRobotIO>::SharedPtr set_io_client_;
  rclcpp::Client<ivg_interfaces::srv::ChangeTool>::SharedPtr scene_attach_client_;
  rclcpp::Client<ivg_interfaces::srv::ChangeTool>::SharedPtr scene_detach_client_;
  int32_t gripper_io_index_{ 7 };
  bool simulation_skip_io_{ false };

  float joint_velocity_scaling_{ 0.7f };
  float joint_acceleration_scaling_{ 0.3f };
  float home_velocity_scaling_{ 0.7f };
  float home_acceleration_scaling_{ 0.3f };
  double joint_cartesian_switch_delay_sec_{ 0.05 };

  rclcpp::Publisher<ivg_interfaces::msg::ToolChangerStatus>::SharedPtr tool_status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;  // 周期性发布 /tool_changer_status，确保后连接订阅者能收到喵~

  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Service<ivg_interfaces::srv::RunGripperSwap>::SharedPtr gripper_swap_srv_;
  rclcpp::Service<ivg_interfaces::srv::ChangeTool>::SharedPtr change_tool_srv_;
  rclcpp::Service<ivg_interfaces::srv::GetCurrentTool>::SharedPtr get_tool_srv_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Service<ivg_interfaces::srv::MoveToPose>::SharedPtr debug_move_xyz_srv_;

  std::atomic<bool> shutdown_requested_{ false };
};

}  // namespace tool_changer

#endif  // TOOL_CHANGER_GRIPPER_SWAP_WORKER_H_
