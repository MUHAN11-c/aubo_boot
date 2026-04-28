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

/** 笛卡尔路径单段 */
struct CartesianSegment
{
  char axis;      // 'x' / 'y' / 'z'
  double offset;  // 偏移量 (m)
};

/**
 * @brief 夹爪快换 Worker（独立节点）
 *
 * 多线程模型：
 * - main() 用 MultiThreadedExecutor(2) spin
 * - 服务回调在 MutuallyExclusive 组，阻塞完成整个快换流程
 * - 另一线程 spin MoveIt 轨迹反馈 + IO 服务响应
 * - setGripperIo 内 async_send_request + future.wait_for：
 *   回调线程阻塞在 wait_for，响应由另一 spin 线程接收并 set future ready
 *
 * 工具状态机：
 *   known tool ──releaseGripper──→ (none) ──pickGripper──→ target tool
 *   同工具切换 → 跳过，直接返回成功
 */
class GripperSwapWorker : public rclcpp::Node
{
public:
  explicit GripperSwapWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GripperSwapWorker() override = default;

  static std::shared_ptr<GripperSwapWorker> create(const rclcpp::NodeOptions& options);

  /** 等待 /aubo_driver/set_io 服务就绪 */
  bool waitForServices(std::chrono::seconds timeout);

  /** 主循环：多线程 spin + 阻塞等待退出 */
  void run();

  /** 优雅退出：回安全位、关 IO */
  void onShutdown();

  /** 请求退出：SIGINT 时调用 */
  void requestShutdown();

  /** 是否已请求退出 */
  bool isShutdownRequested() const { return shutdown_requested_.load(); }

private:
  // ═══════════════════════════════════════════════════════════════
  // 工具定义
  // ═══════════════════════════════════════════════════════════════

  struct ToolInfo {
    std::string id;
    std::string name;
    std::string type;        // "gripper" / "suction" / "other"
    std::string parameters;  // JSON
  };

  static const ToolInfo kToolGripper0;
  static const ToolInfo kToolGripper2;
  static const ToolInfo kToolNone;

  // ═══════════════════════════════════════════════════════════════
  // 夹爪快换原语：放 / 取
  // ═══════════════════════════════════════════════════════════════

  /** 放夹爪：接近 dock → 开IO释放 → 停留 → 抬离 → 关IO锁机构 */
  bool releaseGripper(const std::vector<CartesianSegment>& approach,
                      const std::vector<CartesianSegment>& depart,
                      double settle_release_sec,
                      double settle_close_sec);

  /** 取夹爪：开IO准备 → 接近 dock → 停留 → 关IO锁定 → 停留 → 抬离 */
  bool pickGripper(const std::vector<CartesianSegment>& approach,
                   const std::vector<CartesianSegment>& depart,
                   double settle_lock_sec);

  // ═══════════════════════════════════════════════════════════════
  // 流程函数
  // ═══════════════════════════════════════════════════════════════

  bool swapToGripper0();   // gripper2 → gripper0（先放后取）
  bool swapToGripper2();   // gripper0 → gripper2（先放后取）
  bool switchToGripper2(); // → gripper2（当前无工具，直接取）
  bool switchToGripper0(); // → gripper0（当前无工具，直接取）

  /** 自动根据当前工具选择放/取序列 */
  bool changeToTool(const std::string& target_id);

  // ═══════════════════════════════════════════════════════════════
  // 运动原语
  // ═══════════════════════════════════════════════════════════════

  void initMoveGroup();

  bool moveToJoints(const std::array<double, 6>& joints, float vel, float acc);
  bool moveToPose(double x, double y, double z,
                  double qx, double qy, double qz, double qw,
                  float vel, float acc);
  bool moveToHome(float vel, float acc);
  bool moveToTargetXYZ(double target_x, double target_y, double target_z, float vel, float acc);

  /** 多段笛卡尔路径，不做 Z 安全下限裁剪（快换工位需要） */
  bool runCartesianPath(const std::vector<CartesianSegment>& segments, float vel, float acc);
  /** 单轴 → 委托到 runCartesianPath */
  bool runCartesianPath(char axis, double offset, float vel, float acc);

  // ═══════════════════════════════════════════════════════════════
  // IO
  // ═══════════════════════════════════════════════════════════════

  /** 设置夹爪 IO（仿真模式下自动跳过），open_gripper=true 释放/ false 锁定 */
  bool setGripperIoSafe(bool open_gripper);
  /** 实际调用 /aubo_driver/set_io */
  bool setGripperIo(int32_t io_index, bool high);

  // ═══════════════════════════════════════════════════════════════
  // 工具状态
  // ═══════════════════════════════════════════════════════════════

  ToolInfo current_tool_;

  /** 发布 /tool_changer_status */
  void publishToolStatus(bool connected);

  // ═══════════════════════════════════════════════════════════════
  // 服务回调
  // ═══════════════════════════════════════════════════════════════

  enum class SwapResult { Success, Failed, Busy };

  /** 带并发守护的交换操作执行器，统一 try/catch 和响应填充 */
  SwapResult runSwapOperation(std::function<bool()> operation);

  void onChangeTool(
      const std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> request,
      std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> response);

  void onGetCurrentTool(
      const std::shared_ptr<tool_changer_interface::srv::GetCurrentTool::Request> request,
      std::shared_ptr<tool_changer_interface::srv::GetCurrentTool::Response> response);

  void onGripperSwapRequest(
      const std::shared_ptr<tool_changer_interface::srv::RunGripperSwap::Request> request,
      std::shared_ptr<tool_changer_interface::srv::RunGripperSwap::Response> response);

  // ═══════════════════════════════════════════════════════════════
  // 辅助
  // ═══════════════════════════════════════════════════════════════

  bool sleepJointCartesianSwitchDelay(const char* where);

  // ═══════════════════════════════════════════════════════════════
  // 成员
  // ═══════════════════════════════════════════════════════════════

  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // IO
  rclcpp::Client<demo_interface::srv::SetRobotIO>::SharedPtr set_io_client_;
  int32_t gripper_io_index_{7};

  // 运动参数
  float joint_velocity_scaling_{0.7f};
  float joint_acceleration_scaling_{0.3f};
  float home_velocity_scaling_{0.7f};
  float home_acceleration_scaling_{0.3f};
  double joint_cartesian_switch_delay_sec_{0.05};

  // 话题
  rclcpp::Publisher<tool_changer_interface::msg::ToolChangerStatus>::SharedPtr tool_status_pub_;

  // 服务
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Service<tool_changer_interface::srv::RunGripperSwap>::SharedPtr gripper_swap_srv_;
  rclcpp::Service<tool_changer_interface::srv::ChangeTool>::SharedPtr     change_tool_srv_;
  rclcpp::Service<tool_changer_interface::srv::GetCurrentTool>::SharedPtr get_tool_srv_;

  // 并发
  std::atomic<bool> swap_in_progress_{false};
  std::atomic<bool> shutdown_requested_{false};
};

}  // namespace tool_changer

#endif  // TOOL_CHANGER_GRIPPER_SWAP_WORKER_H_
