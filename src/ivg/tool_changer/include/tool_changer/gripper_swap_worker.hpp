// 夹爪快换 Worker — 数据驱动，轨迹参数全部来自 tools.yaml。
// 源自 aubo_boot tool_changer（2026-09-30 移植，Jazzy 适配）：
//  - RobotController 从 demo_driver 切到 ivg_demo_services
//  - IO 客户端（ivg SetRobotIO）删除，统一走 RobotController→aubo_msgs/SetIO
//  - /debug/move_to_xyz 重复服务删除（由 ivg_demo_services 提供）
//  - ToolConfig 解析抽到 tool_config.cpp（纯核可测）
#ifndef TOOL_CHANGER__GRIPPER_SWAP_WORKER_HPP_
#define TOOL_CHANGER__GRIPPER_SWAP_WORKER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <ivg_demo_services/robot_controller.hpp>
#include <ivg_interfaces/msg/tool_changer_status.hpp>
#include <ivg_interfaces/srv/change_tool.hpp>
#include <ivg_interfaces/srv/get_current_tool.hpp>
#include <ivg_interfaces/srv/run_gripper_swap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "tool_changer/tool_config.hpp"

namespace tool_changer
{

using ivg_demo_services::CartesianSegment;

class GripperSwapWorker : public rclcpp::Node
{
public:
  explicit GripperSwapWorker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GripperSwapWorker() override = default;

  static std::shared_ptr<GripperSwapWorker> create(const rclcpp::NodeOptions & options);

  void run();
  void onShutdown();
  void requestShutdown();
  bool isShutdownRequested() const
  {
    return shutdown_requested_.load();
  }

private:
  struct ToolInfo
  {
    std::string id;
    std::string name;
    std::string type;
    std::string parameters;
  };

  // ── 轨迹原语 ──
  bool moveToHome(float vel, float acc);
  bool moveToJoints(const std::array<double, 6> & joints, float vel, float acc);
  bool moveToTargetXYZ(double x, double y, double z, float vel, float acc);
  bool moveToDockApproach(const ToolConfig & tool);
  bool pickTool(const ToolConfig & tool);
  bool releaseTool(const ToolConfig & tool, bool * tool_released = nullptr);

  // ── IO / 场景 ──
  bool setGripperIoSafe(bool open_gripper);
  bool updateSceneAttachment(const std::string & tool_id, bool attached);

  // ── 综合流程（数据驱动：释放当前 → 取目标 → 回 home）──
  bool changeToTool(const std::string & target_id);

  // ── 状态与辅助 ──
  void publishToolStatus(bool connected);
  bool sleepJointCartesianSwitchDelay(const char * where);

  // ── 服务回调 ──
  void onChangeTool(
    const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp);
  void onGetCurrentTool(
    const std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Response> resp);
  void onGripperSwapRequest(
    const std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Request> req,
    std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Response> resp);

  // ── 成员 ──
  std::map<std::string, ToolConfig> tool_configs_;
  std::unique_ptr<ivg_demo_services::RobotController> robot_;
  rclcpp::Client<ivg_interfaces::srv::ChangeTool>::SharedPtr scene_attach_client_;
  rclcpp::Client<ivg_interfaces::srv::ChangeTool>::SharedPtr scene_detach_client_;
  ToolInfo current_tool_;
  int32_t gripper_io_index_{7};
  bool simulation_skip_io_{false};

  float joint_velocity_scaling_{0.7f};
  float joint_acceleration_scaling_{0.3f};
  float home_velocity_scaling_{0.7f};
  float home_acceleration_scaling_{0.3f};
  double joint_cartesian_switch_delay_sec_{0.05};

  rclcpp::Publisher<ivg_interfaces::msg::ToolChangerStatus>::SharedPtr tool_status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Service<ivg_interfaces::srv::RunGripperSwap>::SharedPtr gripper_swap_srv_;
  rclcpp::Service<ivg_interfaces::srv::ChangeTool>::SharedPtr change_tool_srv_;
  rclcpp::Service<ivg_interfaces::srv::GetCurrentTool>::SharedPtr get_tool_srv_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

  std::atomic<bool> shutdown_requested_{false};
};

}  // namespace tool_changer

#endif  // TOOL_CHANGER__GRIPPER_SWAP_WORKER_HPP_
