// 咖啡拉花工作流节点 — 源自 aubo_boot latte_backend（2026-09-30 移植）：
// RobotController 依赖从 demo_driver 切到 ivg_demo_services；服务 QoS 用
// rclcpp::ServicesQoS()；HeartParams 移至 latte_heart.hpp。
#ifndef LATTE_BACKEND__LATTE_WORKFLOW_NODE_HPP_
#define LATTE_BACKEND__LATTE_WORKFLOW_NODE_HPP_

#include <array>
#include <geometry_msgs/msg/pose.hpp>
#include <ivg_demo_services/robot_controller.hpp>
#include <ivg_interfaces/srv/run_latte_workflow.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "latte_backend/latte_heart.hpp"

namespace latte_backend
{

// 调试辅助：四元数 → RPY(度) → 格式化字符串（实现在 .cpp，gtest 不依赖）
std::string quatToRPYStr(const geometry_msgs::msg::Quaternion & q);
std::string poseToStr(const geometry_msgs::msg::Pose & p);

class LatteWorkflowNode : public rclcpp::Node
{
public:
  explicit LatteWorkflowNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  bool init();

private:
  std::unique_ptr<ivg_demo_services::RobotController> robot_;
  rclcpp::Service<ivg_interfaces::srv::RunLatteWorkflow>::SharedPtr srv_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::mutex mtx_;

  // 工作流参数（每次 service 回调刷新；lwf_* 参数名与 aubo_boot 一致）
  struct Params
  {
    double approach_h, retract_h;
    double vel, acc;
    int gripper_pin;
    std::string pattern_type;
    bool execute_latte;
    std::array<double, 6> coffee_joints;
    std::array<double, 6> place_coffee_joints;
    std::array<double, 6> pick_milk_joints;
    std::array<double, 6> nozzle_joints;
    std::array<double, 6> rotate_up_joints;
    HeartParams heart;
    double spout_offset_x, spout_offset_y, spout_offset_z;  // TCP→奶缸嘴（TCP 局部）
    double cup_x, cup_y, cup_z;  // 纸杯杯口世界坐标
    bool debug_verbose;
  };

  void readParameters();
  Params params_;

  void handleRunWorkflow(
    const std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Request> req,
    std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Response> res);

  // 工作流步骤（step0 取放咖啡杯保留未启用，多杯方案时取消注释）
  bool step0_pickCoffee();
  bool step0_placeCoffee();
  bool step1_pickMilk();
  bool step2_approachNozzle();
  bool step3_reorient();
  bool step4_pour();          // 绕世界 X 轴前倾 45°（当前禁用，roll 已绝对起算）
  bool step5_executeLatte();  // 心形三段式笛卡尔轨迹：委托 LatteTrajectoryGenerator
};

}  // namespace latte_backend

#endif  // LATTE_BACKEND__LATTE_WORKFLOW_NODE_HPP_
