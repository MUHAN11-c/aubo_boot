#ifndef DEMO_DRIVER_LATTE_WORKFLOW_NODE_H_
#define DEMO_DRIVER_LATTE_WORKFLOW_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <string>

#include "demo_driver/robot_controller.h"
#include <ivg_interfaces/srv/run_latte_workflow.hpp>
#include <ivg_interfaces/srv/replay_latte_trajectory.hpp>

namespace demo_driver
{

class LatteWorkflowNode : public rclcpp::Node
{
public:
    explicit LatteWorkflowNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    bool init();

private:
    // ── 成员 ──
    std::shared_ptr<RobotController> robot_;
    rclcpp::Service<ivg_interfaces::srv::RunLatteWorkflow>::SharedPtr srv_;
    rclcpp::Client<ivg_interfaces::srv::ReplayLatteTrajectory>::SharedPtr latte_client_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mtx_;

    // ── 参数 (每次 service 回调刷新) ──
    struct Params {
        double approach_h, retract_h;
        double vel, acc;
        int    gripper_pin;
        std::string pattern_type;
        bool execute_latte;
        std::array<double, 6> coffee_joints;
        std::array<double, 6> place_coffee_joints;
        std::array<double, 6> pick_milk_joints;
        std::array<double, 6> nozzle_joints;
        std::array<double, 6> rotate_up_joints;
    };

    void readParameters();
    Params params_;

    // ── Service ──
    void handleRunWorkflow(
        const std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Request> req,
        std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Response> res);

    // ── 工作流步骤 ──
    bool step0_pickCoffee();       // 保留未启用 (多杯方案)
    bool step0_placeCoffee();      // 保留未启用 (多杯方案)
    bool step1_pickMilk();         // [1/5] 取牛奶杯
    bool step2_approachNozzle();   // [2/5] 打奶泡
    bool step3_reorient();         // [3/5] 转腕朝上 → 放置咖啡杯
    bool step4_pour();             // [4/5] 嘴口倾倒
    bool step5_executeLatte();     // [5/5] 拉花执行
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_LATTE_WORKFLOW_NODE_H_
