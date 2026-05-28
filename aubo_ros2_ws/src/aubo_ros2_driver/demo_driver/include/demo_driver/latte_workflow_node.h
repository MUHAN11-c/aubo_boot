#ifndef DEMO_DRIVER_LATTE_WORKFLOW_NODE_H_
#define DEMO_DRIVER_LATTE_WORKFLOW_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <string>

#include "demo_driver/robot_controller.h"
#include <ivg_interfaces/srv/run_latte_workflow.hpp>
#include <ivg_interfaces/srv/replay_latte_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

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
        double coffee_x, coffee_y, coffee_z;
        double lizhu_x,  lizhu_y,  lizhu_z;
        double cup0_x,   cup0_y,   cup0_z;
        double nozzle_x, nozzle_y, nozzle_z;
        double ref_x, ref_y, ref_z, ref_roll, ref_pitch, ref_yaw;
        double approach_h, retract_h;
        double vel, acc;
        int    gripper_pin;
        std::string pattern_type;
        bool execute_latte;    // false → skip step 7
        std::array<double, 6> coffee_joints;  // 拿咖啡杯预教关节角
        std::array<double, 6> place_coffee_joints;  // 放咖啡杯预教关节角
        std::array<double, 6> milk_up_joints;      // 牛奶杯开口朝上 (防倾倒)
        std::array<double, 6> milk_pour_joints;    // 牛奶杯嘴口倾倒 (拉花)
        std::array<double, 6> pick_milk_joints;    // 拿取牛奶杯预教关节角
        std::array<double, 6> nozzle_joints;       // 打花喷嘴终点预教关节角
        std::array<double, 6> rotate_up_joints;    // 转腕朝上中间位姿关节角
    };

    void readParameters();
    Params params_;

    // ── Service ──
    void handleRunWorkflow(
        const std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Request> req,
        std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Response> res);

    // ── 7 步工作流 ──
    bool step1_pickCoffee();
    bool step2_placeCoffee();
    bool step3_pickMilk();
    bool step4_approachNozzle();
    bool step5_reorient();
    bool step6_approachLizhu();
    bool step7_callLatte();
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_LATTE_WORKFLOW_NODE_H_
