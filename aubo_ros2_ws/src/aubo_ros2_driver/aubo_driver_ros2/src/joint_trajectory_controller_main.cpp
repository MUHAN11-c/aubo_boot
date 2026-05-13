/*
 * JointTrajectoryController 独立入口 —— 新框架驱动核心。
 *
 * 替代旧架构: aubo_ros2_trajectory_action + aubo_robot_simulator_ros2 + aubo_driver_ros2
 *
 * 数据流:
 *   MoveIt2 → FollowJointTrajectory Action → 本 Controller (C++, 200Hz 插值)
 *     → HardwareInterface::writeTrajectoryPoints() → 机器人
 *
 * 启动:
 *   ros2 run aubo_driver_ros2 joint_trajectory_controller \
 *     --ros-args -p server_host:=169.254.10.98
 *
 * 注意: 本控制器独占 TCP2CAN 模式，不能与旧 aubo_driver_ros2 同时运行。
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "aubo_driver_ros2/joint_trajectory_controller.h"
#include "aubo_driver_ros2/aubo_hardware_interface.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // ---- 读取参数 ----
    auto t = std::make_shared<rclcpp::Node>("_ctrl_params");
    t->declare_parameter<std::string>("server_host", "169.254.10.98");
    t->declare_parameter<int>("server_port", 8899);
    std::string host = t->get_parameter("server_host").as_string();
    int port = t->get_parameter("server_port").as_int();
    t.reset();

    // ---- 创建 HardwareInterface (独立连接) ----
    auto hw = std::make_shared<aubo_driver::AuboHardwareInterface>();
    if (!hw->init(host, port)) {
        RCLCPP_FATAL(rclcpp::get_logger("controller"), "HW init failed");
        rclcpp::shutdown();
        return 1;
    }

    // ---- 进入 TCP2CAN 模式 (独占, 不能与旧驱动同时运行) ----
    if (!hw->enterTcp2CanbusMode()) {
        RCLCPP_FATAL(rclcpp::get_logger("controller"),
            "Failed to enter TCP2CAN mode (is old driver still running?)");
        hw->shutdown();
        rclcpp::shutdown();
        return 1;
    }

    // ---- 创建控制器 ----
    rclcpp::NodeOptions o;
    o.automatically_declare_parameters_from_overrides(true);
    auto ctrl = std::make_shared<aubo_driver::JointTrajectoryController>(hw,
        std::vector<std::string>{
            "shoulder_joint","upperArm_joint","foreArm_joint",
            "wrist1_joint","wrist2_joint","wrist3_joint"},
        o);
    ctrl->configure();

    RCLCPP_INFO(rclcpp::get_logger("controller"),
        "新框架驱动已启动 (JointTrajectoryController + HardwareInterface TCP2CAN)");

    // ---- 运行 ----
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    exec.add_node(ctrl);
    exec.spin();

    // ---- 关闭 ----
    RCLCPP_INFO(rclcpp::get_logger("controller"),
        "Shutting down, leaving TCP2CAN mode...");
    ctrl->deactivate();
    ctrl.reset();
    hw->leaveTcp2CanbusMode();
    hw->shutdown();
    rclcpp::shutdown();
}
