/*
 * AuboDashboardNode 入口 —— 独立进程管理所有非实时 SDK 功能。
 *
 * 用法:
 *   ros2 run aubo_driver_ros2 aubo_dashboard_node --ros-args -p server_host:=<IP>
 *
 * Lifecycle 管理:
 *   ros2 lifecycle set /aubo_dashboard configure
 *   ros2 lifecycle set /aubo_dashboard activate
 *   ros2 service call /aubo/startup std_srvs/srv/Trigger
 *   ros2 lifecycle set /aubo_dashboard deactivate
 *   ros2 lifecycle set /aubo_dashboard shutdown
 *
 * 异常关闭: Ctrl+C 或 SIGTERM 时自动 leaveTcp2CanbusMode + logout，
 * 确保示教器能重新接管控制权。
 */

#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include "aubo_driver_ros2/aubo_dashboard_node.h"

static std::shared_ptr<aubo_driver::AuboDashboardNode> g_node;

static void sigHandler(int) {
    RCLCPP_WARN(rclcpp::get_logger("dashboard"), "Received signal, forcing shutdown...");
    if (g_node) {
        // 强制触发 Lifecycle 清理: cleanup → shutdown
        g_node->~AuboDashboardNode();  // 内部 hw_->shutdown() → leaveTcp2Canbus + logout
        g_node.reset();
    }
    rclcpp::shutdown();
    std::_Exit(0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);

    g_node = std::make_shared<aubo_driver::AuboDashboardNode>(opts);

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(g_node->get_node_base_interface());
    executor.spin();

    // 正常退出
    RCLCPP_INFO(rclcpp::get_logger("dashboard"), "Normal shutdown, leaving TCP2CAN mode...");
    g_node.reset();  // → ~AuboHardwareInterface → leaveTcp2Canbus + logout
    rclcpp::shutdown();
    return 0;
}
