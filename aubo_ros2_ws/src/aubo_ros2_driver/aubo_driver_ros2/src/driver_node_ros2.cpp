/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017-2018, AUBO Robotics
 * Ported to ROS2: main entry for aubo_driver_ros2.
 *
 * 单节点 + 显式 Callback Group：轨迹订阅、500Hz update_control、50Hz timer 分属不同组，
 * 由 MultiThreadedExecutor 并行执行，无需拆成多节点。符合 ROS2 推荐做法。
 *
 * 参考：https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html
 */

#include "aubo_driver_ros2/aubo_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<aubo_driver::AuboDriver>(0);
    node->run();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 10);
    executor.add_node(node);
    executor.spin();

    // 异常退出: 确保 leaveTcp2Canbus + logout，否则示教器无法接管
    RCLCPP_WARN(node->get_logger(), "Shutting down, leaving TCP2CAN mode...");
    node.reset();  // → ~AuboDriver → leaveTcp2CanbusMode + logout
    rclcpp::shutdown();
    return 0;
}
