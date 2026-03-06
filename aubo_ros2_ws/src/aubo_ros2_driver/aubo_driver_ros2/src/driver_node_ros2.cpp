/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017-2018, AUBO Robotics
 * Ported to ROS2: main entry for aubo_driver_ros2.
 */

#include "aubo_driver_ros2/aubo_driver.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.allow_undeclared_parameters(true);
    opts.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<aubo_driver::AuboDriver>(0);
    node->run();

    rclcpp::Rate loop_rate(node->UPDATE_RATE_);
    while(rclcpp::ok()) {
        node->updateControlStatus();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    RCLCPP_WARN(node->get_logger(), "Exiting robot_driver");
    rclcpp::shutdown();
    return 0;
}
