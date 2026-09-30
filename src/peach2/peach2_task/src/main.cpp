// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "peach2_task/task_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<peach2_task::TaskNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
