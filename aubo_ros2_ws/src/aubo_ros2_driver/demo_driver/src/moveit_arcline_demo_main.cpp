/**
 * @file moveit_arcline_demo_main.cpp
 * @brief moveit_arcline_demo_node 的 main 入口。
 */

#include "demo_driver/moveit_arcline_demo.h"

#include <chrono>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = demo_driver::MoveitArclineDemo::create(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const int kServiceWaitSec = 10;
  if (!node->waitForServices(std::chrono::seconds(kServiceWaitSec)))
  {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  bool ok = node->run();

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
