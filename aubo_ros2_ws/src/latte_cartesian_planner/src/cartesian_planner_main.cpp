#include "latte_cartesian_planner/cartesian_planner_node.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<latte_cartesian_planner::CartesianPlannerNode>(opts);
  node->init();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
