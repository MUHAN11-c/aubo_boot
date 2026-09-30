#include <memory>

#include "manipulation_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<peach2_manipulation::ManipulationNode>();
  // Enough threads for: state subscriptions, service/action callbacks, timers, the GetDecision /
  // SetIO client responses and the MoveIt companion node, while the worker thread blocks.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(node->get_node_base_interface());
  // MGI spins only its private group; CurrentStateMonitor and our execute_trajectory client use
  // the companion node's default group.
  executor.add_node(node->moveit_node());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
