// 功能：可执行入口。构造 Lifecycle 技能节点与 MoveIt 伴随节点，多线程自旋。
// MoveIt/MTC 在 on_configure 装配；configure 失败停在 Unconfigured，可再触发。
#include "peach_manipulation/manipulation_skills_node_impl.hpp"
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<peach_manipulation::ManipulationSkillsNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  // MoveIt 伴随节点必须同 executor 自旋：MGI 的 CurrentStateMonitor 等
  // 默认回调组订阅依赖外部 executor（MGI 自旋的仅其私有回调组）。
  executor.add_node(node->moveit_node());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
