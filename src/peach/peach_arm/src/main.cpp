// 功能：可执行入口。构造 Lifecycle 技能节点与 MoveIt 伴随节点，多线程自旋。
// MoveIt/MTC 在 on_configure 装配；configure 失败停在 Unconfigured，可再触发。
#include "peach_arm/manipulation_skills_node.hpp"
#include <cstdio>
#include <exception>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<peach_arm::ManipulationSkillsNode>();
  const auto context = node->get_node_base_interface()->get_context();
  const std::weak_ptr<peach_arm::ManipulationSkillsNode> weak_node = node;
  const auto shutdown_callback = context->add_pre_shutdown_callback(
    [weak_node]() noexcept {
      if (const auto current = weak_node.lock()) {
        current->prepareForShutdown();
      }
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  // MoveIt 伴随节点必须同 executor 自旋：MGI 的 CurrentStateMonitor 等
  // 默认回调组订阅依赖外部 executor（MGI 自旋的仅其私有回调组）。
  executor.add_node(node->moveit_node());
  int exit_code = 0;
  try {
    executor.spin();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "peach_arm executor 异常: %s\n", error.what());
    exit_code = 1;
  } catch (...) {
    std::fputs("peach_arm executor 未知异常\n", stderr);
    exit_code = 1;
  }
  // executor.cancel() 等正常返回路径同样先停轨；不能只等节点析构。
  node->prepareForShutdown();
  rclcpp::shutdown();
  context->remove_pre_shutdown_callback(shutdown_callback);
  return exit_code;
}
