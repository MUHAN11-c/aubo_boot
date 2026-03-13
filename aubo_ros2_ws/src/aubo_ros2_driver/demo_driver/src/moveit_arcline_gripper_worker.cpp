/**
 * @file moveit_arcline_gripper_worker.cpp
 * @brief 夹爪快换 Worker 节点：继承 MoveitArclineDemo。
 */

#include "demo_driver/moveit_arcline_gripper_worker.h"

#include <chrono>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#define CHECK(expr)                                                                                                       \
  do                                                                                                                      \
  {                                                                                                                       \
    if (!(expr))                                                                                                          \
      return false;                                                                                                       \
  } while (0)

namespace demo_driver
{

MoveitArclineGripperWorker::MoveitArclineGripperWorker(const rclcpp::NodeOptions& options) : MoveitArclineDemo(options)
{
  run_gripper_swap_srv_ = create_service<demo_interface::srv::RunGripperSwap>(
      "run_gripper_swap",
      std::bind(&MoveitArclineGripperWorker::runGripperSwapCallback, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(get_logger(), "服务 run_gripper_swap 已创建，通过 direction 选择 gripper0_to_gripper2 或 gripper2_to_gripper0");
}

std::shared_ptr<MoveitArclineGripperWorker> MoveitArclineGripperWorker::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<MoveitArclineGripperWorker>(options);
  node->initMoveGroup();
  return node;
}

bool MoveitArclineGripperWorker::gripper2_to_gripper0()
{
  RCLCPP_INFO(get_logger(), "gripper2_to_gripper0 开始执行");
  CHECK(moveToJoints({ 0.860766, -0.265055, 1.501074, 0.195106, 1.571464, 0.859643 }, 0.15f, 0.1f));
  const double y_step = 0.1;
  std::vector<CartesianSegment> segments = { { 'y', y_step }, { 'z', -0.243 }, { 'y', -y_step }, { 'z', -0.012 } };
  CHECK(runArcPathSequence(segments));
  CHECK(setDigitalOutput(7, true));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.3));
  CHECK(runArcPath(0.255));
  CHECK(setDigitalOutput(7, false));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

  CHECK(moveToPose(0.36767 - 0.003, 0.24267 + 0.105, 0.0405 + 0.185 + 0.1, 0.7068, 0.7074, 0.0002, -0.0005, false, 0.1f,
                   0.1f));
  CHECK(setDigitalOutput(7, true));
  CHECK(runArcPath(-0.255));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(setDigitalOutput(7, false));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(runArcPath(0.255));
  CHECK(moveToHome());
  return true;
}

bool MoveitArclineGripperWorker::gripper0_to_gripper2()
{
  RCLCPP_INFO(get_logger(), "gripper0_to_gripper2 开始执行");
  CHECK(moveToPose(0.36767 - 0.003, 0.24267 + 0.105, 0.0405 + 0.185 + 0.1, 0.7068, 0.7074, 0.0002, -0.0005, false, 0.15f,
                   0.1f));
  CHECK(runArcPath(-0.255));
  CHECK(setDigitalOutput(7, true));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(runArcPath(0.255));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(setDigitalOutput(7, false));

  const double y_step = 0.1;
  CHECK(moveToJoints({ 0.860766, -0.265055, 1.501074, 0.195106, 1.571464, 0.859643 }, 0.15f, 0.1f));
  CHECK(setDigitalOutput(7, true));
  CHECK(runArcPath(-0.255));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(setDigitalOutput(7, false));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  std::vector<CartesianSegment> segments1 = { { 'z', 0.012 }, { 'y', y_step }, { 'z', 0.243 } };
  CHECK(runArcPathSequence(segments1));

  CHECK(moveToHome());
  return true;
}

bool MoveitArclineGripperWorker::run()
{
  CHECK(moveToJoints({ 0.860766, -0.265055, 1.501074, 0.195106, 1.571464, 0.859643 }, 0.15f, 0.1f));
  const double y_step = 0.1;
  std::vector<CartesianSegment> segments = { { 'y', y_step }, { 'z', -0.243 }, { 'y', -y_step }, { 'z', -0.012 } };
  CHECK(runArcPathSequence(segments));
  CHECK(setDigitalOutput(7, true));
  std::this_thread::sleep_for(std::chrono::duration<double>(1));
  std::vector<CartesianSegment> segments1 = { { 'z', 0.012 }, { 'y', y_step }, { 'z', 0.243 } };
  CHECK(runArcPathSequence(segments1));
  CHECK(setDigitalOutput(7, false));

  CHECK(moveToPose(0.36767 - 0.003, 0.24267 + 0.105, 0.0405 + 0.185 + 0.1, 0.7068, 0.7074, 0.0002, -0.0005, false, 0.1f,
                   0.1f));
  CHECK(setDigitalOutput(7, true));
  CHECK(runArcPath(-0.255));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(setDigitalOutput(7, false));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(runArcPath(0.255));
  std::this_thread::sleep_for(std::chrono::seconds(2));

  return true;
}

void MoveitArclineGripperWorker::runGripperSwapCallback(
    const std::shared_ptr<demo_interface::srv::RunGripperSwap::Request> request,
    std::shared_ptr<demo_interface::srv::RunGripperSwap::Response> response)
{
  const std::string& direction = request->direction;
  if (direction != "gripper0_to_gripper2" && direction != "gripper2_to_gripper0")
  {
    response->success = false;
    response->message = "未知 direction，仅支持 gripper0_to_gripper2 或 gripper2_to_gripper0";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "收到 run_gripper_swap 请求 direction=%s，已在后台线程启动", direction.c_str());
  std::thread worker([this, direction]() {
    bool ok = (direction == "gripper0_to_gripper2") ? gripper0_to_gripper2() : gripper2_to_gripper0();
    if (ok)
      RCLCPP_INFO(get_logger(), "后台执行完成: 成功 (%s)", direction.c_str());
    else
      RCLCPP_WARN(get_logger(), "后台执行完成: 失败 (%s)", direction.c_str());
  });
  worker.detach();

  response->success = true;
  response->message = "已启动: " + direction;
}

}  // namespace demo_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = demo_driver::MoveitArclineGripperWorker::create(options);

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

  RCLCPP_INFO(node->get_logger(), "夹爪快换 Worker 已就绪，调用服务 run_gripper_swap 执行夹爪快换");
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
