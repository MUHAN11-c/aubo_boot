// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// 生命周期迁移（A8 / Robotics_Tutorial 2.16-1）用例：
//   - Unconfigured：运动类接口尚未创建（资源分配在 on_configure）；
//   - configure 依赖链违例（grasp 越级开启）→ FAILURE，停在 Unconfigured；
//   - Inactive：start_cycle / set_execution_armed / preview_* / go_to_photo_pose
//     一律拒绝并给出"未 Active"原因，只读 query_state 不受影响；
//   - activate/deactivate 往返：Active 下守卫放行（周期因无目标而按既有语义
//     失败），deactivate 后恢复拒绝，cleanup 后接口释放。
// 进程内构造真实节点（最小 URDF/SRDF 覆盖满足 MGI 构造，无需硬件/仿真/
// move_group 服务端；Active 下不触发任何真实规划/执行）。
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "approach_grasp_node_impl.hpp"

namespace
{
using namespace std::chrono_literals;
using peach_approach_grasp::ApproachGraspNode;

// 最小 URDF/SRDF：MGI 构造要求机器人模型可用且含规划组 manipulator_e5。
constexpr char kMinimalUrdf[] =
  "<robot name='minimal'>"
  "  <link name='base_link'/>"
  "  <joint name='j1' type='continuous'>"
  "    <parent link='base_link'/><child link='tip'/>"
  "  </joint>"
  "  <link name='tip'/>"
  "</robot>";
constexpr char kMinimalSrdf[] =
  "<robot name='minimal'>"
  "  <group name='manipulator_e5'><joint name='j1'/></group>"
  "</robot>";

rclcpp::NodeOptions makeNodeOptions(bool grasp_only = false)
{
  std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("robot_description", std::string(kMinimalUrdf)),
    rclcpp::Parameter("robot_description_semantic", std::string(kMinimalSrdf)),
  };
  if (grasp_only) {
    // 依赖链违例：grasp 开启而 execution 关闭（on_configure 必须 FAILURE）。
    overrides.emplace_back("grasp.enabled", true);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return options;
}

// 进程内联调骨架：被测节点与 MoveIt 伴随节点由独立 executor 线程自旋，
// 服务调用经 client 节点同步等待（spin_until_future_complete）。
struct Harness
{
  void start(const rclcpp::NodeOptions & options = makeNodeOptions())
  {
    node = std::make_shared<ApproachGraspNode>(options);
    executor.add_node(node->get_node_base_interface());
    executor.add_node(node->moveit_node());
    spinner = std::thread([this]() {executor.spin();});
    client_node = std::make_shared<rclcpp::Node>("lifecycle_test_client");
  }

  void stop()
  {
    executor.cancel();
    if (spinner.joinable()) {spinner.join();}
    client_node.reset();
    node.reset();
  }

  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const std::string & name, typename ServiceT::Request::SharedPtr request)
  {
    auto client = client_node->create_client<ServiceT>(name);
    if (!client->wait_for_service(2s)) {return nullptr;}
    auto future = client->async_send_request(std::move(request));
    if (rclcpp::spin_until_future_complete(client_node, future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<ApproachGraspNode> node;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
};

uint8_t stateId(const std::shared_ptr<ApproachGraspNode> & node)
{
  return node->get_current_state().id();
}

TEST(ApproachGraspLifecycle, UnconfiguredHasNoMotionInterfaces)
{
  {
    Harness harness;
    harness.start();
    EXPECT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    // 资源分配在 on_configure：Unconfigured 下运动类服务尚不存在。
    auto client = harness.client_node->create_client<std_srvs::srv::Trigger>(
      "/peach_approach_grasp_node/start_cycle");
    EXPECT_FALSE(client->wait_for_service(500ms));
    harness.stop();
  }
}

TEST(ApproachGraspLifecycle, ConfigureRejectsInvalidEnableChain)
{
  {
    Harness harness;
    harness.start(makeNodeOptions(true));
    harness.node->configure();
    EXPECT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    harness.stop();
  }
}

TEST(ApproachGraspLifecycle, InactiveRejectsMotionEntries)
{
  {
    Harness harness;
    harness.start();
    harness.node->configure();
    ASSERT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    const std::string prefix = "/peach_approach_grasp_node/";

    auto arm_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    arm_request->data = true;
    const auto arm = harness.call<std_srvs::srv::SetBool>(
      prefix + "set_execution_armed", arm_request);
    ASSERT_NE(arm, nullptr);
    EXPECT_FALSE(arm->success);
    EXPECT_NE(arm->message.find("Active"), std::string::npos);

    const auto start = harness.call<std_srvs::srv::Trigger>(
      prefix + "start_cycle", std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_NE(start, nullptr);
    EXPECT_FALSE(start->success);
    EXPECT_NE(start->message.find("Active"), std::string::npos);

    const auto preview = harness.call<std_srvs::srv::Trigger>(
      prefix + "preview_approach_insert",
      std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_NE(preview, nullptr);
    EXPECT_FALSE(preview->success);
    EXPECT_NE(preview->message.find("Active"), std::string::npos);

    const auto photo = harness.call<std_srvs::srv::Trigger>(
      prefix + "go_to_photo_pose", std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_NE(photo, nullptr);
    EXPECT_FALSE(photo->success);
    EXPECT_NE(photo->message.find("Active"), std::string::npos);

    // 只读入口不受 Active 限制。
    const auto query = harness.call<std_srvs::srv::Trigger>(
      prefix + "query_state", std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_NE(query, nullptr);
    EXPECT_TRUE(query->success);
    harness.stop();
  }
}

TEST(ApproachGraspLifecycle, ActivateDeactivateRoundtrip)
{
  {
    Harness harness;
    harness.start();
    harness.node->configure();
    ASSERT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    harness.node->activate();
    ASSERT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const std::string prefix = "/peach_approach_grasp_node/";

    // Active 下 arm 放行；start_cycle 通过 Active 守卫后按既有语义
    // （无可用目标）失败——失败原因不再是"未 Active"。
    auto arm_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    arm_request->data = true;
    const auto arm = harness.call<std_srvs::srv::SetBool>(
      prefix + "set_execution_armed", arm_request);
    ASSERT_NE(arm, nullptr);
    EXPECT_TRUE(arm->success);
    const auto start = harness.call<std_srvs::srv::Trigger>(
      prefix + "start_cycle", std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_NE(start, nullptr);
    EXPECT_FALSE(start->success);
    EXPECT_NE(start->message.find("selected_target"), std::string::npos);

    // deactivate：运动入口恢复拒绝（arm 同时被自动解除）。
    harness.node->deactivate();
    ASSERT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    const auto arm_inactive = harness.call<std_srvs::srv::SetBool>(
      prefix + "set_execution_armed", arm_request);
    ASSERT_NE(arm_inactive, nullptr);
    EXPECT_FALSE(arm_inactive->success);
    EXPECT_NE(arm_inactive->message.find("Active"), std::string::npos);

    // 再次激活后功能恢复（往返）。
    harness.node->activate();
    ASSERT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const auto arm_again = harness.call<std_srvs::srv::SetBool>(
      prefix + "set_execution_armed", arm_request);
    ASSERT_NE(arm_again, nullptr);
    EXPECT_TRUE(arm_again->success);

    // cleanup 回 Unconfigured，运动类接口释放：start_cycle 不再被应答
    // （用超时调用验证服务端点已销毁，不依赖图发现的传播时机）。
    harness.node->deactivate();
    harness.node->cleanup();
    ASSERT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    const auto after_cleanup = harness.call<std_srvs::srv::Trigger>(
      prefix + "start_cycle", std::make_shared<std_srvs::srv::Trigger::Request>());
    EXPECT_EQ(after_cleanup, nullptr);
    harness.stop();
  }
}

// preview 隔离回归（重构协议阶段 B）：previewContact 不写周期身份钉
// cycle_target_id_——预览过后手动周期（start_cycle）不得被预览残留的身份
// 污染（污染表现：btPrepareCycle 把新 selected 误判"目标身份变更"而失败）。
// 本环境无感知/重建数据，preview 会在质量门处失败（走不到规划段），但
// 可观测不变式相同：预览后 query_state 的 target_id 保持为空，且随后
// start_cycle 的失败原因仍是"无可用目标"而非身份冲突。
TEST(ApproachGraspLifecycle, PreviewDoesNotPolluteCycleIdentityPin)
{
  {
    Harness harness;
    harness.start();
    harness.node->configure();
    harness.node->activate();
    ASSERT_EQ(
      stateId(harness.node), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const std::string prefix = "/peach_approach_grasp_node/";

    const auto preview = harness.call<std_srvs::srv::Trigger>(
      prefix + "preview_approach_insert",
      std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_NE(preview, nullptr);
    // 无目标/精化几何：预览按既有语义在质量门失败（本用例不关心成败，
    // 只关心它不留身份副作用）。
    EXPECT_FALSE(preview->success);

    const auto query = harness.call<std_srvs::srv::Trigger>(
      prefix + "query_state", std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_NE(query, nullptr);
    EXPECT_TRUE(query->success);
    EXPECT_NE(query->message.find("\"target_id\":\"\""), std::string::npos)
      << "preview 污染了周期身份钉: " << query->message;

    // preview 后手动周期入口行为不变：通过 Active 守卫后因无目标失败。
    const auto start = harness.call<std_srvs::srv::Trigger>(
      prefix + "start_cycle", std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_NE(start, nullptr);
    EXPECT_FALSE(start->success);
    EXPECT_NE(start->message.find("selected_target"), std::string::npos);
    EXPECT_EQ(start->message.find("目标身份变更"), std::string::npos);
    harness.stop();
  }
}

}  // namespace

namespace
{

// 全局 Environment：只 init 不 shutdown。MGI 的进程级共享单例（shared TF
// listener / CurrentStateMonitor 自带节点）在退出静态析构期仍引用默认
// context；若测试先 shutdown，静态析构会在 context finalize 后访问它
// （段错误，exit 139）。保持 context 存活到进程退出即安全。
class LifecycleTestEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
};

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new LifecycleTestEnvironment);
  return RUN_ALL_TESTS();
}
