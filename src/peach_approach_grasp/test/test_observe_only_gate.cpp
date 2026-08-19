// Copyright 2026, aubo_e5_jazzy_ws authors
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
// RunTargetCycle 受理门矩阵（阶段 E 残局抬质量能力端）集成用例：
//   - OBSERVE_ONLY × goal 目标命中锁定集锚点缓存 × selected 为空 → 受理
//     （残局目标在 FULL 终局后已被感知计划 complete，selected 恒空，受理
//     不得再钉死 selected 缓存）；
//   - OBSERVE_ONLY × 目标不在锁定集 / 在锁定集但锚点缺失 → 拒绝；
//   - FULL/PREVIEW 维持原钉死语义（目标必须仍是感知 selected）；
//   - OBSERVE_ONLY 在 selected 非空且不等于 goal 目标时仍按锁定集命中受理。
// 进程内构造真实节点（最小 URDF/SRDF，模式同 test_lifecycle.cpp）；受理后
// 的周期在本环境无 TF，会在 PrepareCycle 按既有语义快速失败（abort），
// 本用例只钉受理门，每 goal 等终局收口后再发下一个，避免 running_ 互斥
// 干扰矩阵判定。
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "peach_harvest_msgs/action/run_target_cycle.hpp"
#include "peach_pose_msgs/msg/peach_target_observation_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "approach_grasp_node_impl.hpp"

namespace
{
using namespace std::chrono_literals;
using peach_approach_grasp::ApproachGraspNode;
using RunTargetCycle = peach_harvest_msgs::action::RunTargetCycle;
using ObservationArray = peach_pose_msgs::msg::PeachTargetObservationArray;
using Observation = peach_pose_msgs::msg::PeachTargetObservation;
using GoalHandle = rclcpp_action::Client<RunTargetCycle>::GoalHandle;

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

// 带有效锚点几何的单目标观测（confirmed；anchor_ok=false 模拟锚点缺失——
// 几何全零，TargetCache 判 has_anchor=false，条目在但无效）。
Observation makeObservation(const std::string & id, double x, bool anchor_ok = true)
{
  Observation item;
  item.target_id = id;
  item.confirmed = true;
  item.tracking_status = Observation::OBSERVED;
  item.candidate.target_id = id;
  item.candidate.status = peach_pose_msgs::msg::BagGraspCandidate::ACCEPT;
  if (anchor_ok) {
    item.candidate.bag_bottom.x = x;
    item.candidate.bag_bottom.z = 0.10;
    item.candidate.bag_neck.x = x;
    item.candidate.bag_neck.z = 0.18;
    item.candidate.translation_direction.z = 1.0;
    item.candidate.suggested_travel_m = 0.05F;
  }
  return item;
}

// 进程内联调骨架（同 test_lifecycle.cpp）：被测节点与 MoveIt 伴随节点由独立
// executor 线程自旋；client 节点负责发布感知观测与发送 action goal。
struct Harness
{
  void start()
  {
    std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("robot_description", std::string(kMinimalUrdf)),
      rclcpp::Parameter("robot_description_semantic", std::string(kMinimalSrdf)),
    };
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node = std::make_shared<ApproachGraspNode>(options);
    executor.add_node(node->get_node_base_interface());
    executor.add_node(node->moveit_node());
    spinner = std::thread([this]() {executor.spin();});
    client_node = std::make_shared<rclcpp::Node>("observe_only_gate_test_client");
    target_pub =
      client_node->create_publisher<ObservationArray>(
      "/peach/perception/target_observations", 10);
    action_client = rclcpp_action::create_client<RunTargetCycle>(
      client_node, "/peach_approach_grasp_node/run_target_cycle");
  }

  void stop()
  {
    executor.cancel();
    if (spinner.joinable()) {spinner.join();}
    action_client.reset();
    target_pub.reset();
    client_node.reset();
    node.reset();
  }

  void activateNode()
  {
    node->configure();
    node->activate();
    ASSERT_EQ(
      node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  // 连发多帧保证订阅侧（被测节点 executor 线程）至少处理一帧后再判定。
  void publishScene(const ObservationArray & scene)
  {
    for (int i = 0; i < 20; ++i) {
      target_pub->publish(scene);
      std::this_thread::sleep_for(50ms);
    }
  }

  // 发送 goal 并返回服务端 goal handle（nullptr=被拒）；受理的 goal 等待
  // 终局（本环境无 TF，周期在 PrepareCycle 快速 abort）后再返回，保证矩阵
  // 下一步不受 running_ 互斥影响。
  GoalHandle::SharedPtr sendGoal(const std::string & target_id, uint8_t mode)
  {
    RunTargetCycle::Goal goal;
    goal.request_id = "gate-" + target_id;
    goal.run_id = "run-gate";
    goal.cycle_id = goal.request_id;
    goal.target_id = target_id;
    goal.mode = mode;
    auto goal_future = action_client->async_send_goal(goal);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(client_node, goal_future, 5s),
      rclcpp::FutureReturnCode::SUCCESS);
    if (goal_future.wait_for(0s) != std::future_status::ready) {
      return nullptr;
    }
    const GoalHandle::SharedPtr handle = goal_future.get();
    if (!handle) {
      return nullptr;  // 受理门拒绝
    }
    auto result_future = action_client->async_get_result(handle);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(client_node, result_future, 20s),
      rclcpp::FutureReturnCode::SUCCESS);
    return handle;
  }

  std::shared_ptr<ApproachGraspNode> node;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::Publisher<ObservationArray>::SharedPtr target_pub;
  rclcpp_action::Client<RunTargetCycle>::SharedPtr action_client;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
};

// 残局场景（selected 为空）：OBSERVE_ONLY 以锁定集锚点缓存为受理依据。
TEST(ObserveOnlyGate, ResidualSceneEmptySelected)
{
  {
    Harness harness;
    harness.start();
    harness.activateNode();
    ASSERT_TRUE(harness.action_client->wait_for_action_server(5s));

    // 锁定集含 peach_1/peach_2（有效锚点）与 peach_3（confirmed 但锚点
    // 缺失）；selected 为空——残局抬质量的典型场景（FULL 终局后目标已被
    // 感知计划 complete）。
    ObservationArray scene;
    scene.harvest_run_id = "run-gate";
    scene.target_set_locked = true;
    scene.selected_target_id = "";
    scene.observations = {
      makeObservation("peach_1", 0.30), makeObservation("peach_2", 0.45),
      makeObservation("peach_3", 0.60, false)};
    harness.publishScene(scene);

    // OBSERVE_ONLY × 命中锁定集 × 空 selected → 受理（本修复的核心）。
    EXPECT_NE(
      harness.sendGoal("peach_1", RunTargetCycle::Goal::OBSERVE_ONLY), nullptr)
      << "OBSERVE_ONLY 残局目标（锁定集命中）必须受理";
    // OBSERVE_ONLY × 不在锁定集 → 拒绝。
    EXPECT_EQ(
      harness.sendGoal("peach_9", RunTargetCycle::Goal::OBSERVE_ONLY), nullptr)
      << "不在锁定集的目标必须拒绝";
    // OBSERVE_ONLY × 在锁定集但锚点缺失 → 拒绝。
    EXPECT_EQ(
      harness.sendGoal("peach_3", RunTargetCycle::Goal::OBSERVE_ONLY), nullptr)
      << "锚点缺失的锁定集目标必须拒绝";
    // FULL/PREVIEW 钉死语义不变：selected 为空时一律拒绝（即使锁定集有锚点）。
    EXPECT_EQ(
      harness.sendGoal("peach_1", RunTargetCycle::Goal::FULL), nullptr)
      << "FULL 仍钉死 selected，selected 为空必须拒绝";
    EXPECT_EQ(
      harness.sendGoal("peach_2", RunTargetCycle::Goal::PREVIEW), nullptr)
      << "PREVIEW 仍钉死 selected，selected 为空必须拒绝";
    harness.stop();
  }
}

// 正常场景（selected 非空）：FULL 钉死语义与 OBSERVE_ONLY 锁定集语义并存。
TEST(ObserveOnlyGate, SelectedPinnedSemanticsUnchanged)
{
  {
    Harness harness;
    harness.start();
    harness.activateNode();
    ASSERT_TRUE(harness.action_client->wait_for_action_server(5s));

    // selected=peach_1，锁定集同时含 peach_2（非 selected）。
    ObservationArray scene;
    scene.harvest_run_id = "run-gate";
    scene.target_set_locked = true;
    scene.selected_target_id = "peach_1";
    scene.observations = {
      makeObservation("peach_1", 0.30), makeObservation("peach_2", 0.45)};
    scene.observations[0].selected = true;
    harness.publishScene(scene);

    // FULL × goal==selected → 受理（原语义回归）。
    EXPECT_NE(
      harness.sendGoal("peach_1", RunTargetCycle::Goal::FULL), nullptr)
      << "FULL 且 goal==selected 必须受理";
    // FULL × goal!=selected（即使在锁定集）→ 拒绝（钉死语义不变）。
    EXPECT_EQ(
      harness.sendGoal("peach_2", RunTargetCycle::Goal::FULL), nullptr)
      << "FULL 且 goal!=selected 必须拒绝";
    // OBSERVE_ONLY × 锁定集命中但 != selected → 受理（不要求等于 selected）。
    EXPECT_NE(
      harness.sendGoal("peach_2", RunTargetCycle::Goal::OBSERVE_ONLY), nullptr)
      << "OBSERVE_ONLY 命中锁定集即受理，不要求等于 selected";
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
class ObserveOnlyGateTestEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
};

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new ObserveOnlyGateTestEnvironment);
  return RUN_ALL_TESTS();
}
