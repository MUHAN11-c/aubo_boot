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
// 环境几何保护区（阶段 F1）MTC 入口剔除的终局路由集成用例：
// 进程内构造真实节点（最小 URDF/SRDF，模式同 test_observe_only_gate.cpp），
// 以话题数据驱动 FULL 周期走到 MTC 接触段入口校验——
//   - 感知观测话题提供有效锚点（selected=peach_1，持续 10Hz 保新鲜度）；
//   - 重建诊断话题伪造覆盖达标（captured_views=3/基线/深度比），使
//     AcquireViews 首轮即 readyToFinalize 放行，不做任何移动；
//   - finalize/save 服务缺席（timeouts.service_s=0.5 快速失败），精化缺席
//     走降级链：degradedEntryPoint 入口 = center − axis·(travel+standoff)；
//   - scan.protected_zones 罩住该入口点 → MTC 规划前拒规划，action 终局
//     outcome=SKIPPED_UNREACHABLE 且 reason 含“入口在保护区”。
// reason 断言即路由判别：若周期在更早阶段失败（视点空/扫描不可达/再确认
// 失败），reason 文案不同。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "peach_harvest_msgs/action/run_target_cycle.hpp"
#include "peach_harvest_msgs/msg/reconstruction_status.hpp"
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
using ReconstructionStatus = peach_harvest_msgs::msg::ReconstructionStatus;

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

// 单目标观测：锚点 center=(0.4,0,0.14)、轴 +z、suggested_travel=0.05。
// 降级链入口 = center − axis·(travel 0.05 + fallback_standoff 0.05)
//            = (0.40, 0, 0.04)，正是保护区盒罩住的点。
Observation makeObservation()
{
  Observation item;
  item.target_id = "peach_1";
  item.confirmed = true;
  item.selected = true;
  item.tracking_status = Observation::OBSERVED;
  item.candidate.target_id = "peach_1";
  item.candidate.status = peach_pose_msgs::msg::BagGraspCandidate::ACCEPT;
  item.candidate.bag_bottom.x = 0.40;
  item.candidate.bag_bottom.z = 0.10;
  item.candidate.bag_neck.x = 0.40;
  item.candidate.bag_neck.z = 0.18;
  item.candidate.translation_direction.z = 1.0;
  item.candidate.suggested_travel_m = 0.05F;
  item.candidate.entry_pose.orientation.w = 1.0;
  return item;
}

// 伪造覆盖达标的重建诊断：AcquireViews 首轮 readyToFinalize 即放行（不移动）。
ReconstructionStatus makeDiagnostics()
{
  ReconstructionStatus status;
  status.harvest_run_id = "run-pz";
  status.selected_target_id = "peach_1";
  status.target_id = "peach_1";
  status.state = "IDLE";  // 无精化：FinalizeAndValidate 走降级链
  status.captured_views = 3;
  status.max_baseline_deg = 20.0;
  status.mean_nearest_baseline_deg = 10.0;
  status.valid_depth_ratio = 0.5;
  return status;
}

struct Harness
{
  void start()
  {
    std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("robot_description", std::string(kMinimalUrdf)),
      rclcpp::Parameter("robot_description_semantic", std::string(kMinimalSrdf)),
      // 进程内最小环境无真实 BT 路径覆盖，显式指向仓库内 harvest_tree.xml。
      rclcpp::Parameter("behavior_tree.xml", std::string(HARVEST_TREE_XML)),
      // 无 robot_state_publisher：相机与末端钉到 base_link，TF 同帧恒等。
      rclcpp::Parameter("frames.camera", "base_link"),
      rclcpp::Parameter("frames.tip", "base_link"),
      rclcpp::Parameter("frames.tool", "base_link"),
      // 走完 MTC 前的全部分支需要执行+抓取使能（action 驱动自动 arm）；
      // 无 robot_status 发布者，安全门的机器人路关闭（目标路仍生效）。
      rclcpp::Parameter("execution.enabled", true),
      rclcpp::Parameter("grasp.enabled", true),
      rclcpp::Parameter("execution.require_robot_status", false),
      // finalize/save 服务缺席时快速失败（默认 3s×2 拖慢用例）。
      rclcpp::Parameter("timeouts.service_s", 0.5),
      // 保护区罩住降级链入口点 (0.40, 0, 0.04)（含表面，闭区间）。
      rclcpp::Parameter(
        "scan.protected_zones",
        std::vector<double>{0.30, -0.10, -0.05, 0.50, 0.10, 0.15}),
    };
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node = std::make_shared<ApproachGraspNode>(options);
    executor.add_node(node->get_node_base_interface());
    executor.add_node(node->moveit_node());
    spinner = std::thread([this]() {executor.spin();});
    client_node = std::make_shared<rclcpp::Node>("protected_zone_entry_test_client");
    target_pub = client_node->create_publisher<ObservationArray>(
      "/peach/perception/target_observations", 10);
    // 订阅侧为 transient_local 闩锁；持续重发保证数据年龄新鲜（质量门
    // maximum_data_age_s=2s）。
    diag_pub = client_node->create_publisher<ReconstructionStatus>(
      "/peach/reconstruction/diagnostics",
      rclcpp::QoS(1).reliable().transient_local());
    action_client = rclcpp_action::create_client<RunTargetCycle>(
      client_node, "/peach_approach_grasp_node/run_target_cycle");
    // 场景发布线程：10Hz 观测（安全门/再确认新鲜度）+ 5Hz 诊断（数据年龄）。
    publishing.store(true);
    pub_thread = std::thread([this]() {
          ObservationArray scene;
          scene.harvest_run_id = "run-pz";
          scene.target_set_locked = true;
          scene.selected_target_id = "peach_1";
          scene.observations = {makeObservation()};
          const auto diagnostics = makeDiagnostics();
          while (publishing.load()) {
            target_pub->publish(scene);
            diag_pub->publish(diagnostics);
            std::this_thread::sleep_for(100ms);
          }
        });
  }

  void stop()
  {
    publishing.store(false);
    if (pub_thread.joinable()) {pub_thread.join();}
    executor.cancel();
    if (spinner.joinable()) {spinner.join();}
    action_client.reset();
    diag_pub.reset();
    target_pub.reset();
    client_node.reset();
    node.reset();
  }

  std::shared_ptr<ApproachGraspNode> node;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::Publisher<ObservationArray>::SharedPtr target_pub;
  rclcpp::Publisher<ReconstructionStatus>::SharedPtr diag_pub;
  rclcpp_action::Client<RunTargetCycle>::SharedPtr action_client;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
  std::thread pub_thread;
  std::atomic_bool publishing{false};
};

TEST(ProtectedZoneEntry, EntryInsideZoneRejectsMtcWithUnreachable)
{
  {
    Harness harness;
    harness.start();
    harness.node->configure();
    harness.node->activate();
    ASSERT_EQ(
      harness.node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(harness.action_client->wait_for_action_server(5s));
    // 等订阅连通（发布线程已在跑），保证受理门读到缓存目标。
    std::this_thread::sleep_for(500ms);

    RunTargetCycle::Goal goal;
    goal.request_id = "pz-entry";
    goal.run_id = "run-pz";
    goal.cycle_id = goal.request_id;
    goal.target_id = "peach_1";
    goal.mode = RunTargetCycle::Goal::FULL;
    auto goal_future = harness.action_client->async_send_goal(goal);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        harness.client_node, goal_future, 5s),
      rclcpp::FutureReturnCode::SUCCESS);
    const auto handle = goal_future.get();
    ASSERT_NE(handle, nullptr) << "受理门应放行：selected=peach_1 锚点有效";
    auto result_future = harness.action_client->async_get_result(handle);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        harness.client_node, result_future, 30s),
      rclcpp::FutureReturnCode::SUCCESS);
    const auto wrapped = result_future.get();
    ASSERT_NE(wrapped.result, nullptr);
    // 终局路由：入口在保护区 → 拒规划 → SKIPPED_UNREACHABLE（abort 路径按
    // BT 失败点记录的 pending_outcome_ 分级）；reason 文案锁定失败点就是
    // 保护区剔除而非更早阶段。
    EXPECT_EQ(wrapped.result->outcome, RunTargetCycle::Result::SKIPPED_UNREACHABLE)
      << wrapped.result->reason;
    EXPECT_NE(
      wrapped.result->reason.find("入口在保护区"), std::string::npos)
      << wrapped.result->reason;
    harness.stop();
  }
}

}  // namespace

namespace
{

// 全局 Environment：只 init 不 shutdown（MGI 进程级共享单例在静态析构期仍
// 引用默认 context，先 shutdown 会段错误；同 test_observe_only_gate.cpp）。
class ProtectedZoneEntryTestEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
};

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new ProtectedZoneEntryTestEnvironment);
  return RUN_ALL_TESTS();
}
