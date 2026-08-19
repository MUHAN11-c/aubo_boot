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
// 可替换接口契约测试（重构协议 2.14）：每个抽象基类一个 fake 实现，模拟
// 调用端（bt_nodes / 节点服务回调）只经基类指针驱动；并钉死工厂语义——
// 按注册名创建、未知名抛 std::invalid_argument 且信息列出可用名。

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_factory.hpp"
#include "peach_approach_grasp/impl_factory.hpp"
#include "peach_approach_grasp/motion_interface_base.hpp"

namespace peach_approach_grasp
{
namespace
{

// 视点规划 fake：回显固定候选并记录所见输入，验证调用端只依赖
// ViewPlannerBase::generate(ViewContext)。
class FakeViewPlanner : public ViewPlannerBase
{
public:
  std::vector<ViewCandidate> generate(const ViewContext & context) const override
  {
    last_context = context;
    ++calls;
    ViewCandidate candidate;
    candidate.label = "fake_view";
    candidate.score = 1.0;
    return {candidate};
  }

  mutable ViewContext last_context;
  mutable int calls{0};
};

// 质量门 fake：三个判定返回脚本化结果，记录调用次数。
class FakeQualityGate : public QualityGateBase
{
public:
  GateResult readyToFinalize(const QualitySnapshot &) const override
  {
    ++finalize_calls;
    return GateResult{true, "fake_finalize_ok"};
  }

  GateResult readyToPreviewContact(const QualitySnapshot &) const override
  {
    return GateResult{false, "fake_preview_block"};
  }

  GateResult readyToGrasp(const QualitySnapshot &) const override
  {
    return GateResult{grasp_allowed, "fake_grasp"};
  }

  bool grasp_allowed{true};
  mutable int finalize_calls{0};
};

// 安全门 fake：脚本化放行结果，记录新鲜度上限调整。注意：真实部署的任何
// 实现都不得旁路硬件安全门（I5）；本 fake 仅用于编译期/调用面契约验证。
class FakeSafetyGate : public SafetyGateBase
{
public:
  bool robotReady(const RobotStatusSample &, std::string & reason) const override
  {
    ++robot_calls;
    reason = robot_reason;
    return robot_ready;
  }

  bool targetReady(
    const TargetGateSample &, const std::string &, std::string & reason) const override
  {
    reason = target_reason;
    return target_ready;
  }

  void set_target_observation_max_age_s(double value) override
  {
    last_max_age_s = value;
  }

  bool robot_ready{false};
  bool target_ready{true};
  std::string robot_reason{"fake_robot_block"};
  std::string target_reason{"fake_target_ok"};
  double last_max_age_s{-1.0};
  mutable int robot_calls{0};
};

// 运动接口 fake：纯数据回显，验证调用端只依赖 MotionInterfaceBase 最小集。
class FakeMotionInterface : public MotionInterfaceBase
{
public:
  std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target, const std::string & source) override
  {
    last_target_frame = target;
    last_source_frame = source;
    return Eigen::Isometry3d::Identity();
  }

  bool planOrMoveTip(
    const Eigen::Isometry3d &, const std::string & planner_id, bool execute,
    const std::string &, bool) override
  {
    last_planner_id = planner_id;
    last_execute = execute;
    return plan_result;
  }

  bool planOrMoveCamera(
    const Eigen::Isometry3d &, const std::string & planner_id, bool execute,
    const std::string &, bool) override
  {
    last_planner_id = planner_id;
    last_execute = execute;
    ++camera_calls;
    return plan_result;
  }

  bool planOrMoveTool(
    const Eigen::Isometry3d &, const std::string &, const std::string &) override
  {
    ++tool_calls;
    return plan_result;
  }

  bool goToPhotoPose(
    const std::string & named_target, bool execute, std::string & message) override
  {
    last_named_target = named_target;
    last_execute = execute;
    message = "fake_photo_pose";
    return true;
  }

  bool plan_result{true};
  int camera_calls{0};
  int tool_calls{0};
  bool last_execute{true};
  std::string last_planner_id;
  std::string last_target_frame;
  std::string last_source_frame;
  std::string last_named_target;
};

// 以下 helper 模拟真实调用面：签名只出现基类引用/指针，证明调用端不依赖
// 具体实现类型（与 bt_nodes.cpp / 节点服务回调的用法一一对应）。

// bt_nodes btPrepareCycle：视点规划候选生成。
std::vector<ViewCandidate> planCycleViews(
  const ViewPlannerBase & planner, const ViewContext & context)
{
  return planner.generate(context);
}

// bt_nodes btFinalizeAndValidate：质量门抓取判定。
GateResult graspGateDecision(const QualityGateBase & gate, const QualitySnapshot & snapshot)
{
  return gate.readyToGrasp(snapshot);
}

// 节点 safetyReady 薄壳：硬件安全门判定。
bool hardwareGateAllows(const SafetyGateBase & gate, std::string & reason)
{
  return gate.robotReady(RobotStatusSample{}, reason);
}

// bt_nodes btPlanPreview：TF 查询 + 只规划。
bool previewFirstView(MotionInterfaceBase & motion)
{
  const auto tf = motion.lookupTransform("base_link", "camera_link");
  if (!tf) {
    return false;
  }
  return motion.planOrMoveCamera(
    Eigen::Isometry3d::Identity(), "PTP", false, "fake_view", true);
}

}  // namespace

TEST(ImplContract, CallerDrivesViewPlannerThroughBaseOnly)
{
  FakeViewPlanner fake;
  const ViewPlannerBase & base = fake;
  ViewContext context;
  context.target = Eigen::Vector3d(0.4, 0.0, 0.3);
  context.observed_directions = {Eigen::Vector3d::UnitX()};
  const auto candidates = planCycleViews(base, context);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().label, "fake_view");
  EXPECT_EQ(fake.calls, 1);
  EXPECT_TRUE(fake.last_context.target.isApprox(context.target));
}

TEST(ImplContract, CallerDrivesQualityGateThroughBaseOnly)
{
  FakeQualityGate fake;
  const QualityGateBase & base = fake;
  EXPECT_TRUE(base.readyToFinalize(QualitySnapshot{}).allowed);
  EXPECT_FALSE(base.readyToPreviewContact(QualitySnapshot{}).allowed);
  EXPECT_TRUE(base.readyToGrasp(QualitySnapshot{}).allowed);
  fake.grasp_allowed = false;
  const GateResult result = graspGateDecision(base, QualitySnapshot{});
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.reason, "fake_grasp");
  EXPECT_EQ(fake.finalize_calls, 1);
}

TEST(ImplContract, CallerDrivesSafetyGateThroughBaseOnly)
{
  FakeSafetyGate fake;
  SafetyGateBase & base = fake;
  std::string reason;
  EXPECT_FALSE(hardwareGateAllows(base, reason));
  EXPECT_EQ(reason, "fake_robot_block");
  EXPECT_TRUE(base.targetReady(TargetGateSample{}, "t1", reason));
  base.set_target_observation_max_age_s(4.5);
  EXPECT_DOUBLE_EQ(fake.last_max_age_s, 4.5);
  EXPECT_EQ(fake.robot_calls, 1);
}

TEST(ImplContract, CallerDrivesMotionInterfaceThroughBaseOnly)
{
  FakeMotionInterface fake;
  MotionInterfaceBase & base = fake;
  EXPECT_TRUE(previewFirstView(base));
  EXPECT_EQ(fake.last_planner_id, "PTP");
  EXPECT_FALSE(fake.last_execute);  // 只规划预览不得下发执行
  EXPECT_EQ(fake.last_target_frame, "base_link");
  std::string message;
  EXPECT_TRUE(base.goToPhotoPose("global_photo_pose", false, message));
  EXPECT_EQ(fake.last_named_target, "global_photo_pose");
  EXPECT_EQ(message, "fake_photo_pose");
}

TEST(ImplFactory, CreatesKnownCoreImplementationsByName)
{
  std::unique_ptr<ViewPlannerBase> planner =
    createViewPlanner("spherical_adaptive", ViewPlannerConfig{});
  ASSERT_NE(planner, nullptr);
  EXPECT_NE(dynamic_cast<ViewPlanner *>(planner.get()), nullptr);

  std::unique_ptr<QualityGateBase> quality =
    createQualityGate("threshold", QualityGateConfig{});
  ASSERT_NE(quality, nullptr);
  EXPECT_NE(dynamic_cast<QualityGate *>(quality.get()), nullptr);

  std::unique_ptr<SafetyGateBase> safety =
    createSafetyGate("robot_status_gate", SafetyGateConfig{}, []() {return 0.0;});
  ASSERT_NE(safety, nullptr);
  EXPECT_NE(dynamic_cast<SafetyGate *>(safety.get()), nullptr);
}

TEST(ImplFactory, UnknownNamesThrowListingAvailableNames)
{
  try {
    createViewPlanner("does_not_exist", ViewPlannerConfig{});
    FAIL() << "未知 view_planner.impl 必须抛 std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("spherical_adaptive"), std::string::npos);
  }
  try {
    createQualityGate("does_not_exist", QualityGateConfig{});
    FAIL() << "未知 quality_gate.impl 必须抛 std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("threshold"), std::string::npos);
  }
  try {
    createSafetyGate("does_not_exist", SafetyGateConfig{}, []() {return 0.0;});
    FAIL() << "未知 safety_gate.impl 必须抛 std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("robot_status_gate"), std::string::npos);
  }
}

TEST(ImplFactory, UnknownMotionNameThrowsListingAvailableNames)
{
  // 未知名在触碰 MoveIt/TF 指针之前即抛错，故可传空指针与空回调。
  try {
    createMotionInterface(
      "does_not_exist", nullptr, nullptr, rclcpp::get_logger("test_impl_contract"),
      std::make_shared<rclcpp::Clock>(), MoveItMotionConfig{}, nullptr, nullptr);
    FAIL() << "未知 motion.impl 必须抛 std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("moveit_motion"), std::string::npos);
  }
}

}  // namespace peach_approach_grasp
