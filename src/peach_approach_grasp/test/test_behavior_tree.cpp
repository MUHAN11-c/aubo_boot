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
#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace
{

// 节点体 stub 状态：flags 控制三条短路条件分支与再确认成败；calls 按调用顺序
// 记录动作节点名（验证 SubTree 拆分后阶段顺序与 ReconfirmTarget 插入位置不变）。
// 经 shared_ptr 注入工厂：lambda 捕获共享所有权，避免悬引用（临时值入参是 UB）。
struct Stubs
{
  bool plan_only{false};
  bool grasp_disabled{false};
  bool observe_only{false};
  bool reconfirm_ok{true};
  std::vector<std::string> calls;
};

BT::BehaviorTreeFactory makeFactory(const std::shared_ptr<Stubs> & stubs)
{
  BT::BehaviorTreeFactory factory;
  const auto record = [stubs](const char * name) {
      return [stubs, name](BT::TreeNode &) {
               stubs->calls.push_back(name);
               return BT::NodeStatus::SUCCESS;
             };
    };
  factory.registerSimpleAction("PrepareCycle", record("PrepareCycle"));
  factory.registerSimpleCondition(
    "IsPlanOnly", [stubs](BT::TreeNode &) {
      return stubs->plan_only ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleAction(
    "PlanObservationPreview", record("PlanObservationPreview"));
  factory.registerSimpleAction(
    "AcquireReconstructionViews", record("AcquireReconstructionViews"));
  factory.registerSimpleAction(
    "FinalizeAndValidate", record("FinalizeAndValidate"));
  factory.registerSimpleCondition(
    "IsObserveOnly", [stubs](BT::TreeNode &) {
      return stubs->observe_only ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleAction("ReportObserveOnly", record("ReportObserveOnly"));
  factory.registerSimpleCondition(
    "IsGraspDisabled", [stubs](BT::TreeNode &) {
      return stubs->grasp_disabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleAction(
    "ReportReadyForGrasp", record("ReportReadyForGrasp"));
  // 抓取前再确认（2.7-RECONFIRM）：FinalizeAndValidate 之后、MTC 之前的验证关；
  // reconfirm_ok=false 模拟放弃（SKIPPED_QUALITY 路径），接触段必须被阻断。
  factory.registerSimpleAction(
    "ReconfirmTarget", [stubs](BT::TreeNode &) {
      stubs->calls.push_back("ReconfirmTarget");
      return stubs->reconfirm_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleAction(
    "MTCApproachAndInsert", record("MTCApproachAndInsert"));
  factory.registerSimpleAction("ActuateTool", record("ActuateTool"));
  factory.registerSimpleAction("MTCRetreat", record("MTCRetreat"));
  factory.registerSimpleAction("CompleteTarget", record("CompleteTarget"));
  return factory;
}

TEST(BehaviorTree, PlanOnlyBranchParsesAndSucceeds)
{
  auto factory = makeFactory(std::make_shared<Stubs>(Stubs{true, true, false}));
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(BehaviorTree, ActiveGraspBranchParsesAndSucceeds)
{
  auto factory = makeFactory(std::make_shared<Stubs>(Stubs{false, false, false}));
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(BehaviorTree, ObserveOnlyBranchParsesAndSucceeds)
{
  // OBSERVE_ONLY 周期：FinalizeAndValidate 后经 IsObserveOnly 短路，
  // 不进入 ReportReadyForGrasp / 再确认 / MTC / 工具 / 撤离段。
  const auto stubs = std::make_shared<Stubs>(Stubs{false, false, true});
  auto factory = makeFactory(stubs);
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    stubs->calls,
    (std::vector<std::string>{
      "PrepareCycle", "AcquireReconstructionViews", "FinalizeAndValidate",
      "ReportObserveOnly"}));
}

TEST(BehaviorTree, ObserveOnlyTakesPrecedenceOverGraspDisabled)
{
  // observe_only 与 grasp.disabled 同时成立时仍走 observe-only 短路分支；
  // 安全边界钉死：即使 grasp 已使能（grasp_disabled=false），OBSERVE_ONLY
  // 也绝不进入再确认/MTC/工具/撤离段（接触段节点一个都不许被调用）。
  const auto stubs = std::make_shared<Stubs>(Stubs{false, true, true});
  auto factory = makeFactory(stubs);
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    stubs->calls,
    (std::vector<std::string>{
      "PrepareCycle", "AcquireReconstructionViews", "FinalizeAndValidate",
      "ReportObserveOnly"}));
}

TEST(BehaviorTree, ObserveOnlyNeverReachesContactStagesWhenGraspEnabled)
{
  // 同上但 grasp 使能（grasp_disabled=false）：Fallback 第三支路（靠近抓取→
  // 工具→撤离）整段不得触发——IsObserveOnly 优先级高于 IsGraspDisabled。
  const auto stubs = std::make_shared<Stubs>(Stubs{false, false, true});
  auto factory = makeFactory(stubs);
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    stubs->calls,
    (std::vector<std::string>{
      "PrepareCycle", "AcquireReconstructionViews", "FinalizeAndValidate",
      "ReportObserveOnly"}));
  EXPECT_EQ(
    std::find(stubs->calls.begin(), stubs->calls.end(), "ReconfirmTarget"),
    stubs->calls.end());
  EXPECT_EQ(
    std::find(stubs->calls.begin(), stubs->calls.end(), "MTCApproachAndInsert"),
    stubs->calls.end());
  EXPECT_EQ(
    std::find(stubs->calls.begin(), stubs->calls.end(), "ActuateTool"),
    stubs->calls.end());
  EXPECT_EQ(
    std::find(stubs->calls.begin(), stubs->calls.end(), "MTCRetreat"),
    stubs->calls.end());
}

TEST(BehaviorTree, ReconfirmRunsAfterFinalizeBeforeMtc)
{
  // 完整抓取支路的阶段顺序钉死（SubTree 拆分不改变行为）：
  // …→FinalizeAndValidate→ReconfirmTarget→MTCApproachAndInsert→工具→撤离→完成。
  const auto stubs = std::make_shared<Stubs>(Stubs{false, false, false});
  auto factory = makeFactory(stubs);
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    stubs->calls,
    (std::vector<std::string>{
      "PrepareCycle", "AcquireReconstructionViews", "FinalizeAndValidate",
      "ReconfirmTarget", "MTCApproachAndInsert", "ActuateTool", "MTCRetreat",
      "CompleteTarget"}));
}

TEST(BehaviorTree, ReconfirmFailureBlocksContactStages)
{
  // 再确认放弃（SKIPPED_QUALITY）：MTC/工具/撤离/完成一律不得执行。
  const auto stubs = std::make_shared<Stubs>(Stubs{false, false, false, false});
  auto factory = makeFactory(stubs);
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    stubs->calls,
    (std::vector<std::string>{
      "PrepareCycle", "AcquireReconstructionViews", "FinalizeAndValidate",
      "ReconfirmTarget"}));
}

}  // namespace
