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

namespace
{

BT::BehaviorTreeFactory makeFactory(bool plan_only, bool grasp_disabled)
{
  BT::BehaviorTreeFactory factory;
  const auto success = [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;};
  factory.registerSimpleAction("PrepareCycle", success);
  factory.registerSimpleCondition(
    "IsPlanOnly", [plan_only](BT::TreeNode &) {
      return plan_only ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleAction("PlanObservationPreview", success);
  factory.registerSimpleAction("AcquireReconstructionViews", success);
  factory.registerSimpleAction("FinalizeAndValidate", success);
  factory.registerSimpleCondition(
    "IsGraspDisabled", [grasp_disabled](BT::TreeNode &) {
      return grasp_disabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleAction("ReportReadyForGrasp", success);
  factory.registerSimpleAction("MTCApproachAndInsert", success);
  factory.registerSimpleAction("ActuateTool", success);
  factory.registerSimpleAction("MTCRetreat", success);
  factory.registerSimpleAction("CompleteTarget", success);
  return factory;
}

TEST(BehaviorTree, PlanOnlyBranchParsesAndSucceeds)
{
  auto factory = makeFactory(true, true);
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(BehaviorTree, ActiveGraspBranchParsesAndSucceeds)
{
  auto factory = makeFactory(false, false);
  auto tree = factory.createTreeFromFile(HARVEST_TREE_XML);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

}  // namespace
