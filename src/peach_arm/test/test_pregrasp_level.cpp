#include "peach_arm/acm_policy.hpp"
#include "peach_arm/model_contract.hpp"
#include "peach_arm/plan_contract.hpp"
#include "peach_arm/pregrasp_level.hpp"
#include "peach_arm/retreat_policy.hpp"
#include "peach_arm/tool_txn.hpp"

#include <gtest/gtest.h>

TEST(PregraspLevel, ResidualFailIsReachedNotVerified)
{
  EXPECT_EQ(
    peach_arm::kLevelPregraspReached,
    peach_arm::completion_level_after_pregrasp_verify(false));
}

TEST(PregraspLevel, ResidualPassIsVerified)
{
  EXPECT_EQ(
    peach_arm::kLevelPregraspVerified,
    peach_arm::completion_level_after_pregrasp_verify(true));
}

TEST(PregraspLevel, NotReachedStaysNone)
{
  EXPECT_EQ(
    peach_arm::kLevelNone,
    peach_arm::completion_level_after_pregrasp_verify(true, false, true));
}

TEST(AcmPolicy, WholeOctomapToolExemptionIsF10Fallback)
{
  // F10 真机回退（5fbb8d9）：眼在手上时 octomap updater self-filter 漏收
  // 工具点云，工具×地图幽灵体素自碰死锁（Survey 全灭根因）——整图豁免=
  // true 是现行语义；self-filter 修复后改回 false 并同步本断言
  // （acm_policy.hpp 头注释同源）。
  EXPECT_TRUE(peach_arm::allowToolVersusWholeOctomap());
  EXPECT_FALSE(
    peach_arm::acmAllows(
      "target_1", "sleeve_mouth", peach_arm::ContactAcmStage::Transit));
  EXPECT_TRUE(
    peach_arm::acmAllows(
      "target_1", "sleeve_mouth", peach_arm::ContactAcmStage::Sleeve));
}

TEST(PlanContract, PreviewMustMatchExecute)
{
  peach_arm::ContactPlan preview;
  preview.plan_id = "plan-1";
  preview.scene_epoch = 2;
  preview.model.run_id = "run";
  preview.model.target_id = "t1";
  preview.model.model_revision = "m1";
  preview.model.tool_profile_id = "hollow_cylinder_v1";
  preview.model.calibration_revision = "cal";
  preview.model.config_revision = "cfg";
  preview.start_joints = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  peach_arm::ContactPlan execute = preview;
  EXPECT_TRUE(peach_arm::previewMatchesExecute(preview, execute, 0.05));
  execute.plan_id = "other";
  EXPECT_FALSE(peach_arm::previewMatchesExecute(preview, execute, 0.05));
}

TEST(PlanContract, ObserveBindingSkipsJoints)
{
  peach_arm::ContactPlan observe;
  observe.plan_id = "plan-1";
  observe.scene_epoch = 2;
  observe.require_start_joints = false;
  observe.model.run_id = "run";
  observe.model.target_id = "t1";
  observe.model.model_revision = "m1";
  observe.model.tool_profile_id = "hollow_cylinder_v1";
  observe.model.calibration_revision = "cal";
  observe.model.config_revision = "cfg";
  peach_arm::ContactPlan execute = observe;
  execute.start_joints = {0.2, 0.1, 0.0, 0.3, 0.4, 0.5};
  EXPECT_TRUE(peach_arm::previewMatchesExecute(observe, execute, 0.05));
  execute.model.tool_profile_id = "other";
  EXPECT_FALSE(peach_arm::previewMatchesExecute(observe, execute, 0.05));
}

TEST(RetreatPolicy, PartialUsesActual)
{
  EXPECT_EQ(
    peach_arm::RetreatMode::FromActual,
    peach_arm::sleeveRetreatMode(true));
  EXPECT_EQ(
    peach_arm::RetreatMode::ReverseNominal,
    peach_arm::sleeveRetreatMode(false));
}

TEST(ToolTxn, TimeoutUnknownNoAutoRetreat)
{
  EXPECT_EQ(
    peach_arm::ToolTxnState::Unknown,
    peach_arm::onSetIoTimeout(peach_arm::ToolTxnState::CommandSent));
  EXPECT_FALSE(
    peach_arm::mayAutoRetreatOnTimeout(
      peach_arm::ToolTxnState::Unknown));
  EXPECT_FALSE(peach_arm::mayResendCut(peach_arm::ToolTxnState::Unknown));
  EXPECT_FALSE(peach_arm::harvestConfirmed(true, false));
  EXPECT_TRUE(peach_arm::harvestConfirmed(true, true));
}

TEST(ModelContract, EmptyVersionNotExecutable)
{
  peach_arm::ModelSnapshot snap;
  snap.identity.run_id = "run";
  snap.identity.target_id = "t1";
  snap.generated_s = 1.0;
  snap.valid_until_s = 5.0;
  EXPECT_FALSE(peach_arm::modelExecutable(snap, 2.0, false));
  EXPECT_TRUE(peach_arm::modelExecutable(snap, 2.0, true));
  EXPECT_FALSE(peach_arm::heartbeatRenewsValidity());
}

TEST(ModelContract, ExpiredAndWrongToolRejected)
{
  peach_arm::ModelIdentity left;
  left.run_id = "run";
  left.target_id = "t1";
  left.model_revision = "m1";
  left.tool_profile_id = "a";
  left.calibration_revision = "cal";
  left.config_revision = "cfg";
  peach_arm::ModelIdentity right = left;
  right.tool_profile_id = "b";
  EXPECT_FALSE(peach_arm::identitiesMatch(left, right));
  peach_arm::ModelSnapshot snap;
  snap.identity = left;
  snap.generated_s = 1.0;
  snap.valid_until_s = 2.0;
  EXPECT_FALSE(peach_arm::modelExecutable(snap, 3.0, false));
  EXPECT_TRUE(
    peach_arm::allowedFromCapabilities(
      peach_arm::Capability::Valid,
      peach_arm::Capability::Valid,
      peach_arm::Capability::Valid));
  EXPECT_FALSE(
    peach_arm::allowedFromCapabilities(
      peach_arm::Capability::Valid,
      peach_arm::Capability::Valid,
      peach_arm::Capability::Invalid));
}
