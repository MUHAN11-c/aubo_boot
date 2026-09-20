// 功能：抓取前再确认策略测试（PASS/REFINED/PENDING/ABORT 逐样本判定）。
#include "peach_arm/reconfirm_policy.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

// 逐样本便捷构造：fresh/identity/drift/swing 四要素决定判定。
peach_arm::ReconfirmSample sample(
  bool fresh, bool identity_ok, double drift_m, bool swinging)
{
  peach_arm::ReconfirmSample value;
  value.fresh = fresh;
  value.identity_ok = identity_ok;
  value.anchor_drift_m = drift_m;
  value.swinging = swinging;
  value.anchor = Eigen::Vector3d(0.1, 0.2, 0.5);
  value.axis = Eigen::Vector3d::UnitZ();
  return value;
}

}  // namespace

// ---------- PASS：身份一致且漂移在容差内 ----------

TEST(ReconfirmPolicy, CleanSamplePassesImmediately)
{
  peach_arm::ReconfirmPolicy policy({0.03, 3, false});
  const peach_arm::ReconfirmDecision decision =
    policy.check(sample(true, true, 0.01, false));
  EXPECT_EQ(decision.verdict, peach_arm::ReconfirmVerdict::PASS);
  EXPECT_NE(decision.reason.find("再确认通过"), std::string::npos);
  // 容差压线（恰好 0.03）不算超限。
  peach_arm::ReconfirmPolicy edge({0.03, 3, false});
  EXPECT_EQ(
    edge.check(sample(true, true, 0.03, false)).verdict,
    peach_arm::ReconfirmVerdict::PASS);
}

// ---------- 身份变更：立即 ABORT，不计超限 ----------

TEST(ReconfirmPolicy, IdentityChangeAbortsImmediately)
{
  peach_arm::ReconfirmPolicy policy({0.03, 3, false});
  const peach_arm::ReconfirmDecision decision =
    policy.check(sample(true, false, 0.0, false));
  EXPECT_EQ(decision.verdict, peach_arm::ReconfirmVerdict::ABORT);
  EXPECT_NE(decision.reason.find("目标身份变更"), std::string::npos);
}

// ---------- 漂移超限：REFINED，累计达 max_attempts 转 ABORT ----------

TEST(ReconfirmPolicy, DriftOverToleranceRefinesThenAborts)
{
  peach_arm::ReconfirmPolicy policy({0.03, 3, false});
  EXPECT_EQ(
    policy.check(sample(true, true, 0.05, false)).verdict,
    peach_arm::ReconfirmVerdict::REFINED);
  EXPECT_EQ(
    policy.check(sample(true, true, 0.06, false)).verdict,
    peach_arm::ReconfirmVerdict::REFINED);
  // 第 3 次超限 → ABORT，直报锚点漂移。
  const peach_arm::ReconfirmDecision abort = policy.check(
    sample(true, true, 0.05, false));
  EXPECT_EQ(abort.verdict, peach_arm::ReconfirmVerdict::ABORT);
  EXPECT_NE(abort.reason.find("锚点漂移超限"), std::string::npos);
  EXPECT_EQ(abort.reason.find("摆动"), std::string::npos);
}

TEST(ReconfirmPolicy, DriftAfterSwingReportsPersistentSwing)
{
  // 曾见摆动旗标后漂移累计到顶：致因文案冠「目标持续摆动」。
  peach_arm::ReconfirmPolicy policy({0.03, 3, false});
  EXPECT_EQ(
    policy.check(sample(true, true, 0.01, true)).verdict,
    peach_arm::ReconfirmVerdict::PENDING);
  EXPECT_EQ(
    policy.check(sample(true, true, 0.05, false)).verdict,
    peach_arm::ReconfirmVerdict::REFINED);
  EXPECT_EQ(
    policy.check(sample(true, true, 0.05, false)).verdict,
    peach_arm::ReconfirmVerdict::REFINED);
  const peach_arm::ReconfirmDecision abort = policy.check(
    sample(true, true, 0.05, false));
  EXPECT_EQ(abort.verdict, peach_arm::ReconfirmVerdict::ABORT);
  EXPECT_NE(abort.reason.find("目标持续摆动"), std::string::npos);
}

// ---------- 摆动：PENDING 等平息，连续 2 帧干净才 PASS ----------

TEST(ReconfirmPolicy, SwingingWaitsForTwoCalmFrames)
{
  peach_arm::ReconfirmPolicy policy({0.03, 3, false});
  // 摆动中：PENDING（不计超限）。
  EXPECT_EQ(
    policy.check(sample(true, true, 0.01, true)).verdict,
    peach_arm::ReconfirmVerdict::PENDING);
  // 首帧干净：还要再等一帧确认平息。
  const peach_arm::ReconfirmDecision first_calm =
    policy.check(sample(true, true, 0.01, false));
  EXPECT_EQ(first_calm.verdict, peach_arm::ReconfirmVerdict::PENDING);
  EXPECT_NE(first_calm.reason.find("摆动后首帧干净"), std::string::npos);
  // 第二帧干净：PASS。
  EXPECT_EQ(
    policy.check(sample(true, true, 0.01, false)).verdict,
    peach_arm::ReconfirmVerdict::PASS);
}

TEST(ReconfirmPolicy, DriftResetsCalmFrameCounter)
{
  // 平息确认途中又漂移超限：冷静帧清零，REFINED 后仍需重新攒 2 帧干净。
  peach_arm::ReconfirmPolicy policy({0.03, 3, false});
  EXPECT_EQ(
    policy.check(sample(true, true, 0.01, true)).verdict,
    peach_arm::ReconfirmVerdict::PENDING);
  EXPECT_EQ(
    policy.check(sample(true, true, 0.01, false)).verdict,
    peach_arm::ReconfirmVerdict::PENDING);
  EXPECT_EQ(
    policy.check(sample(true, true, 0.05, false)).verdict,
    peach_arm::ReconfirmVerdict::REFINED);
  EXPECT_EQ(
    policy.check(sample(true, true, 0.01, false)).verdict,
    peach_arm::ReconfirmVerdict::PENDING);
  EXPECT_EQ(
    policy.check(sample(true, true, 0.01, false)).verdict,
    peach_arm::ReconfirmVerdict::PASS);
}

// ---------- 窗口耗尽：计次 PENDING，耗尽满额 ABORT；回退开关放行 ----------

TEST(ReconfirmPolicy, ExhaustedWindowsCountStrikesThenAbort)
{
  peach_arm::ReconfirmPolicy policy({0.03, 3, false});
  const peach_arm::ReconfirmDecision first =
    policy.check(peach_arm::ReconfirmSample::exhausted());
  EXPECT_EQ(first.verdict, peach_arm::ReconfirmVerdict::PENDING);
  EXPECT_NE(first.reason.find("窗口耗尽"), std::string::npos);
  EXPECT_EQ(
    policy.check(peach_arm::ReconfirmSample::exhausted()).verdict,
    peach_arm::ReconfirmVerdict::PENDING);
  const peach_arm::ReconfirmDecision abort =
    policy.check(peach_arm::ReconfirmSample::exhausted());
  EXPECT_EQ(abort.verdict, peach_arm::ReconfirmVerdict::ABORT);
  EXPECT_NE(abort.reason.find("未获得新鲜观测"), std::string::npos);
  EXPECT_NE(abort.reason.find("累计 3 次"), std::string::npos);
}

TEST(ReconfirmPolicy, StaleAnchorFallbackPassesOnExhausted)
{
  // allow_stale_anchor=true（验证期遗留回退开关）：窗口耗尽按静态锚点放行。
  peach_arm::ReconfirmPolicy policy({0.03, 3, true});
  const peach_arm::ReconfirmDecision decision =
    policy.check(peach_arm::ReconfirmSample::exhausted());
  EXPECT_EQ(decision.verdict, peach_arm::ReconfirmVerdict::PASS);
  EXPECT_NE(decision.reason.find("allow_stale_anchor"), std::string::npos);
}
