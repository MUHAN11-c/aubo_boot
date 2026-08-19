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
// 抓取前再确认纯核策略（2.7-RECONFIRM）用例：漂移容差内 PASS、超限 REFINED
// 后按新锚点复核 PASS、窗口耗尽累计到上限 ABORT（SKIPPED_QUALITY 由节点体
// 落地）、摆动等平息（连续 2 帧干净才放行）、allow_stale_anchor 退化放行、
// 身份变更立即 ABORT。等待窗口/计时在 BT 节点体，本文件只测逐样本判定。
#include <gtest/gtest.h>

#include <string>

#include "peach_approach_grasp/reconfirm_policy.hpp"

namespace peach_approach_grasp
{
namespace
{

ReconfirmConfig testConfig()
{
  // 容差 3cm、上限 3 次、不允许静态锚点回退（与 config/approach_grasp.yaml 一致）。
  return ReconfirmConfig{0.03, 3, false};
}

ReconfirmSample freshSample(double drift_m, bool swinging = false)
{
  ReconfirmSample sample;
  sample.fresh = true;
  sample.identity_ok = true;
  sample.swinging = swinging;
  sample.anchor_drift_m = drift_m;
  sample.anchor = Eigen::Vector3d(drift_m, 0.0, 0.0);
  sample.axis = Eigen::Vector3d::UnitZ();
  return sample;
}

TEST(ReconfirmPolicy, DriftWithinTolerancePasses)
{
  ReconfirmPolicy policy(testConfig());
  const auto decision = policy.check(freshSample(0.02));
  EXPECT_EQ(decision.verdict, ReconfirmVerdict::PASS);
  EXPECT_EQ(policy.strikes(), 0);
}

TEST(ReconfirmPolicy, DriftExceedOnceRefinesThenPasses)
{
  // 漂移超限 → REFINED（节点体用最新锚点重算 entry/axis 一次）并计一次超限；
  // 下一轮窗口按新锚点复核（漂移归零）→ PASS。
  ReconfirmPolicy policy(testConfig());
  const auto first = policy.check(freshSample(0.05));
  EXPECT_EQ(first.verdict, ReconfirmVerdict::REFINED);
  EXPECT_NE(first.reason.find("锚点漂移"), std::string::npos);
  EXPECT_EQ(policy.strikes(), 1);
  const auto second = policy.check(freshSample(0.01));
  EXPECT_EQ(second.verdict, ReconfirmVerdict::PASS);
}

TEST(ReconfirmPolicy, ExhaustedWindowsAccumulateToAbort)
{
  // 窗口耗尽（未等到新鲜观测）每次计一次超限；累计达上限 → ABORT，
  // reason 含"窗口耗尽"。
  ReconfirmPolicy policy(testConfig());
  auto decision = policy.check(ReconfirmSample::exhausted());
  EXPECT_EQ(decision.verdict, ReconfirmVerdict::PENDING);
  decision = policy.check(ReconfirmSample::exhausted());
  EXPECT_EQ(decision.verdict, ReconfirmVerdict::PENDING);
  EXPECT_EQ(policy.strikes(), 2);
  decision = policy.check(ReconfirmSample::exhausted());
  EXPECT_EQ(decision.verdict, ReconfirmVerdict::ABORT);
  EXPECT_NE(decision.reason.find("窗口耗尽"), std::string::npos);
}

TEST(ReconfirmPolicy, DriftStrikesAccumulateToAbort)
{
  // 漂移持续超限累计到上限 → ABORT，reason 含"锚点漂移超限"。
  ReconfirmPolicy policy(testConfig());
  EXPECT_EQ(policy.check(freshSample(0.05)).verdict, ReconfirmVerdict::REFINED);
  EXPECT_EQ(policy.check(freshSample(0.06)).verdict, ReconfirmVerdict::REFINED);
  const auto decision = policy.check(freshSample(0.04));
  EXPECT_EQ(decision.verdict, ReconfirmVerdict::ABORT);
  EXPECT_NE(decision.reason.find("锚点漂移超限"), std::string::npos);
}

TEST(ReconfirmPolicy, SwingingWaitsForTwoCalmFrames)
{
  // 摆动帧 PENDING 等平息（本身不计超限）；摆动后首帧干净仍 PENDING，
  // 连续第 2 帧干净才 PASS。
  ReconfirmPolicy policy(testConfig());
  EXPECT_EQ(
    policy.check(freshSample(0.01, true)).verdict, ReconfirmVerdict::PENDING);
  EXPECT_EQ(policy.strikes(), 0);
  EXPECT_EQ(
    policy.check(freshSample(0.01, false)).verdict, ReconfirmVerdict::PENDING);
  EXPECT_EQ(
    policy.check(freshSample(0.01, false)).verdict, ReconfirmVerdict::PASS);
}

TEST(ReconfirmPolicy, PersistentSwingingExhaustsWindowToAbort)
{
  // 摆动不息的现实路径：摆动帧 PENDING 不计超限，但摆动持续导致窗口耗尽，
  // 耗尽累计到上限 → ABORT 且 reason 含"目标持续摆动"。
  ReconfirmPolicy policy(testConfig());
  EXPECT_EQ(
    policy.check(freshSample(0.01, true)).verdict, ReconfirmVerdict::PENDING);
  policy.check(ReconfirmSample::exhausted());
  policy.check(ReconfirmSample::exhausted());
  const auto decision = policy.check(ReconfirmSample::exhausted());
  EXPECT_EQ(decision.verdict, ReconfirmVerdict::ABORT);
  EXPECT_NE(decision.reason.find("目标持续摆动"), std::string::npos);
}

TEST(ReconfirmPolicy, AllowStaleAnchorPassesOnExhaustedWindow)
{
  // 回退开关（验证期遗留）：allow_stale_anchor=true 时窗口耗尽按静态锚点放行。
  ReconfirmPolicy policy(ReconfirmConfig{0.03, 3, true});
  const auto decision = policy.check(ReconfirmSample::exhausted());
  EXPECT_EQ(decision.verdict, ReconfirmVerdict::PASS);
  EXPECT_NE(decision.reason.find("allow_stale_anchor"), std::string::npos);
}

TEST(ReconfirmPolicy, IdentityMismatchAbortsImmediately)
{
  // 身份不一致立即 ABORT（不计超限，由周期钉死语义兜底）。
  ReconfirmPolicy policy(testConfig());
  ReconfirmSample sample = freshSample(0.0);
  sample.identity_ok = false;
  const auto decision = policy.check(sample);
  EXPECT_EQ(decision.verdict, ReconfirmVerdict::ABORT);
  EXPECT_NE(decision.reason.find("身份变更"), std::string::npos);
  EXPECT_EQ(policy.strikes(), 0);
}

}  // namespace
}  // namespace peach_approach_grasp
