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
// 观察段扫描预算（2.13-E2）纯核用例：下限保证、质量收敛提前收口、预算
// 墙钟到期与移动成本 EMA 预测性收口、移动次数兜底、运行期预算伸缩。
#include <gtest/gtest.h>

#include "peach_approach_grasp/scan_budget.hpp"

namespace
{
using peach_approach_grasp::ScanBudget;
using peach_approach_grasp::ScanBudgetConfig;
using peach_approach_grasp::ScanVerdict;

ScanBudget makeBudget(int max_moves = 5, int min_views = 2, double budget_s = 5.0)
{
  return ScanBudget(ScanBudgetConfig{max_moves, min_views, budget_s});
}

TEST(ScanBudget, LowerBoundOverridesConvergence)
{
  // 下限保证（2.13-E2）：质量门已放行但有效视点不足下限 → 继续扫描，
  // 不得提前收口（防"没看够就 finalize"）。
  const auto budget = makeBudget();
  EXPECT_EQ(
    budget.poll(true, 1, 1, 1.0, 0.0), ScanVerdict::CONTINUE);
  EXPECT_EQ(
    budget.poll(true, 0, 0, 0.0, 0.0), ScanVerdict::CONTINUE);
}

TEST(ScanBudget, ConvergedEarlyExitAfterLowerBound)
{
  // 达到下限且质量收敛（覆盖/基线达标）→ 提前收口。
  const auto budget = makeBudget();
  EXPECT_EQ(budget.poll(true, 2, 2, 1.0, 0.0), ScanVerdict::CONVERGED);
  // 未达下限即使预算所剩无几也继续（下限保证优先于预算）。
  EXPECT_EQ(budget.poll(false, 4, 1, 4.9, 0.0), ScanVerdict::CONTINUE);
}

TEST(ScanBudget, BudgetWallClockExpiry)
{
  // 达到下限后墙钟预算到期 → 预算收口（强制 finalize 走降级链）。
  const auto budget = makeBudget();
  EXPECT_EQ(
    budget.poll(false, 2, 2, 5.0, 0.0), ScanVerdict::BUDGET_EXHAUSTED);
  EXPECT_EQ(
    budget.poll(false, 2, 2, 4.0, 0.0), ScanVerdict::CONTINUE);
}

TEST(ScanBudget, PredictiveCutByMoveCostEma)
{
  // 移动成本 EMA 预测：剩余预算换不起一个有效视点 → 提前收口。
  // 预算 5s，已耗 3.5s，实测成本 2.0s → 剩余 1.5s < 2.0s，收口。
  const auto budget = makeBudget();
  EXPECT_EQ(
    budget.poll(false, 2, 2, 3.5, 2.0), ScanVerdict::BUDGET_EXHAUSTED);
  // 剩余 2.5s ≥ 成本 2.0s → 还能再换一个视点，继续。
  EXPECT_EQ(budget.poll(false, 2, 2, 2.5, 2.0), ScanVerdict::CONTINUE);
}

TEST(ScanBudget, AdaptiveBudgetStretchesWithCostEma)
{
  // 运行期预算 = max(配置下限, 2.5×成本EMA)（协议 2.7-OBSERVE T(scan_budget)）：
  // 成本 4s 时预算 10s，墙钟 7s 未到期；EMA 未测得（≤0）回退配置下限。
  const auto budget = makeBudget(5, 2, 5.0);
  EXPECT_DOUBLE_EQ(budget.effectiveBudgetS(4.0), 10.0);
  EXPECT_DOUBLE_EQ(budget.effectiveBudgetS(1.0), 5.0);
  EXPECT_DOUBLE_EQ(budget.effectiveBudgetS(0.0), 5.0);
  // 成本 4s：已耗 7s 时剩余 3s < 4s → 预测性收口；已耗 5.5s 时剩余 4.5s → 继续。
  EXPECT_EQ(
    budget.poll(false, 2, 2, 7.0, 4.0), ScanVerdict::BUDGET_EXHAUSTED);
  EXPECT_EQ(budget.poll(false, 2, 2, 5.5, 4.0), ScanVerdict::CONTINUE);
}

TEST(ScanBudget, MovesCapIsLastResort)
{
  // 移动次数上限耗尽 → MOVES_EXHAUSTED（含有效视点未达下限的场景：
  // 下限保证不越过次数硬上限，防候选全失败死循环）。
  const auto budget = makeBudget(5, 2, 5.0);
  EXPECT_EQ(budget.poll(false, 5, 0, 1.0, 0.0), ScanVerdict::MOVES_EXHAUSTED);
  EXPECT_EQ(budget.poll(true, 5, 1, 1.0, 0.0), ScanVerdict::MOVES_EXHAUSTED);
}

}  // namespace
