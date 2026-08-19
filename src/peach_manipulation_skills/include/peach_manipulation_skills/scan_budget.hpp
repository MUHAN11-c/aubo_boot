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
#ifndef PEACH_MANIPULATION_SKILLS__SCAN_BUDGET_HPP_
#define PEACH_MANIPULATION_SKILLS__SCAN_BUDGET_HPP_

#include <algorithm>

namespace peach_manipulation_skills
{

// 观察段扫描预算（2.13-E2 / 2.7-OBSERVE）纯核：从"固定 maximum_moves 计数 +
// 固定时间盒"升级为预算制——
//   * 下限保证：有效视点观测（移动到位且收到新鲜目标观测）未达
//     min_effective_views 前，质量收敛与预算耗尽都不得收口；
//   * 提前收口：达到下限且质量门允许 finalize（覆盖/基线达标）即停；
//   * 预算自适应：运行预算 = max(配置下限 time_budget_s,
//     2.5 × 实测移动+等帧成本 EMA)（协议 T(scan_budget)）；移动成本 EMA 已测
//     得时，剩余预算换不起一个视点则预测性收口，提前 finalize 走降级链；
//   * 移动次数上限 maximum_moves 仍兜底（候选规划/移动失败的极端场景）。
// 纯核零 ROS、零阻塞、零时钟依赖：耗时/EMA 由调用方注入，可单测。

struct ScanBudgetConfig
{
  // 默认值以 config/approach_grasp.yaml 为权威源，此处仅为直接构造兜底。
  int maximum_moves{5};        // 移动次数硬上限（兜底，防候选全失败死循环）
  int min_effective_views{2};  // 有效视点观测下限（收口前提）
  double time_budget_s{5.0};  // 观察段时间预算下限（秒），运行期按成本 EMA 伸缩
};

enum class ScanVerdict
{
  CONTINUE,          // 继续扫描（下限未达 / 预算仍够换一个视点）
  CONVERGED,         // 质量收敛且下限已达：提前收口 finalize
  BUDGET_EXHAUSTED,  // 预算耗尽（墙钟到期或预测剩余换不起一个视点）：强制 finalize
  MOVES_EXHAUSTED    // 移动次数上限耗尽
};

class ScanBudget
{
public:
  explicit ScanBudget(ScanBudgetConfig config = ScanBudgetConfig())
  : config_(config)
  {
  }

  // 运行期有效预算（秒）：配置值为下限；移动+等帧成本 EMA 测得后按
  // 2.5× 伸缩（协议 2.7-OBSERVE 的 T(scan_budget)），ema≤0 表示未测得。
  double effectiveBudgetS(double move_cost_ema_s) const
  {
    if (move_cost_ema_s <= 0.0) {
      return config_.time_budget_s;
    }
    return std::max(config_.time_budget_s, 2.5 * move_cost_ema_s);
  }

  // 每轮扫描循环顶部的收口判定。
  //   gate_allowed     ：质量门 readyToFinalize 当前是否放行（覆盖/基线达标）；
  //   moves/effective_views：已执行移动次数 / 有效视点观测数（到位且收到新鲜帧）；
  //   elapsed_s        ：观察段已耗时（秒）；
  //   move_cost_ema_s  ：单视点移动+等帧成本 EMA（秒，≤0=未测得）。
  ScanVerdict poll(
    bool gate_allowed, int moves, int effective_views, double elapsed_s,
    double move_cost_ema_s) const
  {
    if (moves >= config_.maximum_moves) {
      return ScanVerdict::MOVES_EXHAUSTED;
    }
    // 下限保证：有效视点未达下限前，质量收敛也不得收口（2.13-E2）。
    if (effective_views < config_.min_effective_views) {
      return ScanVerdict::CONTINUE;
    }
    if (gate_allowed) {
      return ScanVerdict::CONVERGED;
    }
    const double budget_s = effectiveBudgetS(move_cost_ema_s);
    if (elapsed_s >= budget_s) {
      return ScanVerdict::BUDGET_EXHAUSTED;
    }
    // 预测性收口：剩余预算按实测成本换不起一个有效视点，提前 finalize 走
    // 降级链，避免"再动一次却等不到帧"白耗一个预算窗口。
    if (move_cost_ema_s > 0.0 && budget_s - elapsed_s < move_cost_ema_s) {
      return ScanVerdict::BUDGET_EXHAUSTED;
    }
    return ScanVerdict::CONTINUE;
  }

private:
  ScanBudgetConfig config_;
};

}  // namespace peach_manipulation_skills
#endif  // PEACH_MANIPULATION_SKILLS__SCAN_BUDGET_HPP_
