// 功能：观察段扫描预算。覆盖够就停；否则把 maximum_moves 走完。
#ifndef PEACH_MANIPULATION__SCAN_BUDGET_HPP_
#define PEACH_MANIPULATION__SCAN_BUDGET_HPP_

namespace peach_manipulation
{

// 停准则对齐体积重建惯例（Open3D TSDF 对一组相机位姿逐张 integrate；
// 工业 NBV 在 max_views 或信息增益够了才停），而不是用「上次移动+等帧」
// 预测下一视点买不买得起：
//   * 质量门放行 → CONVERGED（不再为凑次数运动）；
//   * moves < maximum_moves → CONTINUE（把设计好的短移走完）；
//   * 否则 MOVES_EXHAUSTED。
// 墙钟 time_budget_s 只作日志对照。等帧超时在 waitForFreshTarget /
// waitForNewStation，不在这里用 EMA 放大预算或预测收口。
// 08-31 1633/1636：等帧把 EMA 抬到 ~8 s，15 s 预算剩 6 s，8 cm 第二机位被
// 预测收口砍掉，基线永远不够。
// 纯核零 ROS、零阻塞、零时钟依赖。

struct ScanBudgetConfig
{
  int maximum_moves{2};
  int min_effective_views{1};
  double time_budget_s{15.0};
  double budget_cost_margin{1.5};  // 保留兼容；不再参与停准则
};

enum class ScanVerdict
{
  CONTINUE,
  CONVERGED,
  BUDGET_EXHAUSTED,
  MOVES_EXHAUSTED
};

class ScanBudget
{
public:
  explicit ScanBudget(ScanBudgetConfig config = ScanBudgetConfig())
  : config_(config)
  {
  }

  double effectiveBudgetS(double /*move_cost_ema_s*/) const
  {
    return config_.time_budget_s;
  }

  ScanVerdict poll(
    bool gate_allowed, int moves, int /*effective_views*/, double /*elapsed_s*/,
    double /*move_cost_ema_s*/) const
  {
    if (gate_allowed) {
      return ScanVerdict::CONVERGED;
    }
    if (moves < config_.maximum_moves) {
      return ScanVerdict::CONTINUE;
    }
    return ScanVerdict::MOVES_EXHAUSTED;
  }

private:
  ScanBudgetConfig config_;
};

}  // namespace peach_manipulation
#endif  // PEACH_MANIPULATION__SCAN_BUDGET_HPP_
