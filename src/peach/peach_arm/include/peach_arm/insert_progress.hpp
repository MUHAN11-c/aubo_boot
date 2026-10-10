// 批次5（2026-09-23 重构）：套入/撤退实测行程判据纯核（零 ROS）。
// FINAL_PLAN §11.3：时间只作截止，不作到位证明；无进展超时收口 UNKNOWN。
#ifndef PEACH_MANIPULATION__INSERT_PROGRESS_HPP_
#define PEACH_MANIPULATION__INSERT_PROGRESS_HPP_

#include <algorithm>
#include <cstdint>

namespace peach_arm
{

enum class InsertProgress : uint8_t
{
  CONTINUE = 0,   ///< 继续等（有进展或未到停滞窗）
  DONE = 1,       ///< 实测行程达目标（≥ target − tol）
  STALLED = 2,    ///< 停滞：窗口内实测行程增益 < 最小增益
  DEADLINE = 3,   ///< 截止到（时间上限，非到位）
};

/// 单步判据。
///
/// measured_m：实测行程（FK 沿轴投影；或进度话题回退值）。
/// target_m/tol_m：目标行程与容差（DONE 判据 measured ≥ target − tol）。
/// gain_window_s/gain_since_s：停滞窗——距上次有效增益的时长。
/// min_gain_m：窗口内视为"有进展"的最小增益 [m]。
/// deadline_in_s：剩余时间预算（≤0 即截止）。
inline InsertProgress insertProgressDecision(
  double measured_m, double target_m, double tol_m,
  double gain_since_s, double gain_window_s, double min_gain_m,
  double deadline_in_s)
{
  if (measured_m >= target_m - std::max(0.0, tol_m)) {
    return InsertProgress::DONE;
  }
  if (deadline_in_s <= 0.0) {
    return InsertProgress::DEADLINE;
  }
  if (gain_since_s >= gain_window_s && min_gain_m > 0.0) {
    return InsertProgress::STALLED;
  }
  return InsertProgress::CONTINUE;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__INSERT_PROGRESS_HPP_
