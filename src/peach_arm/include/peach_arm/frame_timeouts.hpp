// 功能：帧率自适应超时族纯核（W5-3）。六个公式自 manipulation_skills_node
// 的 trackFrameInterval/effective* 成员逐字搬移（含 COLLECTING 窗口分支）；
// 观测帧间隔 EMA 状态内聚于此，互斥保护跨线程读写（订阅线程写、周期线程读）。
// 配置仅在空闲期经 updateConfig 重写（沿用节点参数重载纪律）。
#ifndef PEACH_MANIPULATION__FRAME_TIMEOUTS_HPP_
#define PEACH_MANIPULATION__FRAME_TIMEOUTS_HPP_

#include <algorithm>
#include <mutex>
#include <utility>

#include "peach_arm/safety_gate.hpp"  // adaptive_timeout_s

namespace peach_arm
{

struct FrameRateTimeoutConfig
{
  /// 未测得观测间隔 EMA 时的回退帧间隔 [s]。0=不预填。
  double assumed_frame_interval_s{0.4};
  double frame_wait_s{4.0};              ///< 等帧超时配置上限 [s]。
  double target_observation_max_age_config_s{3.0};  ///< 观测龄配置基准 [s]。
  double reconfirm_wait_s{6.0};          ///< 再确认等新鲜观测超时 [s]。
  double refined_timeout_s{30.0};        ///< 精化等待超时 [s]。
};

class FrameRateTimeouts
{
public:
  explicit FrameRateTimeouts(FrameRateTimeoutConfig config = FrameRateTimeoutConfig())
  : config_(config) {}

  // 空闲期配置重写（loadParameters；周期运行中改参已被 onParameters 前置拒绝）。
  void updateConfig(FrameRateTimeoutConfig config) {config_ = config;}

  // 观测话题到达间隔 EMA 更新（onTargets 每帧调用）。
  void onTargetFrame(double arrival_s)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_arrival_s_ > 0.0) {
      const double dt = arrival_s - last_arrival_s_;
      // 异常间隔（暂停后首帧/时钟跳变）不进 EMA，避免污染帧率估计
      if (dt > 1e-3 && dt < 30.0) {
        frame_interval_ema_s_ = frame_interval_ema_s_ > 0.0 ?
          0.7 * frame_interval_ema_s_ + 0.3 * dt : dt;
      }
    }
    last_arrival_s_ = arrival_s;
  }

  /// 实测观测间隔 EMA [s]；≤0=未测得。
  double frameIntervalEmaS() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_interval_ema_s_;
  }

  // 帧率自适应取值：等帧窗口在 EMA 未测得时用 assumed_frame_interval_s
  // 估超时；新鲜度门在未测得前保持 yaml 回退，且不得收得比回退更紧。
  double waitIntervalS() const
  {
    const double ema = frameIntervalEmaS();
    if (ema > 0.0) {
      return ema;
    }
    return config_.assumed_frame_interval_s;
  }

  double frameWaitS() const
  {
    const double interval = waitIntervalS();
    if (interval <= 0.0) {return config_.frame_wait_s;}
    // 视点到位后等 ~4 帧 + 1s 稳定余量；下限 2s，上限为配置值
    return adaptive_timeout_s(interval, 4.0, 1.0, 2.0, config_.frame_wait_s);
  }

  double targetMaxAgeS() const
  {
    const double ema = frameIntervalEmaS();
    if (ema <= 0.0) {return config_.target_observation_max_age_config_s;}
    // 低帧率放宽；不得收得比 yaml 回退更紧（曾用 0.4s 预填 EMA，把 3s 收到 1.5s）。
    return std::max(
      config_.target_observation_max_age_config_s,
      adaptive_timeout_s(ema, 2.5, 0.5, 1.0, 10.0));
  }

  double reconfirmWaitS() const
  {
    const double interval = waitIntervalS();
    if (interval <= 0.0) {return config_.reconfirm_wait_s;}
    // 与视点等帧同一形状：等 ~4 帧 + 1s 稳定余量，下限 2s，上限为配置值
    // （reconfirm_wait_s 同时承担回退值与自适应上限，摆动等平息也在本窗口预算内）。
    return adaptive_timeout_s(interval, 4.0, 1.0, 2.0, config_.reconfirm_wait_s);
  }

  // 精化等待（2.7-FINALIZE 的 T(refined)）。collecting=true（重建仍在
  // COLLECTING）用配置上限覆盖采集→finalize→refit 全程；IDLE 维持短窗快速
  // 失败（无会话等不来）。短窗假设「finalize 已触发、refit 在 ~3 帧内闩锁」：
  // refit 耗时由重建节点持有、本包不可得，按观测帧间隔近似（≈3 帧 + 2s 余量；
  // 下限 2s——高帧率时 refit 仍有固定计算耗时；上限为配置值）。
  double refinedWaitS(bool reconstruction_collecting) const
  {
    if (reconstruction_collecting) {
      return config_.refined_timeout_s;
    }
    const double interval = waitIntervalS();
    if (interval <= 0.0) {return config_.refined_timeout_s;}
    return adaptive_timeout_s(
      interval, 3.0, 2.0, std::min(2.0, config_.refined_timeout_s),
      config_.refined_timeout_s);
  }

private:
  // 配置为平凡成员：仅在空闲期 updateConfig 重写（运行中改参被前置拒绝）。
  FrameRateTimeoutConfig config_;
  mutable std::mutex mutex_;
  double frame_interval_ema_s_{0.0};
  double last_arrival_s_{0.0};
};

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__FRAME_TIMEOUTS_HPP_
