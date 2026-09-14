// 功能：接触检测纯核（④层硬接触止损，2026-09-14 约束重设计）。
// 输入为逐关节电流序列（/aubo_io_controller/joint_status 的
// JointStatus.current，SDK 原始单位）；判别是「特征级」而非「电平级」——
// 套入段与袋/果摩擦是合法接触（渐变、有界），硬碰枝是腕轴偏差尖峰/陡增。
// 阈值须真机受控试验标定（默认 0=不触发，仅接线）。
// 零 ROS 纯核：不 import rclpy / 不造 DDS，可被包内 pytest 直接驱动。
#ifndef PEACH_MANIPULATION__CONTACT_MONITOR_HPP_
#define PEACH_MANIPULATION__CONTACT_MONITOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace peach_manipulation
{

// 关节顺序约定（与 JOINT_ORDER 一致）：0=shoulder 1=upperArm 2=foreArm
// 3=wrist1 4=wrist2 5=wrist3。腕轴重力矩小、离刀具近，信噪比最好。
constexpr std::array<std::size_t, 3> kWristJointIndices = {3U, 4U, 5U};

struct ContactDetectConfig
{
  bool enabled{false};
  double baseline_s{0.2};        // 守护段开始前的静止基线窗（秒）
  double slope_threshold{0.0};   // 腕轴偏差增长速率阈值（单位/秒）；<=0 不判
  double spike_threshold{0.0};   // 腕轴偏差瞬时阈值（单位）；<=0 不判
};

struct CurrentSample
{
  double t{0.0};
  std::array<double, 6> current{};
};

enum class ContactVerdict
{
  NORMAL,          // 正常（含合法摩擦渐变）
  SUSPECTED_HARD,  // 疑似硬接触（尖峰或陡增超阈）
  INSUFFICIENT,    // 样本不足（基线未成 / 无新样本）
};

struct ContactReport
{
  ContactVerdict verdict{ContactVerdict::NORMAL};
  double max_wrist_deviation{0.0};
  double max_wrist_slope{0.0};
  std::size_t joint_index{0U};
  std::string reason{"正常"};
};

// 单守护段实例：start 传入段前基线样本；update 逐样本送入，返回当次判定。
// 线程模型：单线程顺序调用（节点 timer 串行），无锁。
class ContactMonitor
{
public:
  explicit ContactMonitor(const ContactDetectConfig & config)
  : config_(config) {}

  void start(const std::vector<CurrentSample> & baseline)
  {
    samples_.clear();
    baseline_mean_.fill(0.0);
    prev_wrist_deviation_.fill(0.0);
    reason_.clear();
    if (baseline.size() < 2U) {
      baseline_ready_ = false;
      return;
    }
    std::array<double, 6> sum{};
    for (const auto & sample : baseline) {
      for (std::size_t j = 0; j < 6U; ++j) {
        sum[j] += sample.current[j];
      }
    }
    const double n = static_cast<double>(baseline.size());
    for (std::size_t j = 0; j < 6U; ++j) {
      baseline_mean_[j] = sum[j] / n;
    }
    // 基线期偏差视为 0：斜率从守护段开始计量增量
    for (const std::size_t j : kWristJointIndices) {
      prev_wrist_deviation_[j] = 0.0;
    }
    baseline_ready_ = true;
  }

  ContactReport update(const CurrentSample & sample)
  {
    ContactReport report;
    if (!config_.enabled || !baseline_ready_) {
      report.verdict = ContactVerdict::INSUFFICIENT;
      report.reason = config_.enabled ? "基线未成" : "接触检测关闭";
      return report;
    }
    if (!samples_.empty() && sample.t <= samples_.back().t) {
      // 时钟回退/重复样本：丢掉但不算异常
      report.reason = "重复样本";
      return report;
    }
    const double dt = samples_.empty() ?
      config_.baseline_s : sample.t - samples_.back().t;
    samples_.push_back(sample);
    if (samples_.size() > 64U) {
      samples_.erase(samples_.begin());
    }
    for (const std::size_t j : kWristJointIndices) {
      if (!std::isfinite(sample.current[j])) {
        continue;
      }
      const double deviation = sample.current[j] - baseline_mean_[j];
      report.max_wrist_deviation =
        std::max(report.max_wrist_deviation, std::abs(deviation));
      // 斜率 = 相邻样本的偏差增量/dt（不是累计偏差/dt——合法摩擦渐变
      // 会持续抬高累计偏差，用后者会把缓升误判成陡增）。
      const double delta =
        std::abs(deviation - prev_wrist_deviation_[j]);
      const double slope = dt > 1.0e-3 ? delta / dt : 0.0;
      prev_wrist_deviation_[j] = deviation;
      report.max_wrist_slope = std::max(report.max_wrist_slope, slope);
      if (config_.spike_threshold > 0.0 &&
        std::abs(deviation) > config_.spike_threshold)
      {
        report.verdict = ContactVerdict::SUSPECTED_HARD;
        report.joint_index = j;
        report.reason = "腕轴电流偏差尖峰";
        return report;
      }
      if (config_.slope_threshold > 0.0 && slope > config_.slope_threshold) {
        report.verdict = ContactVerdict::SUSPECTED_HARD;
        report.joint_index = j;
        report.reason = "腕轴电流偏差陡增";
        return report;
      }
    }
    report.reason = "正常";
    return report;
  }

private:
  ContactDetectConfig config_;
  std::array<double, 6> baseline_mean_{};
  std::array<double, 6> prev_wrist_deviation_{};
  std::vector<CurrentSample> samples_;
  bool baseline_ready_{false};
  std::string reason_;
};

}  // namespace peach_manipulation
#endif  // PEACH_MANIPULATION__CONTACT_MONITOR_HPP_
