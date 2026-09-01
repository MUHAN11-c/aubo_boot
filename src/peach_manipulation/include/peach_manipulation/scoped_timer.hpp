// 功能：回调耗时 RAII 计时器与累计表。随 ~/status 发布，不新增话题。
// 用法：回调入口栈上构造 ScopedTimer；析构按标签记耗时（steady_clock）。
#ifndef PEACH_MANIPULATION__SCOPED_TIMER_HPP_
#define PEACH_MANIPULATION__SCOPED_TIMER_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace peach_manipulation
{
// 回调耗时累计注册表：线程安全自给（节点回调跨默认组/规划组并发）。
// 输出为标签 → {count, total_ms, max_ms, last_ms} 的 JSON 投影。
class CallbackTimingRegistry
{
public:
  struct Entry
  {
    uint64_t count{0};
    double total_ms{0.0};
    double max_ms{0.0};
    double last_ms{0.0};
  };

  void record(const std::string & label, double elapsed_ms)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & entry = entries_[label];
    entry.count += 1;
    entry.total_ms += elapsed_ms;
    entry.max_ms = elapsed_ms > entry.max_ms ? elapsed_ms : entry.max_ms;
    entry.last_ms = elapsed_ms;
  }

  nlohmann::json toJson() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nlohmann::json out = nlohmann::json::object();
    for (const auto & [label, entry] : entries_) {
      out[label] = {
        {"count", entry.count},
        {"total_ms", entry.total_ms},
        {"max_ms", entry.max_ms},
        {"last_ms", entry.last_ms},
      };
    }
    return out;
  }

  // 测试用快照（值拷贝，避免暴露内部锁）。
  std::map<std::string, Entry> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_;
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, Entry> entries_;
};

// RAII 计时器：构造取起始点，析构计算耗时——注入注册表则累计一条记录，
// 并按节流周期输出 DEBUG 日志。禁止拷贝/移动（析构只应发生一次）。
class ScopedTimer
{
public:
  explicit ScopedTimer(
    rclcpp::Logger logger, std::string label,
    CallbackTimingRegistry * registry = nullptr, double throttle_ms = 2000.0)
  : logger_(std::move(logger)),
    label_(std::move(label)),
    registry_(registry),
    throttle_ms_(throttle_ms),
    start_(std::chrono::steady_clock::now())
  {
  }

  ScopedTimer(const ScopedTimer &) = delete;
  ScopedTimer & operator=(const ScopedTimer &) = delete;

  ~ScopedTimer()
  {
    const double elapsed = elapsedMs();
    if (registry_ != nullptr) {
      registry_->record(label_, elapsed);
    }
    RCLCPP_DEBUG_THROTTLE(
      logger_, steady_clock_, throttle_ms_,
      "回调耗时 %s: %.2f ms", label_.c_str(), elapsed);
  }

  double elapsedMs() const
  {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start_).count();
  }

private:
  rclcpp::Logger logger_;
  std::string label_;
  CallbackTimingRegistry * registry_;
  double throttle_ms_;
  std::chrono::steady_clock::time_point start_;
  // 节流宏需要时钟左值；RCL_STEADY_TIME 与耗时口径一致。
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace peach_manipulation
#endif  // PEACH_MANIPULATION__SCOPED_TIMER_HPP_
