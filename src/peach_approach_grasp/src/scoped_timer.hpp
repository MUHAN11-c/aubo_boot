// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
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
// 回调耗时埋点（重构协议 2.16-5）：轻量 RAII 计时器 + 累计注册表。
// 用法：关键回调入口栈上构造 ScopedTimer，析构时按标签记录耗时——
// DEBUG 日志节流输出 + 累计进 CallbackTimingRegistry（节点侧随 ~/status
// 状态 JSON 一起发布，不新增话题）。计时一律 steady_clock（墙钟）。
#ifndef SCOPED_TIMER_HPP_
#define SCOPED_TIMER_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace peach_approach_grasp
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

}  // namespace peach_approach_grasp
#endif  // SCOPED_TIMER_HPP_
