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
#ifndef PEACH_APPROACH_GRASP__PREPLAN_SLOT_HPP_
#define PEACH_APPROACH_GRASP__PREPLAN_SLOT_HPP_

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

namespace peach_approach_grasp
{

// MTC 预规划槽（2.13-E3 plan-while-waiting）纯核：再确认等待窗口期后台预规划
// 接近/插入任务的生命周期状态与复用判定。MTC 任务对象本体由 GraspTask 持有，
// 本槽只记录"规划是否落定/成败/规划基准锚点"，供 BT 工作线程与预规划线程
// 之间做最小同步。线程安全自给（内部互斥+条件变量）。
//
// 生命周期：begin（再确认入口，记规划基准锚点）→ 预规划线程 complete（落定
// READY/FAILED）→ 再确认通过点 reusable 判定（漂移 ≤ grasp.replan_threshold_m
// 才复用，超限重规划）→ discard（取消/周期结束/漂移重算后清槽）。
// discard 后迟到的 complete 一律忽略（stale 防护：重算几何后旧预规划结果
// 不得再被采纳）。
class PreplanSlot
{
public:
  enum class State
  {
    EMPTY,     // 空槽（未启动或已丢弃）
    PLANNING,  // 后台规划中
    READY,     // 规划成功，解可复用
    FAILED     // 规划失败（reason 给出原因）
  };

  // 启动一次预规划：PLANNING/READY 状态下重复 begin 拒绝（返回 false），
  // 调用方须先 discard。anchor 为规划所用入口几何对应的目标锚点（base 系），
  // 复用判定以它与再确认最终锚点的漂移为准。
  bool begin(const Eigen::Vector3d & anchor)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::PLANNING || state_ == State::READY) {
      return false;
    }
    anchor_ = anchor;
    reason_.clear();
    state_ = State::PLANNING;
    return true;
  }

  // 预规划线程落定结果；非 PLANNING 状态（已 discard/重复落定）忽略。
  void complete(bool success, const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ != State::PLANNING) {
        return;
      }
      state_ = success ? State::READY : State::FAILED;
      reason_ = reason;
    }
    cv_.notify_all();
  }

  // 等待落定（READY/FAILED）：true=已落定；false=超时或 cancel 置位。
  // 50ms 轮询切片保取消响应（与 cache_ 等待同纪律）。
  bool waitReady(double timeout_s, const std::atomic_bool & cancel) const
  {
    using Clock = std::chrono::steady_clock;
    const Clock::time_point deadline = Clock::now() +
      std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(timeout_s));
    std::unique_lock<std::mutex> lock(mutex_);
    while (state_ == State::PLANNING) {
      if (cancel.load()) {
        return false;
      }
      const Clock::time_point now = Clock::now();
      if (now >= deadline) {
        return false;
      }
      cv_.wait_until(
        lock, std::min(deadline, now + std::chrono::milliseconds(50)));
    }
    return state_ != State::PLANNING;
  }

  // 复用判定：READY 且 |anchor − 规划基准锚点| ≤ threshold_m 才复用；
  // 否则 why 给出不复用原因（未规划/失败/漂移超限），调用方走重规划。
  bool reusable(
    const Eigen::Vector3d & anchor, double threshold_m, std::string & why) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::EMPTY) {
      why = "无预规划（未启动或已丢弃）";
      return false;
    }
    if (state_ == State::PLANNING) {
      why = "预规划尚未落定";
      return false;
    }
    if (state_ == State::FAILED) {
      why = "预规划失败: " + reason_;
      return false;
    }
    const double drift_m = (anchor - anchor_).norm();
    if (drift_m > threshold_m) {
      why = "锚点漂移 " + std::to_string(drift_m) + "m 超预规划复用阈值 " +
        std::to_string(threshold_m) + "m，重规划";
      return false;
    }
    why.clear();
    return true;
  }

  // 丢弃槽（取消/周期结束/漂移重算后调用）：此后迟到的 complete 被忽略。
  void discard()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = State::EMPTY;
    reason_.clear();
  }

  State state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  std::string reason() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return reason_;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  State state_{State::EMPTY};
  Eigen::Vector3d anchor_{Eigen::Vector3d::Zero()};
  std::string reason_;
};

}  // namespace peach_approach_grasp
#endif  // PEACH_APPROACH_GRASP__PREPLAN_SLOT_HPP_
