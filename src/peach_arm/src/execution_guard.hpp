// 执行守卫（私有头，不安装）：move_group TEM 异常（stop 事件风暴）时，
// 同步 execute 永久阻塞且不理取消。此处把执行放入独立线程，先到先收：
// 完成 / 取消 / 超时三者谁先到按谁收口；弃等时线程所有权移交 RetireBucket
// （闭包只持有自己的拷贝与共享资源，绝不引用调用方栈或 this）。
// 依据 C++ [futures.async]/7：std::async 的 future 析构阻塞到任务结束，
// 故不能用 std::async 表达"弃等"（2026-09-23 审查 P0-1）。
#ifndef PEACH_MANIPULATION__EXECUTION_GUARD_HPP_
#define PEACH_MANIPULATION__EXECUTION_GUARD_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/logger.hpp>

namespace peach_arm
{

/// 弃等线程收纳桶：条目完结后由后续调用 reap；桶随宿主类析构（不 join，
/// 线程只触自己捕获的共享资源）。
class RetireBucket
{
public:
  struct Entry
  {
    std::thread worker;
    std::shared_ptr<std::atomic_bool> finished;
  };

  void add(std::thread && worker, std::shared_ptr<std::atomic_bool> finished)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    retiring_.push_back(Entry{std::move(worker), std::move(finished)});
  }

  void reap()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    retiring_.erase(
      std::remove_if(
        retiring_.begin(), retiring_.end(),
        [](Entry & e) {
          if (!e.finished->load()) {
            return false;
          }
          if (e.worker.joinable()) {
            e.worker.join();
          }
          return true;
        }),
      retiring_.end());
  }

  std::size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return retiring_.size();
  }

  /// 宿主析构：完结的 join 收尾，滞留的 detach（线程只触自己的捕获；
  /// 共享资源由捕获的 shared_ptr 维持存活）。
  void dispose()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & e : retiring_) {
      if (e.finished->load() && e.worker.joinable()) {
        e.worker.join();
      } else if (e.worker.joinable()) {
        e.worker.detach();
      }
    }
    retiring_.clear();
  }

private:
  std::mutex mutex_;
  std::vector<Entry> retiring_;
};

/// 执行体抛异常时按 FAILURE 收口（不得让异常经有界执行逃逸到周期线程）。
inline moveit::core::MoveItErrorCode settleResult(
  std::shared_future<moveit::core::MoveItErrorCode> & future)
{
  try {
    return future.get();
  } catch (...) {
    return moveit::core::MoveItErrorCode::FAILURE;
  }
}

/// 有界执行：execute_fn 在独立线程运行并写入 promise；等待环 100ms 轮询，
/// 取消探针或超时先到即 stop_fn() 并给 10s 宽限；宽限后仍未完结则弃等
/// （线程移交 retiring，闭包只持有自己的捕获）。返回终态错误码；弃等
/// 返回 FAILURE 且 *abandoned=true（宿主据此处置所有权，如泄漏任务）。
template<typename ExecFn>
moveit::core::MoveItErrorCode runBoundedExecute(
  ExecFn && execute_fn, const std::function<void()> & stop_fn,
  const std::function<bool()> & cancel_probe, double timeout_s,
  const rclcpp::Logger & logger, RetireBucket & retiring,
  bool * abandoned)
{
  *abandoned = false;
  auto result = std::make_shared<
    std::promise<moveit::core::MoveItErrorCode>>();
  auto finished = std::make_shared<std::atomic_bool>(false);
  auto shared_result =
    std::shared_future<moveit::core::MoveItErrorCode>(result->get_future());
  std::thread worker(
    [fn = std::forward<ExecFn>(execute_fn), result, finished]() mutable {
      // 执行体异常不得逃逸线程入口（std::terminate）：捕获后经
      // future 传递，按异常收口（审查 F2）。
      try {
        result->set_value(fn());
      } catch (...) {
        result->set_exception(std::current_exception());
      }
      finished->store(true);
    });
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_s);
  bool triggered = false;
  bool by_timeout = false;
  while (!triggered &&
    shared_result.wait_for(std::chrono::milliseconds(100)) !=
    std::future_status::ready)
  {
    if (cancel_probe && cancel_probe()) {
      triggered = true;
    } else if (std::chrono::steady_clock::now() >= deadline) {
      triggered = true;
      by_timeout = true;
    }
  }
  if (!triggered) {
    worker.join();
    return settleResult(shared_result);
  }
  // 触发点留痕（P0 观测性）：此前超时移交只体现为调用方的失败 reason
  if (by_timeout) {
    RCLCPP_WARN(
      logger, "有界执行超时（%.1fs），已发起停止并进入 10s 宽限", timeout_s);
  } else {
    RCLCPP_INFO(logger, "有界执行收到取消，已发起停止并进入 10s 宽限");
  }
  if (stop_fn) {
    stop_fn();
  }
  if (shared_result.wait_for(std::chrono::seconds(10)) ==
    std::future_status::ready)
  {
    worker.join();
    return settleResult(shared_result);
  }
  retiring.add(std::move(worker), finished);
  *abandoned = true;
  RCLCPP_WARN(
    logger,
    "有界执行弃等：10s 宽限后执行线程未完结，移交 retire 桶（%s）",
    by_timeout ? "超时" : "取消");
  return moveit::core::MoveItErrorCode::FAILURE;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__EXECUTION_GUARD_HPP_
