// 功能：通过任务完成信号有界等待，再由 std::thread 正式回收线程对象。
#ifndef PEACH_ARM__THREAD_JOIN_HPP_
#define PEACH_ARM__THREAD_JOIN_HPP_

#include <chrono>
#include <future>
#include <thread>

namespace peach_arm
{
inline bool joinFinishedThread(
  std::thread & worker, std::future<void> & done, std::chrono::milliseconds timeout)
{
  if (!worker.joinable()) {
    return true;
  }
  if (!done.valid() || worker.get_id() == std::this_thread::get_id() ||
    done.wait_for(timeout) != std::future_status::ready)
  {
    return false;
  }
  // 不用 pthread_join 回收 std::thread 的 native_handle：它不会清 joinable，
  // 后续析构会触发 std::terminate。future 只判完成，所有权由 join() 释放。
  worker.join();
  return true;
}
}  // namespace peach_arm
#endif  // PEACH_ARM__THREAD_JOIN_HPP_
