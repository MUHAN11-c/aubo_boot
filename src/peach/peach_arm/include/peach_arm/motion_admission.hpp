// 一个本地动作或手动规划服务从受理到退场独占运动策略和规划器。
#ifndef PEACH_ARM__MOTION_ADMISSION_HPP_
#define PEACH_ARM__MOTION_ADMISSION_HPP_

#include <atomic>
#include <mutex>

namespace peach_arm
{
class MotionAdmission
{
public:
  // std::unique_lock 持有的是请求席位。短互斥只保护进出，请求可移交工作线程。
  bool try_lock()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (active_ || stopping_ > 0 || reload_pending_) {return false;}
    active_ = true;
    return true;
  }

  void unlock()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    canceled_.store(false);
    active_ = false;
  }

  bool busy() const {return active_.load();}
  const std::atomic_bool & canceled() const {return canceled_;}

  template<class Stop>
  void cancel(Stop stop)
  {
    std::unique_lock<std::mutex> guard(mutex_);
    canceled_.store(active_);
    ++stopping_;
    guard.unlock();
    // 在锁外停止 MoveIt/MTC；所有在途停止级联结束前不准入新请求。
    try {
      stop();
    } catch (...) {
      guard.lock();
      --stopping_;
      throw;
    }
    guard.lock();
    --stopping_;
  }

  // 服务与参数回调属于不同回调组。验证后若已被抢占，先禁止新请求，
  // 由默认组定时器在当前请求结束后提交已接受的参数。
  template<class Reload>
  void reloadIfIdle(bool parameter_changed, Reload reload)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    reload_pending_ = reload_pending_ || parameter_changed;
    if (reload_pending_ && !active_ && stopping_ == 0) {
      reload();
      reload_pending_ = false;
    }
  }

private:
  std::mutex mutex_;
  std::atomic_bool active_{false};
  std::atomic_bool canceled_{false};
  bool reload_pending_{false};
  unsigned int stopping_{0};
};
}  // namespace peach_arm

#endif  // PEACH_ARM__MOTION_ADMISSION_HPP_
