#include "peach2_end_effector/io_backend.hpp"

#include <functional>
#include <mutex>
#include <optional>
#include <utility>

namespace peach2_end_effector
{

MockIoBackend::MockIoBackend(Config config, std::function<double()> now_s)
: config_(config), now_s_(std::move(now_s))
{
  output_level_ = !config_.close_level;
}

bool MockIoBackend::set_output(int pin, bool level)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++writes_;
  if (config_.fault == Fault::WRITE_FAILS) {
    return false;
  }
  if (pin != config_.cmd_pin) {
    return true;
  }
  const double now = now_s_();
  if (!output_written_ || level != output_level_) {
    closed_at_change_ = blade_closed_locked(now);
    output_change_s_ = now;
  }
  output_level_ = level;
  output_written_ = true;
  return true;
}

bool MockIoBackend::blade_closed_locked(double now) const
{
  const bool commanded_closed = output_level_ == config_.close_level;
  const double since = now - output_change_s_;
  if (commanded_closed) {
    return closed_at_change_ || since >= config_.close_delay_s;
  }
  if (config_.fault == Fault::OPEN_STUCK) {
    return closed_at_change_;
  }
  return closed_at_change_ && since < config_.open_delay_s;
}

std::optional<bool> MockIoBackend::feedback(int pin)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pin != config_.feedback_pin) {
    return std::nullopt;
  }
  bool closed = false;
  switch (config_.fault) {
    case Fault::NO_FEEDBACK:
      return std::nullopt;
    case Fault::STUCK_OPEN:
      closed = false;
      break;
    case Fault::STUCK_CLOSED:
      closed = true;
      break;
    case Fault::LOOPBACK:
      closed = output_level_ == config_.close_level;
      break;
    case Fault::NONE:
    case Fault::WRITE_FAILS:
    case Fault::OPEN_STUCK:
      closed = blade_closed_locked(now_s_());
      break;
  }
  return config_.feedback_active_high ? closed : !closed;
}

std::optional<double> MockIoBackend::current()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!config_.simulate_current) {
    return std::nullopt;
  }
  const bool commanded_closed = output_level_ == config_.close_level;
  if (!commanded_closed) {
    return config_.idle_current_a;
  }
  const double since = now_s_() - output_change_s_;
  if (since < config_.close_delay_s) {
    return config_.peak_current_a * (0.3 + 0.7 * since / config_.close_delay_s);
  }
  return config_.current_cut_signature ? config_.idle_current_a : config_.peak_current_a;
}

void MockIoBackend::set_fault(Fault fault)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_.fault = fault;
}

void MockIoBackend::set_current_cut_signature(bool present)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_.current_cut_signature = present;
}

int MockIoBackend::write_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return writes_;
}

std::optional<bool> MockIoBackend::last_output(int pin) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pin != config_.cmd_pin || !output_written_) {
    return std::nullopt;
  }
  return output_level_;
}

}  // namespace peach2_end_effector
