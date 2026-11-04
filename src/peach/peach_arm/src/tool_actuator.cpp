// 功能：刀具 GPIO 状态机。SetIO ACK 只表示柜侧接受命令，不等于切断确认。
#include "peach_arm/tool_actuator.hpp"

#include <exception>
#include <utility>

namespace peach_arm
{

void ToolActuator::setSendIo(SendIo send_io)
{
  std::lock_guard<std::mutex> lock(mutex_);
  send_io_ = std::move(send_io);
}

bool ToolActuator::arm(const ToolCommandContext & ctx, std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (sending_ || cut_commanded_) {
    reason = "cut_already_commanded_this_transaction";
    return false;
  }
  ctx_ = ctx;
  state_ = ToolActuatorState::ARMED;
  reason.clear();
  return true;
}

bool ToolActuator::sendCut(std::string & reason)
{
  SendIo send;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    send = send_io_;
  }
  return sendCut(0, send, reason);
}

bool ToolActuator::sendCut(
  int64_t dispatch_stamp_ns, const SendIo & send_io, std::string & reason)
{
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ToolActuatorState::ARMED || cut_commanded_ || !send_io) {
      reason = cut_commanded_ ? "cut_already_commanded_this_transaction" :
        (!send_io ? "tool_io_not_wired" : "tool_not_armed");
      return false;
    }
    cut_commanded_ = true;
    sending_ = true;
    new_closed_edge_ = false;
    dispatch_stamp_ns_ = dispatch_stamp_ns;
    generation = generation_;
  }
  // 等 ROS 服务响应时 DI/诊断继续进入。ACK 与发送后的沿到齐才可确认。
  bool accepted = false;
  try {
    accepted = send_io(reason);
  } catch (const std::exception & error) {
    reason = error.what();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  sending_ = false;
  if (generation_ != generation || state_ != ToolActuatorState::ARMED) {
    reason = "tool_transaction_changed";
    return false;
  }
  if (!accepted) {
    state_ = ToolActuatorState::UNKNOWN;
    return false;
  }
  state_ = ToolActuatorState::CUT_COMMAND_SENT;
  if (new_closed_edge_) {confirmClosed();}
  reason = "CUT_COMMAND_ACCEPTED";
  return true;
}

bool ToolActuator::ingestToolDi(bool pin_closed_level, std::string & edge_desc, int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stamp_ns > 0) {
    if (stamp_ns <= last_di_stamp_ns_) {return false;}
    last_di_stamp_ns_ = stamp_ns;
  }
  const bool had_prev = di_seen_;
  const bool prev = di_pin_last_;
  di_seen_ = true;
  di_pin_last_ = pin_closed_level;
  if (!had_prev || prev == pin_closed_level) {
    return false;  // 首帧（无沿基准）或电平无变化
  }
  edge_desc = pin_closed_level ? "di:closed-rise" : "di:open-fall";
  // 三轴投影（批次3）：闭合沿→BLADE_CLOSED；开沿→BLADE_OPEN+载荷释放。
  axes_.blade = pin_closed_level ? 3u : 1u;  // ToolState BLADE_CLOSED/BLADE_OPEN
  axes_.evidence_source = edge_desc;
  if (!pin_closed_level) {
    new_closed_edge_ = false;
    axes_.payload = 1u;  // PAYLOAD_ABSENT（开刀=释放；真载荷证据 M0）
    axes_.retention = 1u;  // RETENTION_READY
  } else if (cut_commanded_ &&
    (dispatch_stamp_ns_ == 0 || stamp_ns > dispatch_stamp_ns_))
  {
    new_closed_edge_ = true;
    if (stamp_ns > 0 && state_ == ToolActuatorState::CUT_COMMAND_SENT) {
      confirmClosed();
    }
  }
  return true;
}

bool ToolActuator::confirmFeedback(bool hardware_ok, std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != ToolActuatorState::CUT_COMMAND_SENT) {
    reason = "cut_command_not_sent";
    return false;
  }
  if (!new_closed_edge_) {
    // §12.2：闭合证据必须是本命令后的新事件——早已卡高/无沿不得确认
    reason = "no_new_di_edge";
    return false;
  }
  if (!hardware_ok) {
    reason = "cut_feedback_unavailable";
    return false;
  }
  confirmClosed();
  reason = "CUT_CONFIRMED";
  return true;
}

void ToolActuator::confirmClosed()
{
  state_ = ToolActuatorState::CUT_FEEDBACK_CONFIRMED;
  axes_.blade = 3u;  // BLADE_CLOSED（新沿已核；闭合≠分离，见头注）
  axes_.evidence_source = "di:closed-rise+confirm";
  new_closed_edge_ = false;
}

void ToolActuator::resetSafe()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  dispatch_stamp_ns_ = 0;
  last_di_stamp_ns_ = 0;
  state_ = sending_ ? ToolActuatorState::UNKNOWN : ToolActuatorState::SAFE;
  cut_commanded_ = false;
  di_seen_ = false;
  di_pin_last_ = false;
  new_closed_edge_ = false;
  axes_ = ToolAxes{};  // 三轴归 UNKNOWN
  ctx_ = ToolCommandContext{};
}

void ToolActuator::markUnknown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = ToolActuatorState::UNKNOWN;
}

bool ToolActuator::expireUnconfirmed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == ToolActuatorState::CUT_FEEDBACK_CONFIRMED) {return false;}
  state_ = ToolActuatorState::UNKNOWN;
  return true;
}

ToolActuatorState ToolActuator::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

ToolCommandContext ToolActuator::context() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ctx_;
}

ToolAxes ToolActuator::axes(ToolActuatorState * state) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state) {*state = state_;}
  return axes_;
}

bool ToolActuator::sameTransaction(const std::string & transaction_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !transaction_id.empty() && transaction_id == ctx_.contact_transaction_id;
}

bool ToolActuator::cutAlreadyCommanded() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cut_commanded_;
}

}  // namespace peach_arm
