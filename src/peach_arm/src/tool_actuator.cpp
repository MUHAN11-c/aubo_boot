// 功能：刀具 GPIO 状态机。SetIO ACK 只表示柜侧接受命令，不等于切断确认。
#include "peach_arm/tool_actuator.hpp"

namespace peach_arm
{

void ToolActuator::setSendIo(SendIo send_io)
{
  send_io_ = std::move(send_io);
}

bool ToolActuator::arm(const ToolCommandContext & ctx, std::string & reason)
{
  if (cut_commanded_ && ctx.contact_transaction_id == ctx_.contact_transaction_id &&
    !ctx.contact_transaction_id.empty())
  {
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
  if (state_ != ToolActuatorState::ARMED) {
    reason = "tool_not_armed";
    return false;
  }
  if (cut_commanded_) {
    reason = "cut_already_commanded_this_transaction";
    return false;
  }
  if (!send_io_) {
    reason = "tool_io_not_wired";
    return false;
  }
  if (!send_io_(reason)) {
    state_ = ToolActuatorState::UNKNOWN;
    return false;
  }
  cut_commanded_ = true;
  new_closed_edge_ = false;  // 命令边界：旧沿作废（§12.2 新事件门）
  state_ = ToolActuatorState::CUT_COMMAND_SENT;
  reason = "CUT_COMMAND_ACCEPTED";
  return true;
}

bool ToolActuator::ingestToolDi(bool pin_closed_level, std::string & edge_desc)
{
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
    axes_.payload = 1u;  // PAYLOAD_ABSENT（开刀=释放；真载荷证据 M0）
    axes_.retention = 1u;  // RETENTION_READY
  } else {
    new_closed_edge_ = true;  // 本命令后的闭合新沿（confirmFeedback 消费）
  }
  return true;
}

bool ToolActuator::confirmFeedback(bool hardware_ok, std::string & reason)
{
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
  state_ = ToolActuatorState::CUT_FEEDBACK_CONFIRMED;
  axes_.blade = 3u;  // BLADE_CLOSED（新沿已核；闭合≠分离，见头注）
  axes_.evidence_source = "di:closed-rise+confirm";
  new_closed_edge_ = false;
  reason = "CUT_CONFIRMED";
  return true;
}

void ToolActuator::resetSafe()
{
  state_ = ToolActuatorState::SAFE;
  cut_commanded_ = false;
  di_seen_ = false;
  di_pin_last_ = false;
  new_closed_edge_ = false;
  axes_ = ToolAxes{};  // 三轴归 UNKNOWN
  ctx_ = ToolCommandContext{};
}

void ToolActuator::markUnknown()
{
  state_ = ToolActuatorState::UNKNOWN;
}

bool ToolActuator::sameTransaction(const std::string & transaction_id) const
{
  return !transaction_id.empty() && transaction_id == ctx_.contact_transaction_id;
}

bool ToolActuator::cutAlreadyCommanded() const
{
  return cut_commanded_;
}

}  // namespace peach_arm
