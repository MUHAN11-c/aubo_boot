// 功能：刀具 GPIO 状态机。SetIO ACK ≠ 切断确认。
#ifndef PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_
#define PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include "peach_manipulation/tool_profile.hpp"

namespace peach_manipulation
{

enum class ToolActuatorState : uint8_t
{
  SAFE = 0,
  ARMED = 1,
  CUT_COMMAND_SENT = 2,
  CUT_FEEDBACK_CONFIRMED = 3,
  SAFE_OR_HOLDING = 4
};

struct ToolCommandContext
{
  std::string run_id;
  std::string target_id;
  std::string model_revision;
  std::string contact_transaction_id;
};

class ToolActuator
{
public:
  using SendIo = std::function<bool(int fun, int pin, double state, std::string & reason)>;

  explicit ToolActuator(ToolProfile profile = defaultHollowCylinderV1());

  void setSendIo(SendIo send_io);
  bool arm(const ToolCommandContext & ctx, std::string & reason);
  // SetIO ACK 只产生 CUT_COMMAND_ACCEPTED，不得自称切断确认。
  bool sendCut(std::string & reason);
  // 预留：接 /aubo_io_controller/io_states 工具 DI 后启用（当前无调用点；
  // 切断确认保持删除态，收割终验按保守语义走 CUT_FEEDBACK_TIMEOUT）。
  bool confirmFeedback(bool hardware_ok, std::string & reason);
  void resetSafe();
  bool sameTransaction(const std::string & transaction_id) const;
  bool cutAlreadyCommanded() const;

  ToolActuatorState state() const {return state_;}
  const ToolCommandContext & context() const {return ctx_;}
  const ToolProfile & profile() const {return profile_;}

private:
  ToolProfile profile_;
  ToolActuatorState state_{ToolActuatorState::SAFE};
  ToolCommandContext ctx_;
  SendIo send_io_;
  bool cut_commanded_{false};
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_
