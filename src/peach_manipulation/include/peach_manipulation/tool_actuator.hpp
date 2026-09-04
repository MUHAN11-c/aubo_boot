// 功能：刀具 GPIO 状态机。SetIO ACK ≠ 切断确认。
#ifndef PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_
#define PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_

#include <cstdint>
#include <functional>
#include <string>

namespace peach_manipulation
{

// hollow_cylinder_v1 静态档案；与 aubo_description/config 对齐。
struct ToolProfile
{
  std::string profile_id{"hollow_cylinder_v1"};
  std::string version{"1.1"};
  double d_inner{0.104};
  double d_outer{0.120};
  double l_insert{0.200};
  double l_blade{0.0};
  double wall_clearance{0.002};
  double blade_capture_half_width{0.008};
  double fruit_safety_clearance{0.012};
  double axial_safety_margin{0.004};
  double tool_runout95{0.001};
  int io_fun{3};
  int io_pin{0};
  double close_state{1.0};
  double feedback_timeout_s{1.5};
};

inline ToolProfile defaultHollowCylinderV1()
{
  return ToolProfile{};
}

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
