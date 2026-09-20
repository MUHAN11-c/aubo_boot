// 功能：刀具 GPIO 状态机。SetIO ACK ≠ 切断确认。
// 工具档案（W5-6）：IO 三元组走 params tool.*（节点 commandToolClose 下发），
// 几何/连杆清单走 params tool.links / tool.contact_links / tool.body_*；
// 原 ToolProfile 死档案已删（几何字段无运行期消费点）。
#ifndef PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_
#define PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_

#include <cstdint>
#include <functional>
#include <string>

namespace peach_arm
{

enum class ToolActuatorState : uint8_t
{
  SAFE = 0,
  ARMED = 1,
  CUT_COMMAND_SENT = 2,
  CUT_FEEDBACK_CONFIRMED = 3,
  SAFE_OR_HOLDING = 4,
  UNKNOWN = 5
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
  // IO 下发回调：实现内完成授权矩阵 TOOL 级检查与 SetIO 调用；
  // 失败时置 reason 并返回 false（IO 三元组由实现侧参数提供）。
  using SendIo = std::function<bool(std::string & reason)>;

  ToolActuator() = default;

  void setSendIo(SendIo send_io);
  bool arm(const ToolCommandContext & ctx, std::string & reason);
  // SetIO ACK 只产生 CUT_COMMAND_ACCEPTED，不得自称切断确认。
  bool sendCut(std::string & reason);
  // 预留：接 /aubo_io_controller/io_states 工具 DI 后启用（当前无调用点；
  // 切断确认保持删除态，收割终验按保守语义走 CUT_FEEDBACK_TIMEOUT）。
  bool confirmFeedback(bool hardware_ok, std::string & reason);
  void resetSafe();
  void markUnknown();
  bool sameTransaction(const std::string & transaction_id) const;
  bool cutAlreadyCommanded() const;

  ToolActuatorState state() const {return state_;}
  const ToolCommandContext & context() const {return ctx_;}

private:
  ToolActuatorState state_{ToolActuatorState::SAFE};
  ToolCommandContext ctx_;
  SendIo send_io_;
  bool cut_commanded_{false};
};

/// 采摘确认（原 tool_txn.hpp 唯一在用函数，随死档案删除迁入）：
/// 切断与撤退双证据才记采摘成功（cut 证据=硬件反馈确认，现行预留 false）。
inline bool harvestConfirmed(bool cut_evidence, bool retreat_evidence)
{
  return cut_evidence && retreat_evidence;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_
