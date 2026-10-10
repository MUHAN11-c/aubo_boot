// 功能：末端工具三轴状态机（批次3，2026-09-23 重构）。SetIO ACK ≠ 切断确认。
// 三轴（FINAL_PLAN §12）：刀（blade）/保持（retention）/载荷（payload），
// 各自 UNKNOWN 优先；刀闭合 DI 只是组合证据之一（L86），载荷证据 M0 后接。
// 工具档案（W5-6）：IO 三元组走 params tool.*（节点 commandToolClose 下发），
// 几何/连杆清单走 params tool.links / tool.contact_links / tool.body_*。
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

/// 三轴观测（发布 /peach_arm/tool_state 的数据源）。
struct ToolAxes
{
  uint8_t blade{0};      // ToolState.msg BLADE_*
  uint8_t retention{0};  // RETENTION_*
  uint8_t payload{0};    // PAYLOAD_*
  std::string evidence_source;
  std::string fault_code;
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
  // 批次3：接 /aubo_io_controller/io_states 工具 DI。hardware_ok=本命令后
  // 的新上升沿（早已卡高不算）；沿只升 BLADE_CLOSED——P0 层位的切断证据
  // （闭合≠分离，M0 台架后升级为组合判别）。
  bool confirmFeedback(bool hardware_ok, std::string & reason);
  // DI 帧摄入：pin 电平快照 → 沿检测与三轴投影。返回 true=产生新沿。
  bool ingestToolDi(bool pin_closed_level, std::string & edge_desc);
  void resetSafe();
  void markUnknown();
  bool sameTransaction(const std::string & transaction_id) const;
  bool cutAlreadyCommanded() const;

  ToolActuatorState state() const {return state_;}
  const ToolCommandContext & context() const {return ctx_;}
  ToolAxes axes() const {return axes_;}

private:
  ToolActuatorState state_{ToolActuatorState::SAFE};
  ToolCommandContext ctx_;
  SendIo send_io_;
  bool cut_commanded_{false};
  bool di_pin_last_{false};
  bool di_seen_{false};
  bool new_closed_edge_{false};  // 本命令后是否见过闭合新沿（§12.2）
  ToolAxes axes_;
};

/// 采摘确认（原 tool_txn.hpp 唯一在用函数，随死档案删除迁入）：
/// 切断与撤退双证据才记采摘成功。
inline bool harvestConfirmed(bool cut_evidence, bool retreat_evidence)
{
  return cut_evidence && retreat_evidence;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__TOOL_ACTUATOR_HPP_
