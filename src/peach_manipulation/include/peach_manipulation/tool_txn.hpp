// 功能：刀具事务 UNARMED→…→UNKNOWN/FAILED。超时不自动撤退/重发。
#ifndef PEACH_MANIPULATION__TOOL_TXN_HPP_
#define PEACH_MANIPULATION__TOOL_TXN_HPP_

#include <cstdint>
#include <string>

namespace peach_manipulation
{

enum class ToolTxnState : std::uint8_t
{
  Unarmed = 0,
  Armed = 1,
  CommandSent = 2,
  Confirmed = 3,
  Unknown = 4,
  Failed = 5
};

struct ToolTxn
{
  ToolTxnState state{ToolTxnState::Unarmed};
  std::string transaction_id;
  bool cut_evidence{false};
  bool retreat_evidence{false};
};

inline ToolTxnState onSetIoTimeout(ToolTxnState)
{
  return ToolTxnState::Unknown;
}

inline bool mayResendCut(ToolTxnState state)
{
  return state == ToolTxnState::Armed;
}

inline bool mayAutoRetreatOnTimeout(ToolTxnState)
{
  return false;
}

inline bool harvestConfirmed(bool cut_evidence, bool retreat_evidence)
{
  return cut_evidence && retreat_evidence;
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__TOOL_TXN_HPP_
