// 功能：动作终局分类（RUNNING / SUCCEEDED / CANCELED / FAILED / RECOVERY_REQUIRED）。
#ifndef PEACH_MANIPULATION__ACTION_CONTRACT_HPP_
#define PEACH_MANIPULATION__ACTION_CONTRACT_HPP_

namespace peach_manipulation
{
// action 终局分类。状态枚举与 terminalOutcome(CycleState) 见 cycle_state.hpp；
// 终局判定只走枚举，不再从状态字符串反推。
enum class CycleOutcome {RUNNING, SUCCEEDED, CANCELED, FAILED, RECOVERY_REQUIRED};
}  // namespace peach_manipulation
#endif  // PEACH_MANIPULATION__ACTION_CONTRACT_HPP_
