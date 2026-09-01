// 功能：执行前安全门抽象。robotReady 是硬件门，实现不得旁路或恒放行。
// require_robot_status=false 是部署配置，不算实现层旁路。
#ifndef PEACH_MANIPULATION__SAFETY_GATE_BASE_HPP_
#define PEACH_MANIPULATION__SAFETY_GATE_BASE_HPP_

#include <string>

namespace peach_manipulation
{

// robot_status 的纯值快照：节点把 int8 消息字段转成布尔后传入，纯核不认识消息类型。
struct RobotStatusSample
{
  bool received{false};    // 是否已收到过 robot_status
  double received_s{0.0};  // 接收时刻（秒，与注入时钟同源）
  bool e_stopped{false};
  bool in_error{false};
  bool drives_powered{false};
  bool motion_possible{false};
};

// 周期目标的纯值快照：id 在观测无效时也可能非空（身份一致性判定先于有效性判定）。
struct TargetGateSample
{
  std::string id;
  bool valid{false};
  double received_s{0.0};
};

// 执行前安全门抽象基类：机器人状态门 + 周期目标门。
// 用途：所有运动下发前判定硬件状态与目标身份/新鲜度。
// 生命周期：由节点构造期/参数重载时重建，unique_ptr 独占持有；
//   时钟由实现构造时注入（秒），实现内部不得直接读系统时钟。
// 线程安全：判定方法为 const，可被周期工作线程与 executor 回调并发调用；
//   set_target_observation_max_age_s 由订阅回调（executor 线程）调用，实现
//   须保证与判定方法的并发安全（默认实现为平凡双精度写，沿用既有语义）。
// 可替换性：唯一实现 SafetyGate（节点直接构造）。
class SafetyGateBase
{
public:
  virtual ~SafetyGateBase() = default;

  // 机器人状态门（I5：任何实现不得旁路）。
  // 前置：sample 由调用端在锁内组装，时刻与实现注入时钟同源。
  // 后置：返回 false 时 reason 为稳定原因串（robot_status_missing/stale/
  //   not_motion_ready 等），调用端据此中止运动。
  virtual bool robotReady(
    const RobotStatusSample & sample, std::string & reason) const = 0;
  // 周期目标门：目标身份一致、观测有效、未超龄。
  virtual bool targetReady(
    const TargetGateSample & sample, const std::string & target_id,
    std::string & reason) const = 0;
  // 运行期按实测帧率自适应调整目标观测新鲜度上限。
  virtual void set_target_observation_max_age_s(double value) = 0;
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__SAFETY_GATE_BASE_HPP_
