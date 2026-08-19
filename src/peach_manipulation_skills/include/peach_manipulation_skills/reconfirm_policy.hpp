// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef PEACH_MANIPULATION_SKILLS__RECONFIRM_POLICY_HPP_
#define PEACH_MANIPULATION_SKILLS__RECONFIRM_POLICY_HPP_

#include <Eigen/Geometry>

#include <string>

namespace peach_manipulation_skills
{

// 抓取前再确认策略（2.7-RECONFIRM）纯核：身份一致性、锚点漂移容差、摆动平息
// 判定与超限计数。等待窗口/帧率自适应在 BT 节点体（btReconfirmTarget），本类
// 只做逐样本判定，零 ROS、零阻塞，可单测。

struct ReconfirmConfig
{
  double tolerance_m{0.03};    // 锚点漂移容差（米）
  int max_attempts{3};         // 累计超限放弃阈值（次）
  bool allow_stale_anchor{false};  // 回退开关：窗口耗尽时按静态锚点放行（验证期遗留）
};

// 单次再确认样本：fresh=false 表示该尝试窗口耗尽（未等到新鲜观测），其余字段
// 仅在 fresh=true 时有意义。anchor_drift_m 由节点体按 |最新锚点−参考锚点| 计算。
struct ReconfirmSample
{
  bool fresh{false};
  bool identity_ok{false};    // 观测身份与周期钉死 ID 一致
  bool swinging{false};       // target_swinging 诊断旗标
  double anchor_drift_m{0.0};
  Eigen::Vector3d anchor{Eigen::Vector3d::Zero()};  // 最新观测锚点（REFINED 重算用）
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};   // 最新观测轴

  // 窗口耗尽样本（未等到新鲜观测）。
  static ReconfirmSample exhausted() {return ReconfirmSample{};}
};

enum class ReconfirmVerdict
{
  PENDING,   // 继续等下一窗口/下一帧（摆动等平息中）
  PASS,      // 再确认通过（含 allow_stale_anchor 的静态锚点回退放行）
  REFINED,   // 漂移超限：已用最新锚点重算一次，重新复核
  ABORT      // 放弃：reason 给出原因（身份变更/持续摆动）
};

struct ReconfirmDecision
{
  ReconfirmVerdict verdict{ReconfirmVerdict::PENDING};
  std::string reason;
};

// 逐样本判定：
// - 身份不一致 → 立即 ABORT（不计超限，由周期钉死语义兜底失败）；
// - 窗口耗尽 → allow_stale_anchor 时 PASS（旧"按静态锚点继续"行为，验证期遗留）；
//   否则计一次超限；
// - 漂移超限 → REFINED（节点体用样本锚点重算 entry/axis 一次）并计一次超限；
// - 摆动（target_swinging）→ PENDING 等平息，须残差低于容差且连续 2 帧干净
//   才 PASS（摆动本身不计超限，摆动持续导致窗口耗尽才计）；
// - 累计超限达 max_attempts → ABORT，reason 按真实致因区分（"锚点漂移超限"/
//   "观测窗口耗尽"，曾见摆动旗标时冠以"目标持续摆动"）；SKIPPED_QUALITY 分级
//   由节点体落地。
class ReconfirmPolicy
{
public:
  explicit ReconfirmPolicy(ReconfirmConfig config)
  : config_(config)
  {
  }

  ReconfirmDecision check(const ReconfirmSample & sample)
  {
    if (sample.fresh && !sample.identity_ok) {
      return {ReconfirmVerdict::ABORT, "目标身份变更：再确认观测与周期钉死 ID 不一致"};
    }
    if (!sample.fresh) {
      if (config_.allow_stale_anchor) {
        // 回退开关（验证期遗留，室外默认关闭）：等不到新鲜观测时按静态目标
        // 锚点继续，准确性现场评估。
        return {ReconfirmVerdict::PASS,
          "allow_stale_anchor=true：未等到新鲜观测，按静态目标锚点继续（旧行为）"};
      }
      return countStrike("再确认窗口耗尽，未获得新鲜观测");
    }
    if (sample.anchor_drift_m > config_.tolerance_m) {
      calm_frames_ = 0;
      saw_swinging_ = saw_swinging_ || sample.swinging;
      ++strikes_;
      if (strikes_ >= config_.max_attempts) {
        // 致因文案区分：曾见摆动旗标时摆动感光上更明显，否则直报漂移超限。
        return {ReconfirmVerdict::ABORT,
          saw_swinging_ ?
          "目标持续摆动且锚点漂移超限（累计 " + std::to_string(strikes_) +
          " 次超容差）" :
          "锚点漂移超限：累计 " + std::to_string(strikes_) + " 次超容差"};
      }
      // 漂移超限：用最新锚点重算 entry/axis 一次（节点体执行），重新复核。
      return {ReconfirmVerdict::REFINED,
        "锚点漂移 " + std::to_string(sample.anchor_drift_m) +
        "m 超容差，已按最新锚点重算入口/轴（第 " + std::to_string(strikes_) + " 次）"};
    }
    if (sample.swinging) {
      // 目标摆动：等平息——要求残差低于容差且连续 2 帧干净才放行。
      saw_swinging_ = true;
      calm_frames_ = 0;
      return {ReconfirmVerdict::PENDING, "目标摆动中，等待平息"};
    }
    ++calm_frames_;
    if (saw_swinging_ && calm_frames_ < 2) {
      return {ReconfirmVerdict::PENDING, "摆动后首帧干净，再等一帧确认平息"};
    }
    return {ReconfirmVerdict::PASS, "再确认通过：身份一致且锚点漂移在容差内"};
  }

  int strikes() const {return strikes_;}

private:
  ReconfirmDecision countStrike(const std::string & what)
  {
    calm_frames_ = 0;
    ++strikes_;
    if (strikes_ >= config_.max_attempts) {
      return {ReconfirmVerdict::ABORT,
        saw_swinging_ ?
        "目标持续摆动：" + what + "（累计 " + std::to_string(strikes_) + " 次）" :
        what + "（累计 " + std::to_string(strikes_) + " 次）"};
    }
    return {ReconfirmVerdict::PENDING,
      what + "（第 " + std::to_string(strikes_) + " 次），重开窗口再试"};
  }

  ReconfirmConfig config_;
  int strikes_{0};
  int calm_frames_{0};
  bool saw_swinging_{false};
};

}  // namespace peach_manipulation_skills
#endif  // PEACH_MANIPULATION_SKILLS__RECONFIRM_POLICY_HPP_
