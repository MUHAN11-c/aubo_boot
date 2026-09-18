// 功能：模型身份元组、有效期与许可派生。纯核，零 ROS。
#ifndef PEACH_MANIPULATION__MODEL_CONTRACT_HPP_
#define PEACH_MANIPULATION__MODEL_CONTRACT_HPP_

#include <cstdint>
#include <string>

namespace peach_arm
{

enum class Capability : std::uint8_t
{
  Valid = 0,     ///< 该项可执行。
  Invalid = 1,   ///< 该项明确不可。
  Unknown = 2    ///< 尚未判定。
};

/// 模型身份元组：须全字段非空才 complete；心跳不得改 valid_until。
struct ModelIdentity
{
  std::string run_id;                 ///< 批次 harvest_run_id。
  std::uint32_t scene_epoch{0};       ///< BeginScene 世代。
  std::string target_id;              ///< 绑定目标。
  std::string model_revision;         ///< 模型修订。
  std::string tool_profile_id;        ///< 工具剖面。
  std::string calibration_revision;   ///< 标定修订。
  std::string config_revision;        ///< 配置修订。
};

struct ModelSnapshot
{
  ModelIdentity identity;             ///< 身份元组。
  double generated_s{0.0};            ///< 生成时刻 [s]（注入时钟）。
  double valid_until_s{0.0};          ///< 过期时刻 [s]；心跳不得续签。
  Capability geometry{Capability::Unknown};   ///< 融合几何能力。
  Capability pregrasp{Capability::Unknown};   ///< 预抓取能力（不进 allowed）。
  Capability sleeve{Capability::Unknown};     ///< 套入能力。
  Capability cut{Capability::Unknown};        ///< 剪切能力。
};

inline bool identityComplete(const ModelIdentity & identity)
{
  return !identity.run_id.empty() &&
         !identity.target_id.empty() &&
         !identity.model_revision.empty() &&
         !identity.tool_profile_id.empty() &&
         !identity.calibration_revision.empty() &&
         !identity.config_revision.empty();
}

inline bool identitiesMatch(const ModelIdentity & expected, const ModelIdentity & actual)
{
  return expected.run_id == actual.run_id &&
         expected.scene_epoch == actual.scene_epoch &&
         expected.target_id == actual.target_id &&
         expected.model_revision == actual.model_revision &&
         expected.tool_profile_id == actual.tool_profile_id &&
         expected.calibration_revision == actual.calibration_revision &&
         expected.config_revision == actual.config_revision;
}

inline bool allowedFromCapabilities(
  Capability geometry, Capability sleeve, Capability cut)
{
  return geometry == Capability::Valid &&
         sleeve == Capability::Valid &&
         cut == Capability::Valid;
}

/// 心跳不得改 valid_until；调用方只在 finalize/模型更新时写入。
inline bool heartbeatRenewsValidity()
{
  return false;
}

inline bool modelExecutable(
  const ModelSnapshot & model, double now_s, bool preview)
{
  if (preview) {
    return true;
  }
  if (!identityComplete(model.identity)) {
    return false;
  }
  if (model.valid_until_s <= model.generated_s) {
    return false;
  }
  return now_s <= model.valid_until_s;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__MODEL_CONTRACT_HPP_
