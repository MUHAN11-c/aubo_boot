// 功能：模型身份元组、有效期与许可派生。纯核，零 ROS。
#ifndef PEACH_MANIPULATION__MODEL_CONTRACT_HPP_
#define PEACH_MANIPULATION__MODEL_CONTRACT_HPP_

#include <cstdint>
#include <string>

namespace peach_manipulation
{

enum class Capability : std::uint8_t
{
  Valid = 0,
  Invalid = 1,
  Unknown = 2
};

struct ModelIdentity
{
  std::string run_id;
  std::uint32_t scene_epoch{0};
  std::string target_id;
  std::string model_revision;
  std::string tool_profile_id;
  std::string calibration_revision;
  std::string config_revision;
};

struct ModelSnapshot
{
  ModelIdentity identity;
  double generated_s{0.0};
  double valid_until_s{0.0};
  Capability geometry{Capability::Unknown};
  Capability pregrasp{Capability::Unknown};
  Capability sleeve{Capability::Unknown};
  Capability cut{Capability::Unknown};
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

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__MODEL_CONTRACT_HPP_
