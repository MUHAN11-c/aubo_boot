#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>

#include "peach2_end_effector/types.hpp"

namespace peach2_manipulation
{

/// GraspDecision as seen by the cycle. Times are on the decision clock (DecisionClient::now_s).
struct DecisionView
{
  std::string target_id;
  std::string tool_id;
  uint64_t revision{0};
  double valid_until_s{0.0};
  bool approach_allowed{false};
  bool sleeve_allowed{false};
  bool cut_allowed{false};
  double radial_margin_m{0.0};
  double axial_margin_m{0.0};
  Eigen::Isometry3d pregrasp_tcp{Eigen::Isometry3d::Identity()};  ///< +Z = bag axis
  Eigen::Vector3d blade_target{Eigen::Vector3d::Zero()};          ///< neck, base_link
  double insert_travel_m{0.0};
  uint32_t failure_code{0};
  std::string reason;
};

/// Synchronous GetDecision. Never cached across queries: the cycle asks again before INSERT
/// and before CUT so a permission cannot outlive the model it was computed from.
class DecisionClient
{
public:
  virtual ~DecisionClient() = default;
  virtual std::optional<DecisionView> get(
    const std::string & target_id, const std::string & tool_id, uint64_t min_revision) = 0;
  virtual double now_s() = 0;
};

/// Latest TargetModel geometry by id (bottom / axis / d95 / length for plugin feasibility).
class TargetSource
{
public:
  virtual ~TargetSource() = default;
  virtual std::optional<peach2_end_effector::TargetGeometry> get(const std::string & target_id) =
  0;
};

}  // namespace peach2_manipulation
