#pragma once

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace peach2_end_effector
{

/// Target geometry in base_link [m]. Built by the node from TargetModel / GraspDecision.
struct TargetGeometry
{
  std::string target_id;
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};   ///< unit, bottom -> neck
  double d95_m{0.0};
  double length_m{0.0};
  double sigma_lateral95_m{0.0};
  double sigma_axial95_m{0.0};
  /// Branch / stem direction near the neck (bite jaw alignment). Absent until v2.1 keypoints.
  std::optional<Eigen::Vector3d> branch_direction;
  /// Direction from the bag towards the nearest hard obstacle on the camera side (shear).
  std::optional<Eigen::Vector3d> avoid_direction;
};

/// Permission slice of GraspDecision that a plugin may look at.
struct BudgetView
{
  bool sleeve_allowed{false};
  bool cut_allowed{false};
  double radial_margin_m{0.0};
  double axial_margin_m{0.0};
};

/// Roll = rotation of TCP +X about the bag axis, measured in roll_frame(axis).
/// period_rad = 2*pi for asymmetric tools, pi for tools symmetric under a half turn.
struct RollConstraint
{
  double center_rad{0.0};
  double half_width_rad{M_PI};
  double period_rad{2.0 * M_PI};

  bool full() const {return half_width_rad * 2.0 >= period_rad - 1e-9;}
  bool contains(double roll_rad) const;
  /// n >= 1 rolls, center first, then alternating outward; covers every period copy in [-pi, pi).
  std::vector<double> samples(int n) const;
};

struct Feasibility
{
  bool ok{false};                 ///< geometry fits this tool
  bool sleeve_ok{false};          ///< ok && budget.sleeve_allowed
  bool cut_ok{false};             ///< sleeve_ok && budget.cut_allowed
  uint32_t failure_code{0};
  std::string reason;
  double radial_clearance_m{0.0}; ///< (D_inner - d95)/2 - wall_clearance
  double overshoot_m{0.0};        ///< mouth passes the neck by this much when the blade is on it
  Eigen::Vector3d tcp_at_cut{Eigen::Vector3d::Zero()};
};

enum class InsertMode : uint8_t
{
  LINEAR = 0,       ///< Pilz LIN along the axis, fixed travel
  ADMITTANCE = 1,   ///< force-controlled advance (needs force sensing; TODO(M4))
};

enum class ArrivalCriterion : uint8_t
{
  TRAVEL_COMPLETE = 0,
  TRAVEL_AND_THROAT_CONTACT = 1,
  CONTACT_FORCE_STABLE = 2,
};

struct InsertPlan
{
  InsertMode mode{InsertMode::LINEAR};
  ArrivalCriterion criterion{ArrivalCriterion::TRAVEL_COMPLETE};
  double speed_mps{0.02};
  double dwell_s{0.25};                 ///< settle after arrival (bag swing decay)
  double contact_force_n{0.0};          ///< ADMITTANCE target force
  double force_stable_s{0.0};
  bool requires_force_sensing{false};
  /// LINEAR travel fallback allowed when force sensing is unavailable.
  bool linear_fallback{true};
};

struct ToolResult
{
  bool ok{false};
  uint32_t failure_code{0};
  std::string reason;

  static ToolResult success(const std::string & why = "ok") {return {true, 0U, why};}
  static ToolResult failure(uint32_t code, const std::string & why) {return {false, code, why};}
};

struct CutVerdict
{
  bool confirmed{false};
  uint32_t failure_code{0};
  std::string reason;
  bool feedback_edge{false};
  bool current_checked{false};
  double peak_current_a{0.0};
  double elapsed_s{0.0};
};

/// Same numeric values as peach2_interfaces/msg/ToolState.
enum class ToolState : uint8_t
{
  UNKNOWN = 0,
  OPEN_CONFIRMED = 1,
  CLOSING = 2,
  CLOSED_CONFIRMED = 3,
  OPENING = 4,
  FAULT = 5,
};

const char * to_string(ToolState state);

struct ToolStatus
{
  std::string tool_id;
  ToolState state{ToolState::UNKNOWN};
  bool command_closed{false};
  std::optional<bool> feedback_closed;
  std::optional<double> current_a;
  std::string fault_reason;
  bool suspected_loopback{false};
};

/// Orthonormal (e1, e2) spanning the plane normal to `axis`; e1 is base +X projected
/// (base +Y when the axis is within ~25 deg of X). Deterministic so rolls are comparable.
struct RollFrame
{
  Eigen::Vector3d axis;
  Eigen::Vector3d e1;
  Eigen::Vector3d e2;
};

RollFrame roll_frame(const Eigen::Vector3d & axis);
/// TCP rotation with +Z = axis and +X at `roll_rad` in roll_frame(axis).
Eigen::Matrix3d tcp_rotation(const Eigen::Vector3d & axis, double roll_rad);
/// Roll of `direction` projected onto the plane normal to `axis`; nullopt if nearly parallel.
std::optional<double> roll_of_direction(
  const Eigen::Vector3d & axis, const Eigen::Vector3d & direction);
/// Wrap to [-pi, pi).
double wrap_angle(double a);

}  // namespace peach2_end_effector
