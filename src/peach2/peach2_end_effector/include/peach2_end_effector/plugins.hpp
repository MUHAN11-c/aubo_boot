#pragma once

#include "peach2_end_effector/cutter_end_effector.hpp"

namespace peach2_end_effector
{

/// Scissor shear (shear_v1). Blade assumed on the TCP +X side; roll keeps it within ±60° of
/// the direction pointing away from the branch (known branch direction), else away from the
/// avoid direction, else any roll. TODO(M0): confirm the blade side on the real tool.
class ShearV1 : public CutterEndEffector
{
public:
  static constexpr double kRollHalfWidthRad = 60.0 * M_PI / 180.0;

  RollConstraint roll_constraint(const TargetGeometry & target) const override;
  InsertPlan insert(const TargetGeometry & target) const override;

protected:
  const char * expected_tool_id() const override {return "shear_v1";}
};

/// Bite shear (bite_shear_v1). Jaws close along TCP +Y, which must be perpendicular to the
/// branch: TCP +X aligned with the branch ±20°. Symmetric under a half turn (period pi).
class BiteShearV1 : public CutterEndEffector
{
public:
  static constexpr double kRollHalfWidthRad = 20.0 * M_PI / 180.0;

  RollConstraint roll_constraint(const TargetGeometry & target) const override;
  InsertPlan insert(const TargetGeometry & target) const override;

protected:
  const char * expected_tool_id() const override {return "bite_shear_v1";}
};

/// Adaptive shear (adaptive_shear_v1). Axisymmetric: any roll. Insert is admittance-driven
/// when force sensing exists (TODO(M4)); otherwise linear travel fallback.
class AdaptiveShearV1 : public CutterEndEffector
{
public:
  RollConstraint roll_constraint(const TargetGeometry & target) const override;
  InsertPlan insert(const TargetGeometry & target) const override;

protected:
  const char * expected_tool_id() const override {return "adaptive_shear_v1";}
};

}  // namespace peach2_end_effector
