#include "peach2_end_effector/types.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace peach2_end_effector
{

double wrap_angle(double a)
{
  double w = std::fmod(a + M_PI, 2.0 * M_PI);
  if (w < 0.0) {
    w += 2.0 * M_PI;
  }
  return w - M_PI;
}

bool RollConstraint::contains(double roll_rad) const
{
  if (full()) {
    return true;
  }
  const double d = std::fmod(std::fabs(wrap_angle(roll_rad - center_rad)), period_rad);
  const double dist = std::min(d, period_rad - d);
  return dist <= half_width_rad + 1e-9;
}

std::vector<double> RollConstraint::samples(int n) const
{
  std::vector<double> out;
  if (n <= 0) {
    return out;
  }
  if (full()) {
    const double step = period_rad / static_cast<double>(n);
    for (int i = 0; i < n; ++i) {
      // Alternate around the center so truncation keeps the preferred roll first.
      const int k = (i % 2 == 0) ? i / 2 : -(i + 1) / 2;
      out.push_back(wrap_angle(center_rad + k * step));
    }
    return out;
  }
  const int copies = static_cast<int>(std::lround(2.0 * M_PI / period_rad));
  const int per_copy = std::max(1, n / std::max(1, copies));
  const double step = per_copy > 1 ? (2.0 * half_width_rad) / (per_copy - 1) : 0.0;
  for (int c = 0; c < copies; ++c) {
    const double center = center_rad + c * period_rad;
    for (int i = 0; i < per_copy; ++i) {
      const int k = (i % 2 == 0) ? i / 2 : -(i + 1) / 2;
      double offset = k * step;
      offset = std::clamp(offset, -half_width_rad, half_width_rad);
      out.push_back(wrap_angle(center + offset));
    }
  }
  return out;
}

const char * to_string(ToolState state)
{
  switch (state) {
    case ToolState::UNKNOWN: return "UNKNOWN";
    case ToolState::OPEN_CONFIRMED: return "OPEN_CONFIRMED";
    case ToolState::CLOSING: return "CLOSING";
    case ToolState::CLOSED_CONFIRMED: return "CLOSED_CONFIRMED";
    case ToolState::OPENING: return "OPENING";
    case ToolState::FAULT: return "FAULT";
  }
  return "INVALID";
}

RollFrame roll_frame(const Eigen::Vector3d & axis_in)
{
  RollFrame f;
  f.axis = axis_in.normalized();
  Eigen::Vector3d ref = Eigen::Vector3d::UnitX();
  if (std::fabs(f.axis.dot(ref)) > 0.9) {
    ref = Eigen::Vector3d::UnitY();
  }
  f.e1 = (ref - ref.dot(f.axis) * f.axis).normalized();
  f.e2 = f.axis.cross(f.e1);
  return f;
}

Eigen::Matrix3d tcp_rotation(const Eigen::Vector3d & axis, double roll_rad)
{
  const RollFrame f = roll_frame(axis);
  const Eigen::Vector3d x = std::cos(roll_rad) * f.e1 + std::sin(roll_rad) * f.e2;
  const Eigen::Vector3d y = f.axis.cross(x);
  Eigen::Matrix3d r;
  r.col(0) = x;
  r.col(1) = y;
  r.col(2) = f.axis;
  return r;
}

std::optional<double> roll_of_direction(
  const Eigen::Vector3d & axis, const Eigen::Vector3d & direction)
{
  const RollFrame f = roll_frame(axis);
  const Eigen::Vector3d perp = direction - direction.dot(f.axis) * f.axis;
  if (!perp.allFinite() || perp.norm() < 0.1 * std::max(direction.norm(), 1e-12)) {
    return std::nullopt;
  }
  return std::atan2(perp.dot(f.e2), perp.dot(f.e1));
}

}  // namespace peach2_end_effector
