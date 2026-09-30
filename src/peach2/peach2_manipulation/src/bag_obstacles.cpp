#include "peach2_manipulation/bag_obstacles.hpp"

#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace peach2_manipulation
{

namespace
{

constexpr double kMinLengthM = 1e-3;

}  // namespace

std::string bag_object_id(const std::string & target_id)
{
  return std::string(kBagObjectPrefix) + target_id;
}

bool is_bag_object_id(const std::string & object_id)
{
  const std::string prefix(kBagObjectPrefix);
  return object_id.size() > prefix.size() && object_id.compare(0, prefix.size(), prefix) == 0;
}

Eigen::Quaterniond BagCapsule::orientation() const
{
  const Eigen::Vector3d dir = neck - bottom;
  if (dir.norm() < kMinLengthM) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(peach2_end_effector::tcp_rotation(dir.normalized(), 0.0));
}

bool BagCapsule::same_geometry(const BagCapsule & other, double tolerance_m) const
{
  return (bottom - other.bottom).norm() <= tolerance_m &&
         (neck - other.neck).norm() <= tolerance_m &&
         std::fabs(radius_m - other.radius_m) <= tolerance_m;
}

std::optional<BagCapsule> bag_capsule(
  const peach2_end_effector::TargetGeometry & model, double margin_m)
{
  if (!std::isfinite(margin_m) || margin_m < 0.0 || model.target_id.empty()) {
    return std::nullopt;
  }
  if (!model.bottom.allFinite() || !model.neck.allFinite() || !std::isfinite(model.d95_m) ||
    model.d95_m <= 0.0 || (model.neck - model.bottom).norm() < kMinLengthM)
  {
    return std::nullopt;
  }
  BagCapsule c;
  c.id = bag_object_id(model.target_id);
  c.target_id = model.target_id;
  c.bottom = model.bottom;
  c.neck = model.neck;
  c.radius_m = 0.5 * model.d95_m + margin_m;
  return c;
}

std::vector<BagCapsule> desired_bag_capsules(
  const std::vector<peach2_end_effector::TargetGeometry> & models, double margin_m,
  const std::string & exclude_target_id)
{
  std::map<std::string, BagCapsule> by_id;
  for (const auto & m : models) {
    if (!exclude_target_id.empty() && m.target_id == exclude_target_id) {
      continue;
    }
    if (auto c = bag_capsule(m, margin_m)) {
      by_id[c->id] = *c;
    }
  }
  std::vector<BagCapsule> out;
  out.reserve(by_id.size());
  for (auto & [id, c] : by_id) {
    out.push_back(c);
  }
  return out;
}

BagObstacleSet::BagObstacleSet(double tolerance_m)
: tolerance_m_(tolerance_m) {}

BagSceneDiff BagObstacleSet::diff_to(const std::vector<BagCapsule> & desired) const
{
  BagSceneDiff diff;
  std::map<std::string, const BagCapsule *> wanted;
  for (const auto & c : desired) {
    wanted[c.id] = &c;
  }
  for (const auto & [id, c] : wanted) {
    const auto it = applied_.find(id);
    if (it == applied_.end() || !it->second || !it->second->same_geometry(*c, tolerance_m_)) {
      diff.upsert.push_back(*c);
    }
  }
  for (const auto & [id, c] : applied_) {
    if (wanted.count(id) == 0) {
      diff.remove.push_back(id);
    }
  }
  return diff;
}

BagSceneDiff BagObstacleSet::clear_all() const
{
  BagSceneDiff diff;
  for (const auto & [id, c] : applied_) {
    diff.remove.push_back(id);
  }
  return diff;
}

void BagObstacleSet::commit(const BagSceneDiff & diff)
{
  for (const auto & id : diff.remove) {
    applied_.erase(id);
  }
  for (const auto & c : diff.upsert) {
    applied_[c.id] = c;
  }
}

void BagObstacleSet::adopt(const std::vector<std::string> & scene_object_ids)
{
  for (const auto & id : scene_object_ids) {
    if (is_bag_object_id(id) && applied_.count(id) == 0) {
      applied_[id] = std::nullopt;
    }
  }
}

void BagObstacleSet::forget_all()
{
  applied_.clear();
}

}  // namespace peach2_manipulation
