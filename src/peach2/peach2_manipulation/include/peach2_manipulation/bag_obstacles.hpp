#pragma once

#include <Eigen/Geometry>

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "peach2_end_effector/types.hpp"

namespace peach2_manipulation
{

/// Every collision object this package writes into the planning scene starts with this prefix;
/// nothing else in the scene is ever touched.
inline constexpr char kBagObjectPrefix[] = "peach_bag_";

std::string bag_object_id(const std::string & target_id);
bool is_bag_object_id(const std::string & object_id);

/// Collision stand-in for one bag (主审跨包决定): solid cylinder of radius d95/2 + margin from the
/// bottom to the neck. Flat ends, so the pregrasp mouth just below the bottom stays outside.
struct BagCapsule
{
  std::string id;
  std::string target_id;
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};
  double radius_m{0.0};

  double length_m() const {return (neck - bottom).norm();}
  Eigen::Vector3d center() const {return 0.5 * (bottom + neck);}
  /// Rotation taking +Z onto bottom -> neck (cylinder primitive convention).
  Eigen::Quaterniond orientation() const;
  bool same_geometry(const BagCapsule & other, double tolerance_m) const;
};

/// nullopt for unusable geometry (non-finite, d95 <= 0, bottom == neck) or a negative margin.
std::optional<BagCapsule> bag_capsule(
  const peach2_end_effector::TargetGeometry & model, double margin_m);

/// Capsules for every usable model except `exclude_target_id` (empty = keep all), sorted by id.
std::vector<BagCapsule> desired_bag_capsules(
  const std::vector<peach2_end_effector::TargetGeometry> & models, double margin_m,
  const std::string & exclude_target_id);

/// One atomic planning-scene change: ADD (replaces an existing id) and REMOVE.
struct BagSceneDiff
{
  std::vector<BagCapsule> upsert;
  std::vector<std::string> remove;
  bool empty() const {return upsert.empty() && remove.empty();}
};

/// Bookkeeping of the bag objects currently in the planning scene. Pure: the caller applies a
/// diff and calls commit() only when the scene accepted it.
class BagObstacleSet
{
public:
  explicit BagObstacleSet(double tolerance_m = 1e-4);

  /// Objects to add / replace / remove so the scene holds exactly `desired`.
  BagSceneDiff diff_to(const std::vector<BagCapsule> & desired) const;
  /// Remove every bag object known to be in the scene.
  BagSceneDiff clear_all() const;
  void commit(const BagSceneDiff & diff);
  /// Bag objects found in the scene with unknown geometry (e.g. left by a previous process):
  /// replaced when desired, removed otherwise. Ids without the prefix are ignored.
  void adopt(const std::vector<std::string> & scene_object_ids);
  void forget_all();

  std::size_t size() const {return applied_.size();}
  bool contains(const std::string & id) const {return applied_.count(id) > 0;}

private:
  double tolerance_m_;
  std::map<std::string, std::optional<BagCapsule>> applied_;
};

}  // namespace peach2_manipulation
