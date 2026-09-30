// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

/// Next-target selection over a locked observation set. Zero ROS: the node converts
/// TargetObservationArray into ObservationSet and injects the CheckReachability answers.
///
/// Eligibility (every rule must hold, preferred ids included):
///   set locked and id listed in locked_target_ids (a locked id without an observation is
///   reported as locked_not_observed), id not claimed, track confirmed, category BAG,
///   geometry present, not touching the image border, mask_quality and depth_coverage
///   above threshold, camera distance inside the depth window (0 = unknown, not filtered).
///
/// Ranking of eligible candidates (first key wins):
///   1. position in the explicit target list (only when one was given);
///   2. reachability known-reachable before unknown (unknown only allowed when
///      require_reachability is false; known-unreachable is never selected);
///   3. height band ascending (lower bags first): the sleeve approaches from below along the
///      bag axis, so lower bags sit in the approach corridor of the ones above them and
///      harvesting them first clears that corridor;
///   4. camera distance ascending: stereo depth sigma grows with z^2, closer is better measured;
///   5. mask ROI area descending: near duplicates (big box + occluded fragment of the same bag)
///      resolve to the big box (old batch.py:196-198, field decision 2026-09-01);
///   6. target_id for determinism.
namespace peach2_task::core
{

constexpr uint8_t kCategoryBag = 0;  ///< TargetObservation.CATEGORY_BAG

struct Candidate
{
  std::string target_id;
  uint8_t category = kCategoryBag;
  bool confirmed = false;
  bool edge_touch = false;
  bool has_geometry = false;       ///< bottom or neck landmark valid
  float mask_quality = 0.0F;       ///< 0..1
  float depth_coverage = 0.0F;     ///< 0..1
  double camera_distance_m = 0.0;  ///< [m]; 0 = unknown
  double height_m = 0.0;           ///< [m] base_link z of neck (bottom if neck invalid)
  double roi_area_px = 0.0;        ///< [px^2]
};

struct ObservationSet
{
  bool target_set_locked = false;
  uint32_t scene_epoch = 0;
  std::vector<std::string> locked_target_ids;  ///< the only selectable ids
  std::vector<Candidate> candidates;
};

struct SelectionConfig
{
  float min_mask_quality = 0.3F;
  float min_depth_coverage = 0.3F;
  double depth_min_m = 0.3;
  double depth_max_m = 1.6;
  double height_band_m = 0.10;
  bool require_reachability = true;  ///< missing reachability answer = not selectable
};

/// Empty string when valid.
std::string validate(const SelectionConfig & config);

struct Reach
{
  bool reachable = false;
  uint32_t failure_code = 0;
  std::string reason;  ///< CheckReachability.reasons[i]
};
using ReachMap = std::map<std::string, Reach>;

struct Eligibility
{
  std::vector<std::string> eligible;             ///< input order, deduplicated
  std::map<std::string, std::string> rejected;   ///< id -> reason token
};

/// `preferred` empty = all targets; otherwise restricts to those ids.
Eligibility filter_eligible(
  const ObservationSet & set, const std::set<std::string> & claimed,
  const std::vector<std::string> & preferred, const SelectionConfig & config);

struct Selection
{
  std::string target_id;                          ///< empty = nothing selectable
  std::vector<std::string> ranked;                ///< selectable ids, best first
  std::map<std::string, std::string> filtered;    ///< eligibility + reachability rejections
  std::map<std::string, Reach> unreachable;       ///< known-unreachable id -> answer
};

Selection select_next(
  const ObservationSet & set, const std::set<std::string> & claimed,
  const std::vector<std::string> & preferred, const ReachMap & reach,
  const SelectionConfig & config);

}  // namespace peach2_task::core
