// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/core/selection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <tuple>

namespace peach2_task::core
{

namespace
{

std::string with_value(const char * token, double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%s:%.2f", token, value);
  return buf;
}

}  // namespace

std::string validate(const SelectionConfig & c)
{
  if (!(c.min_mask_quality >= 0.0F && c.min_mask_quality <= 1.0F)) {
    return "min_mask_quality must be in [0, 1]";
  }
  if (!(c.min_depth_coverage >= 0.0F && c.min_depth_coverage <= 1.0F)) {
    return "min_depth_coverage must be in [0, 1]";
  }
  if (!(c.depth_min_m >= 0.0) || !(c.depth_max_m > c.depth_min_m)) {
    return "depth window must satisfy 0 <= depth_min_m < depth_max_m";
  }
  if (!(c.height_band_m > 0.0)) {
    return "height_band_m must be > 0";
  }
  return "";
}

Eligibility filter_eligible(
  const ObservationSet & set, const std::set<std::string> & claimed,
  const std::vector<std::string> & preferred, const SelectionConfig & config)
{
  Eligibility out;
  std::set<std::string> seen;
  const std::set<std::string> wanted(preferred.begin(), preferred.end());
  const std::set<std::string> locked(set.locked_target_ids.begin(), set.locked_target_ids.end());
  for (const auto & c : set.candidates) {
    if (c.target_id.empty() || !seen.insert(c.target_id).second) {
      continue;
    }
    std::string reason;
    if (!set.target_set_locked) {
      reason = "not_locked";
    } else if (locked.count(c.target_id) == 0) {
      reason = "not_in_locked_set";
    } else if (claimed.count(c.target_id) != 0) {
      reason = "claimed";
    } else if (!wanted.empty() && wanted.count(c.target_id) == 0) {
      reason = "not_in_target_list";
    } else if (!c.confirmed) {
      reason = "not_confirmed";
    } else if (c.category != kCategoryBag) {
      reason = "not_bag";
    } else if (!c.has_geometry) {
      reason = "no_geometry";
    } else if (c.edge_touch) {
      reason = "edge_touch";
    } else if (c.mask_quality < config.min_mask_quality) {
      reason = with_value("low_mask_quality", c.mask_quality);
    } else if (c.depth_coverage < config.min_depth_coverage) {
      reason = with_value("low_depth_coverage", c.depth_coverage);
    } else if (c.camera_distance_m > 0.0 &&
      (c.camera_distance_m < config.depth_min_m || c.camera_distance_m > config.depth_max_m))
    {
      reason = with_value("out_of_depth_window", c.camera_distance_m);
    }
    if (reason.empty()) {
      out.eligible.push_back(c.target_id);
    } else {
      out.rejected[c.target_id] = reason;
    }
  }
  if (set.target_set_locked) {
    for (const auto & id : set.locked_target_ids) {
      if (!id.empty() && seen.count(id) == 0 && claimed.count(id) == 0 &&
        (wanted.empty() || wanted.count(id) != 0))
      {
        out.rejected[id] = "locked_not_observed";
      }
    }
  }
  return out;
}

Selection select_next(
  const ObservationSet & set, const std::set<std::string> & claimed,
  const std::vector<std::string> & preferred, const ReachMap & reach,
  const SelectionConfig & config)
{
  Selection out;
  const Eligibility elig = filter_eligible(set, claimed, preferred, config);
  out.filtered = elig.rejected;

  std::map<std::string, const Candidate *> by_id;
  for (const auto & c : set.candidates) {
    by_id.emplace(c.target_id, &c);
  }

  using Key = std::tuple<size_t, int, double, double, double, std::string>;
  std::vector<Key> keys;
  for (const auto & id : elig.eligible) {
    int reach_rank = 1;
    const auto r = reach.find(id);
    if (r != reach.end()) {
      if (!r->second.reachable) {
        out.unreachable[id] = r->second;
        std::string why = "unreachable:" + std::to_string(r->second.failure_code);
        if (!r->second.reason.empty()) {
          why += ":" + r->second.reason;
        }
        out.filtered[id] = why;
        continue;
      }
      reach_rank = 0;
    } else if (config.require_reachability) {
      out.filtered[id] = "reach_unknown";
      continue;
    }
    const Candidate & c = *by_id.at(id);
    size_t pref = 0;
    if (!preferred.empty()) {
      pref = static_cast<size_t>(
        std::find(preferred.begin(), preferred.end(), id) - preferred.begin());
    }
    const double band = std::floor(c.height_m / config.height_band_m);
    const double dist = c.camera_distance_m > 0.0 ?
      c.camera_distance_m : std::numeric_limits<double>::infinity();
    keys.emplace_back(pref, reach_rank, band, dist, -c.roi_area_px, id);
  }
  std::sort(keys.begin(), keys.end());
  for (const auto & k : keys) {
    out.ranked.push_back(std::get<5>(k));
  }
  if (!out.ranked.empty()) {
    out.target_id = out.ranked.front();
  }
  return out;
}

}  // namespace peach2_task::core
