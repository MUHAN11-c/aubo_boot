// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "peach2_task/core/selection.hpp"

using peach2_task::core::Candidate;
using peach2_task::core::filter_eligible;
using peach2_task::core::kCategoryBag;
using peach2_task::core::ObservationSet;
using peach2_task::core::Reach;
using peach2_task::core::ReachMap;
using peach2_task::core::select_next;
using peach2_task::core::SelectionConfig;

namespace
{

Candidate good(const std::string & id, double height = 1.0, double dist = 0.8, double area = 100)
{
  Candidate c;
  c.target_id = id;
  c.category = kCategoryBag;
  c.confirmed = true;
  c.has_geometry = true;
  c.mask_quality = 0.9F;
  c.depth_coverage = 0.9F;
  c.camera_distance_m = dist;
  c.height_m = height;
  c.roi_area_px = area;
  return c;
}

ObservationSet locked(std::vector<Candidate> cands)
{
  ObservationSet s;
  s.target_set_locked = true;
  for (const auto & c : cands) {
    if (!c.target_id.empty()) {
      s.locked_target_ids.push_back(c.target_id);
    }
  }
  s.candidates = std::move(cands);
  return s;
}

ReachMap all_reachable(const ObservationSet & s)
{
  ReachMap m;
  for (const auto & c : s.candidates) {
    m[c.target_id] = Reach{true, 0, {}};
  }
  return m;
}

}  // namespace

TEST(Selection, EligibilityRejectsEachRule)
{
  std::vector<Candidate> c;
  c.push_back(good("ok"));
  c.push_back(good("claimed"));
  auto unconfirmed = good("unconfirmed");
  unconfirmed.confirmed = false;
  c.push_back(unconfirmed);
  auto nobag = good("nobag");
  nobag.category = 1;
  c.push_back(nobag);
  auto nogeo = good("nogeo");
  nogeo.has_geometry = false;
  c.push_back(nogeo);
  auto edge = good("edge");
  edge.edge_touch = true;
  c.push_back(edge);
  auto lowq = good("lowq");
  lowq.mask_quality = 0.1F;
  c.push_back(lowq);
  auto lowd = good("lowd");
  lowd.depth_coverage = 0.1F;
  c.push_back(lowd);
  auto far = good("far", 1.0, 2.5);
  c.push_back(far);
  auto near = good("near", 1.0, 0.1);
  c.push_back(near);
  auto unknown_dist = good("unknown_dist", 1.0, 0.0);
  c.push_back(unknown_dist);

  const auto e = filter_eligible(locked(c), {"claimed"}, {}, SelectionConfig{});
  EXPECT_EQ(e.eligible, (std::vector<std::string>{"ok", "unknown_dist"}));
  EXPECT_EQ(e.rejected.at("claimed"), "claimed");
  EXPECT_EQ(e.rejected.at("unconfirmed"), "not_confirmed");
  EXPECT_EQ(e.rejected.at("nobag"), "not_bag");
  EXPECT_EQ(e.rejected.at("nogeo"), "no_geometry");
  EXPECT_EQ(e.rejected.at("edge"), "edge_touch");
  EXPECT_EQ(e.rejected.at("lowq"), "low_mask_quality:0.10");
  EXPECT_EQ(e.rejected.at("lowd"), "low_depth_coverage:0.10");
  EXPECT_EQ(e.rejected.at("far"), "out_of_depth_window:2.50");
  EXPECT_EQ(e.rejected.at("near"), "out_of_depth_window:0.10");
}

TEST(Selection, UnlockedSetSelectsNothing)
{
  auto s = locked({good("a")});
  s.target_set_locked = false;
  const auto sel = select_next(s, {}, {}, all_reachable(s), SelectionConfig{});
  EXPECT_TRUE(sel.target_id.empty());
  EXPECT_EQ(sel.filtered.at("a"), "not_locked");
}

TEST(Selection, DuplicateIdsKeepFirstAndEmptyIdsIgnored)
{
  auto dup = good("a");
  dup.edge_touch = true;
  auto noid = good("");
  const auto e = filter_eligible(locked({good("a"), dup, noid}), {}, {}, SelectionConfig{});
  EXPECT_EQ(e.eligible, (std::vector<std::string>{"a"}));
  EXPECT_TRUE(e.rejected.empty());
}

TEST(Selection, PreferredIdsMustPassEligibility)
{
  auto bad = good("b");
  bad.edge_touch = true;
  const auto s = locked({good("a"), bad, good("c")});
  const auto sel = select_next(s, {}, {"b", "c"}, all_reachable(s), SelectionConfig{});
  EXPECT_EQ(sel.target_id, "c");
  EXPECT_EQ(sel.filtered.at("b"), "edge_touch");
  EXPECT_EQ(sel.filtered.at("a"), "not_in_target_list");
}

TEST(Selection, PreferredOrderWinsOverGeometry)
{
  const auto s = locked({good("low", 0.5), good("high", 1.5)});
  const auto sel = select_next(s, {}, {"high", "low"}, all_reachable(s), SelectionConfig{});
  EXPECT_EQ(sel.ranked, (std::vector<std::string>{"high", "low"}));
}

TEST(Selection, LowerBandFirstThenCloserThenBiggerThenId)
{
  const auto s = locked({
    good("high", 1.35, 0.5),
    good("low_far", 1.01, 1.2),
    good("low_near_small", 1.05, 0.7, 50),
    good("low_near_big", 1.09, 0.7, 500),
    good("low_near_big_b", 1.02, 0.7, 500),
    });
  const auto sel = select_next(s, {}, {}, all_reachable(s), SelectionConfig{});
  EXPECT_EQ(
    sel.ranked, (std::vector<std::string>{
    "low_near_big", "low_near_big_b", "low_near_small", "low_far", "high"}));
  EXPECT_EQ(sel.target_id, "low_near_big");
}

TEST(Selection, UnknownDistanceRanksLast)
{
  const auto s = locked({good("unknown", 1.0, 0.0), good("known", 1.0, 1.5)});
  const auto sel = select_next(s, {}, {}, all_reachable(s), SelectionConfig{});
  EXPECT_EQ(sel.target_id, "known");
}

TEST(Selection, UnreachableNeverSelectedAndReported)
{
  const auto s = locked({good("a", 0.5), good("b", 1.5)});
  ReachMap reach{{"a", Reach{false, 30, {}}}, {"b", Reach{true, 0, {}}}};
  const auto sel = select_next(s, {}, {}, reach, SelectionConfig{});
  EXPECT_EQ(sel.target_id, "b");
  EXPECT_EQ(sel.unreachable.at("a").failure_code, 30U);
  EXPECT_EQ(sel.filtered.at("a"), "unreachable:30");
}

TEST(Selection, UnreachableReasonIsKept)
{
  const auto s = locked({good("a")});
  ReachMap reach{{"a", Reach{false, 31, "collision with peach_bag_b"}}};
  const auto sel = select_next(s, {}, {}, reach, SelectionConfig{});
  EXPECT_TRUE(sel.target_id.empty());
  EXPECT_EQ(sel.unreachable.at("a").reason, "collision with peach_bag_b");
  EXPECT_EQ(sel.filtered.at("a"), "unreachable:31:collision with peach_bag_b");
}

TEST(Selection, OnlyLockedIdsAreSelectable)
{
  auto s = locked({good("a", 0.5), good("b", 1.5)});
  s.locked_target_ids = {"b"};
  const auto sel = select_next(s, {}, {}, all_reachable(s), SelectionConfig{});
  EXPECT_EQ(sel.target_id, "b");
  EXPECT_EQ(sel.filtered.at("a"), "not_in_locked_set");
}

TEST(Selection, EmptyLockedSetSelectsNothing)
{
  auto s = locked({good("a")});
  s.locked_target_ids.clear();
  const auto sel = select_next(s, {}, {}, all_reachable(s), SelectionConfig{});
  EXPECT_TRUE(sel.target_id.empty());
  EXPECT_EQ(sel.filtered.at("a"), "not_in_locked_set");
}

TEST(Selection, LockedButUnobservedIsReported)
{
  auto s = locked({good("a")});
  s.locked_target_ids = {"a", "ghost", "done", "other"};
  const auto e = filter_eligible(s, {"done"}, {"a", "ghost"}, SelectionConfig{});
  EXPECT_EQ(e.eligible, (std::vector<std::string>{"a"}));
  EXPECT_EQ(e.rejected.at("ghost"), "locked_not_observed");
  EXPECT_EQ(e.rejected.count("done"), 0U);
  EXPECT_EQ(e.rejected.count("other"), 0U);
}

TEST(Selection, MissingReachabilityIsNotSelectableWhenRequired)
{
  const auto s = locked({good("a")});
  const auto sel = select_next(s, {}, {}, {}, SelectionConfig{});
  EXPECT_TRUE(sel.target_id.empty());
  EXPECT_EQ(sel.filtered.at("a"), "reach_unknown");
}

TEST(Selection, KnownReachableBeforeUnknownWhenNotRequired)
{
  SelectionConfig cfg;
  cfg.require_reachability = false;
  const auto s = locked({good("unknown", 0.5), good("known", 1.5)});
  const auto sel = select_next(s, {}, {}, {{"known", Reach{true, 0, {}}}}, cfg);
  EXPECT_EQ(sel.ranked, (std::vector<std::string>{"known", "unknown"}));
}

TEST(Selection, ClaimedExcluded)
{
  const auto s = locked({good("a", 0.5), good("b", 1.5)});
  const auto sel = select_next(s, {"a"}, {}, all_reachable(s), SelectionConfig{});
  EXPECT_EQ(sel.target_id, "b");
}

TEST(Selection, ValidateConfig)
{
  SelectionConfig c;
  EXPECT_TRUE(validate(c).empty());
  c.depth_max_m = 0.2;
  EXPECT_FALSE(validate(c).empty());
  c = SelectionConfig{};
  c.height_band_m = 0.0;
  EXPECT_FALSE(validate(c).empty());
  c = SelectionConfig{};
  c.min_mask_quality = 1.5F;
  EXPECT_FALSE(validate(c).empty());
}
