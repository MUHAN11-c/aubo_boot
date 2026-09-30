// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "peach2_task/core/batch_session.hpp"

namespace core = peach2_task::core;
namespace fc = peach2_task::core::fc;
namespace fs = std::filesystem;
using core::Intent;
using core::Policy;

namespace
{

core::Candidate bag(const std::string & id, double height)
{
  core::Candidate c;
  c.target_id = id;
  c.confirmed = true;
  c.has_geometry = true;
  c.mask_quality = 0.9F;
  c.depth_coverage = 0.9F;
  c.camera_distance_m = 0.8;
  c.height_m = height;
  c.roi_area_px = 100.0;
  return c;
}

core::ObservationSet scene(std::vector<core::Candidate> c)
{
  core::ObservationSet s;
  s.target_set_locked = true;
  for (const auto & cand : c) {
    s.locked_target_ids.push_back(cand.target_id);
  }
  s.candidates = std::move(c);
  return s;
}

core::ReachMap reachable(const std::vector<std::string> & ids)
{
  core::ReachMap m;
  for (const auto & id : ids) {
    m[id] = core::Reach{true, 0, {}};
  }
  return m;
}

core::HarvestOutcome harvest_ok(const std::string & id)
{
  core::HarvestOutcome h;
  h.target_id = id;
  h.outcome = core::outcome::SUCCEEDED;
  h.reached = core::reached::PREGRASP;
  return h;
}

core::HarvestOutcome plan_only(const std::string & id)
{
  auto h = harvest_ok(id);
  h.reached = core::reached::NONE;
  h.plan_only = true;
  return h;
}

class SessionTest : public ::testing::Test
{
protected:
  SessionTest()
  : session_([this] {return t_;}, [] {
        return std::chrono::system_clock::time_point(std::chrono::seconds(1790752620));
      }) {}

  void begin(Intent intent, core::BatchLimits limits = {}, std::vector<std::string> ids = {})
  {
    core::BatchRequest r;
    r.request_id = "req";
    r.intent = intent;
    r.tool_id = "adaptive_shear_v1";
    r.target_ids = std::move(ids);
    r.limits = limits;
    session_.begin(r, cfg_, std::move(ledger_));
  }

  double t_ = 100.0;
  core::SessionConfig cfg_;
  std::unique_ptr<core::Ledger> ledger_;
  core::BatchSession session_;
};

}  // namespace

TEST_F(SessionTest, DiscoveredIsUnionOfEligibleIds)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2)}));
  auto edge = bag("c", 1.0);
  edge.edge_touch = true;
  session_.on_locked_snapshot(scene({bag("b", 1.2), bag("d", 1.4), edge}));
  EXPECT_EQ(session_.counts().discovered, 3U);
  EXPECT_EQ(session_.reach_query_ids(), (std::vector<std::string>{"b", "d"}));
}

TEST_F(SessionTest, SelectClaimsAndStartsTarget)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.3), bag("b", 1.05)}));
  EXPECT_EQ(session_.select(reachable({"a", "b"})), "b");
  EXPECT_EQ(session_.current_target(), "b");
  EXPECT_EQ(session_.reach_query_ids(), (std::vector<std::string>{"a"}));
  EXPECT_EQ(session_.select({{"a", core::Reach{false, fc::PLAN_NO_IK, {}}}}), "");
  EXPECT_EQ(session_.last_filtered().at("a"), "unreachable:30");
}

TEST_F(SessionTest, DecisionLevelsAreCumulative)
{
  begin(Intent::FULL);
  core::DecisionOutcome d;
  d.found = true;
  d.approach_allowed = true;
  d.sleeve_allowed = false;
  d.cut_allowed = true;
  EXPECT_TRUE(session_.on_decision(d, core::DecisionLevel::APPROACH));
  EXPECT_FALSE(session_.on_decision(d, core::DecisionLevel::CUT));
  EXPECT_EQ(session_.failure().code, fc::BUDGET_AXIAL_NEGATIVE);
  EXPECT_FALSE(session_.on_decision(d, core::DecisionLevel::SLEEVE));
  EXPECT_EQ(session_.failure().code, fc::BUDGET_RADIAL_NEGATIVE);

  d.approach_allowed = false;
  d.failure_code = fc::SWING_TOO_LARGE;
  d.reason = "swing";
  EXPECT_FALSE(session_.on_decision(d, core::DecisionLevel::APPROACH));
  EXPECT_EQ(session_.failure().code, fc::SWING_TOO_LARGE);
  EXPECT_EQ(session_.failure().reason, "swing");
  EXPECT_EQ(session_.failure().policy(), Policy::WAIT);

  core::DecisionOutcome missing;
  EXPECT_FALSE(session_.on_decision(missing, core::DecisionLevel::APPROACH));
  EXPECT_EQ(session_.failure().code, fc::MODEL_STALE);
  core::DecisionOutcome expired;
  expired.found = true;
  expired.expired = true;
  expired.approach_allowed = true;
  EXPECT_FALSE(session_.on_decision(expired, core::DecisionLevel::APPROACH));
  EXPECT_EQ(session_.failure().code, fc::MODEL_EXPIRED);
}

TEST_F(SessionTest, ObserveAndHarvestMapping)
{
  begin(Intent::FULL);
  core::ObserveOutcome o;
  EXPECT_FALSE(session_.on_observe(o));
  EXPECT_EQ(session_.failure().code, fc::MODEL_NOT_CONVERGED);
  o.converged = true;
  o.model_revision = 9;
  EXPECT_TRUE(session_.on_observe(o));
  EXPECT_EQ(session_.model_revision(), 9U);

  core::HarvestOutcome h;
  h.outcome = core::outcome::CANCELED;
  EXPECT_FALSE(session_.on_harvest(h));
  EXPECT_EQ(session_.failure().code, fc::CANCELED);
  h.outcome = core::outcome::FAILED;
  h.recovery_required = true;
  EXPECT_FALSE(session_.on_harvest(h));
  EXPECT_EQ(session_.failure().code, fc::EXEC_FAILED);
  EXPECT_EQ(session_.failure().policy(), Policy::RECOVER);
  h = harvest_ok("x");
  h.failure_code = fc::CONTACT_ABORT;
  EXPECT_FALSE(session_.on_harvest(h));
}

TEST_F(SessionTest, PregraspCheckpointNeedsAck)
{
  begin(Intent::PREGRASP_ONLY);
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2)}));
  ASSERT_EQ(session_.select(reachable({"a", "b"})), "a");
  session_.begin_attempt();
  ASSERT_TRUE(session_.on_harvest(harvest_ok("a")));
  session_.record_success();
  EXPECT_TRUE(session_.recovery_required());
  EXPECT_EQ(session_.recovery_reason(), "pregrasp_checkpoint");
  EXPECT_FALSE(session_.consume_ack());
  EXPECT_TRUE(session_.grant_ack());
  EXPECT_TRUE(session_.consume_ack());
  EXPECT_FALSE(session_.recovery_required());
  EXPECT_FALSE(session_.grant_ack());
  EXPECT_EQ(session_.counts().succeeded, 1U);
  EXPECT_EQ(session_.results().front().reached, core::reached::PREGRASP);
}

TEST_F(SessionTest, NoCheckpointWhenDisabledOrFull)
{
  cfg_.ack_each_pregrasp = false;
  begin(Intent::PREGRASP_ONLY);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.on_harvest(harvest_ok("a"));
  session_.record_success();
  EXPECT_FALSE(session_.recovery_required());
}

TEST_F(SessionTest, ManipulationRequestedAckOnSuccess)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  auto h = harvest_ok("a");
  h.recovery_required = true;
  session_.on_harvest(h);
  session_.record_success();
  EXPECT_EQ(session_.recovery_reason(), "manipulation_requested_ack");
}

TEST_F(SessionTest, SkipPoliciesAndStopBatch)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2), bag("c", 1.4)}));

  ASSERT_EQ(session_.select(reachable({"a", "b", "c"})), "a");
  session_.set_failure(core::Failure{fc::BUDGET_RADIAL_NEGATIVE, "wide", false, {}, {}});
  EXPECT_TRUE(session_.record_skip());
  EXPECT_EQ(session_.results().back().outcome, core::outcome::SKIPPED);
  EXPECT_EQ(session_.rework().back().kind, "tool");

  ASSERT_EQ(session_.select(reachable({"b", "c"})), "b");
  session_.set_failure(core::Failure{fc::TOOL_FAULT, "fault", false, {}, {}});
  EXPECT_TRUE(session_.record_skip());
  EXPECT_EQ(session_.results().back().outcome, core::outcome::FAILED);
  EXPECT_TRUE(session_.recovery_required());
  ASSERT_TRUE(session_.grant_ack());
  ASSERT_TRUE(session_.consume_ack());

  ASSERT_EQ(session_.select(reachable({"c"})), "c");
  session_.set_failure(core::Failure{fc::ROBOT_NOT_READY, "drives", false, {}, {}});
  EXPECT_FALSE(session_.record_skip());
  EXPECT_EQ(session_.abort_reason(), "stop_batch:ROBOT_NOT_READY:drives");
  EXPECT_FALSE(session_.gate_allows_next());
  EXPECT_EQ(session_.counts().skipped, 1U);
  EXPECT_EQ(session_.counts().failed, 2U);
}

TEST_F(SessionTest, RetryVerdicts)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.set_failure(core::Failure{fc::MODEL_STALE, "", false, {}, {}});
  EXPECT_EQ(session_.retry_verdict(1, 2), core::RetryVerdict::RETRY_NOW);
  EXPECT_EQ(session_.retry_verdict(2, 2), core::RetryVerdict::GIVE_UP);
  session_.set_failure(core::Failure{fc::SWING_TOO_LARGE, "", false, {}, {}});
  EXPECT_EQ(session_.retry_verdict(1, 2), core::RetryVerdict::RETRY_AFTER_WAIT);
  session_.set_failure(core::Failure{fc::PLAN_NO_IK, "", false, {}, {}});
  EXPECT_EQ(session_.retry_verdict(1, 2), core::RetryVerdict::GIVE_UP);
  session_.set_failure(core::Failure{fc::MODEL_STALE, "", true, {}, {}});
  EXPECT_EQ(session_.retry_verdict(1, 2), core::RetryVerdict::GIVE_UP);
}

TEST_F(SessionTest, DeadlineBlocksRetry)
{
  core::BatchLimits l;
  l.per_target_timeout_s = 10.0;
  begin(Intent::FULL, l);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.set_failure(core::Failure{fc::MODEL_STALE, "", false, {}, {}});
  t_ += 9.0;
  EXPECT_FALSE(session_.target_deadline_exceeded());
  EXPECT_EQ(session_.retry_verdict(1, 2), core::RetryVerdict::RETRY_NOW);
  t_ += 1.0;
  EXPECT_TRUE(session_.target_deadline_exceeded());
  EXPECT_EQ(session_.retry_verdict(1, 2), core::RetryVerdict::GIVE_UP);
}

TEST_F(SessionTest, GateSettlesOnTargetListAndLimits)
{
  begin(Intent::FULL, {}, {"b"});
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2)}));
  EXPECT_TRUE(session_.gate_allows_next());
  ASSERT_EQ(session_.select(reachable({"b"})), "b");
  session_.on_harvest(harvest_ok("b"));
  session_.record_success();
  EXPECT_FALSE(session_.gate_allows_next());
  EXPECT_EQ(session_.termination_reason(), "target_list_done");
}

TEST_F(SessionTest, EmptyRoundsSettleAtLimitAndResetOnSelect)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  EXPECT_FALSE(session_.on_empty_round());
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.on_harvest(harvest_ok("a"));
  session_.record_success();
  EXPECT_FALSE(session_.on_empty_round());
  EXPECT_TRUE(session_.on_empty_round());
  EXPECT_TRUE(session_.settled());
  EXPECT_EQ(session_.settle_reason(), "no_targets");
}

TEST_F(SessionTest, CancelRecordsOpenTarget)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2)}));
  ASSERT_EQ(session_.select(reachable({"a", "b"})), "a");
  EXPECT_EQ(session_.finish(core::BatchEnd::CANCELED), "canceled");
  EXPECT_EQ(session_.results().back().outcome, core::outcome::CANCELED);
  EXPECT_EQ(session_.results().back().failure_code, fc::CANCELED);
  EXPECT_EQ(session_.phase(), core::Phase::ABORTED);
  EXPECT_FALSE(session_.active());
  ASSERT_EQ(session_.rework().size(), 2U);
  EXPECT_EQ(session_.rework()[0].kind, "canceled");
  EXPECT_EQ(session_.rework()[1].target_id, "b");
  EXPECT_EQ(session_.rework()[1].kind, "not_attempted");
}

TEST_F(SessionTest, FailedTreeKeepsAbortReason)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.abort("safety:e_stopped");
  EXPECT_EQ(session_.finish(core::BatchEnd::FAILED), "safety:e_stopped");
  EXPECT_EQ(session_.results().back().outcome, core::outcome::FAILED);
  EXPECT_EQ(session_.rework().back().kind, "not_finished");
}

TEST_F(SessionTest, FinishSucceededReworkKinds)
{
  core::BatchLimits l;
  l.target_harvest_ratio = 0.5;
  begin(Intent::FULL, l);
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2)}));
  ASSERT_EQ(session_.select(reachable({"a", "b"})), "a");
  session_.on_harvest(harvest_ok("a"));
  session_.record_success();
  EXPECT_FALSE(session_.gate_allows_next());
  EXPECT_EQ(session_.finish(core::BatchEnd::SUCCEEDED), "ratio_reached");
  EXPECT_EQ(session_.phase(), core::Phase::COMPLETED);
  ASSERT_EQ(session_.rework().size(), 1U);
  EXPECT_EQ(session_.rework()[0].kind, "ratio_satisfied");
}

TEST_F(SessionTest, TargetListIdNeverSeenIsNotObserved)
{
  begin(Intent::FULL, {}, {"ghost"});
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  session_.finish(core::BatchEnd::SUCCEEDED);
  ASSERT_EQ(session_.rework().size(), 1U);
  EXPECT_EQ(session_.rework()[0].target_id, "ghost");
  EXPECT_EQ(session_.rework()[0].kind, "not_observed");
}

TEST_F(SessionTest, SnapshotAndRevision)
{
  const uint64_t r0 = session_.revision();
  begin(Intent::FULL);
  EXPECT_GT(session_.revision(), r0);
  session_.set_safety_blockers({"e_stopped"});
  session_.require_recovery("x");
  const auto s = session_.snapshot();
  EXPECT_EQ(s.request_id, "req");
  EXPECT_EQ(s.phase, core::Phase::SURVEYING);
  EXPECT_EQ(s.blockers, (std::vector<std::string>{"e_stopped", "recovery_required"}));
  EXPECT_TRUE(s.recovery_required);
  const uint64_t r1 = session_.revision();
  session_.set_phase(core::Phase::SURVEYING);
  EXPECT_EQ(session_.revision(), r1);
}

TEST_F(SessionTest, WritesLedgerAndReworkFiles)
{
  const fs::path root = fs::temp_directory_path() /
    ("peach2_task_session_" + std::to_string(::getpid()));
  fs::remove_all(root);
  std::string err;
  ledger_ = core::Ledger::create(root, "req", &err);
  ASSERT_TRUE(ledger_) << err;
  begin(Intent::PREGRASP_ONLY);
  ASSERT_TRUE(fs::exists(root / "req" / "ledger.json"));
  session_.on_scene_begun(4);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.on_harvest(harvest_ok("a"));
  session_.record_success();
  session_.finish(core::BatchEnd::SUCCEEDED);
  EXPECT_EQ(session_.ledger_write_failures(), 0U);

  std::ifstream in(root / "req" / "ledger.json");
  std::stringstream ss;
  ss << in.rdbuf();
  const auto j = nlohmann::json::parse(ss.str());
  EXPECT_EQ(j["termination_reason"], "completed");
  EXPECT_EQ(j["intent"], "PREGRASP_ONLY");
  EXPECT_EQ(j["targets"][0]["outcome"], "SUCCEEDED");
  EXPECT_EQ(j["targets"][0]["reached"], "PREGRASP");
  EXPECT_EQ(j["discovered_ids"][0], "a");
  EXPECT_EQ(j["scene_epoch"], 4);
  EXPECT_EQ(j["reachability"]["a"]["reachable"], true);
  EXPECT_EQ(j["reachability"]["a"]["failure_name"], "NONE");
  EXPECT_EQ(j["targets"][0]["plan_only"], false);
  EXPECT_TRUE(fs::exists(root / "req" / "rework.json"));
  fs::remove_all(root);
}

TEST_F(SessionTest, SceneEpochFromBeginScene)
{
  begin(Intent::FULL);
  EXPECT_FALSE(session_.scene_begun());
  session_.on_scene_begun(7);
  EXPECT_TRUE(session_.scene_begun());
  EXPECT_EQ(session_.scene_epoch(), 7U);
  begin(Intent::FULL);
  EXPECT_FALSE(session_.scene_begun());
}

TEST_F(SessionTest, OnlyLockedIdsAreDiscoveredAndSelected)
{
  begin(Intent::FULL);
  auto s = scene({bag("a", 1.0), bag("b", 1.2)});
  s.locked_target_ids = {"b"};
  session_.on_locked_snapshot(s);
  EXPECT_EQ(session_.counts().discovered, 1U);
  EXPECT_EQ(session_.select(reachable({"a", "b"})), "b");
  EXPECT_EQ(session_.last_filtered().at("a"), "not_in_locked_set");
}

TEST_F(SessionTest, ReachabilityReasonsReachLedgerAndRework)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2)}));
  core::ReachMap reach = reachable({"b"});
  reach["a"] = core::Reach{false, fc::PLAN_COLLISION, "hits peach_bag_b"};
  ASSERT_EQ(session_.select(reach), "b");
  const auto & rec = session_.reachability().at("a");
  EXPECT_FALSE(rec.reachable);
  EXPECT_EQ(rec.failure_name, "PLAN_COLLISION");
  EXPECT_EQ(rec.reason, "hits peach_bag_b");
  EXPECT_TRUE(session_.reachability().at("b").reachable);
  session_.on_harvest(harvest_ok("b"));
  session_.record_success();
  session_.finish(core::BatchEnd::SUCCEEDED);
  ASSERT_EQ(session_.rework().size(), 1U);
  EXPECT_EQ(session_.rework()[0].kind, "unreachable");
  EXPECT_EQ(session_.rework()[0].reason, "check_reachability:PLAN_COLLISION:hits peach_bag_b");
}

TEST_F(SessionTest, PlanOnlySuccessIsNotAttempted)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.begin_attempt();
  ASSERT_TRUE(session_.on_harvest(plan_only("a")));
  session_.record_success();
  EXPECT_EQ(session_.counts().attempted, 0U);
  EXPECT_EQ(session_.counts().succeeded, 0U);
  EXPECT_EQ(session_.counts().plan_only, 1U);
  EXPECT_FALSE(session_.recovery_required());
  EXPECT_TRUE(session_.results().back().plan_only);
  EXPECT_TRUE(session_.records().back().plan_only);
  ASSERT_EQ(session_.rework().size(), 1U);
  EXPECT_EQ(session_.rework()[0].kind, "plan_only");
  EXPECT_EQ(session_.rework()[0].reason, "planned_not_executed");
  EXPECT_FALSE(session_.rework()[0].attempted);
}

TEST_F(SessionTest, PlanOnlyPregraspSkipsCheckpoint)
{
  begin(Intent::PREGRASP_ONLY);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.on_harvest(plan_only("a"));
  session_.record_success();
  EXPECT_FALSE(session_.recovery_required());
}

TEST_F(SessionTest, PlanOnlyFailureIsNotSkippedOrFailed)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  auto h = plan_only("a");
  h.outcome = core::outcome::FAILED;
  h.failure_code = fc::EXEC_FAILED;
  h.reason = "planned only";
  EXPECT_FALSE(session_.on_harvest(h));
  EXPECT_TRUE(session_.record_skip());
  EXPECT_EQ(session_.counts().skipped, 0U);
  EXPECT_EQ(session_.counts().failed, 0U);
  EXPECT_EQ(session_.counts().plan_only, 1U);
  EXPECT_FALSE(session_.recovery_required());
  ASSERT_EQ(session_.rework().size(), 1U);
  EXPECT_EQ(session_.rework()[0].kind, "plan_only");
  EXPECT_EQ(session_.rework()[0].failure_code, fc::EXEC_FAILED);
}

TEST_F(SessionTest, PlanOnlyCountsTowardMaxTargets)
{
  core::BatchLimits l;
  l.max_targets = 1;
  begin(Intent::FULL, l);
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2)}));
  ASSERT_EQ(session_.select(reachable({"a", "b"})), "a");
  session_.on_harvest(plan_only("a"));
  session_.record_success();
  EXPECT_FALSE(session_.gate_allows_next());
  EXPECT_EQ(session_.termination_reason(), "max_targets");
}

TEST_F(SessionTest, PeerRecoveryNeedsTaskAckAndLatchRelease)
{
  begin(Intent::FULL);
  session_.on_peer_recovery(true);
  EXPECT_TRUE(session_.recovery_required());
  EXPECT_EQ(session_.recovery_reason(), "manipulation_recovery_required");
  const auto blockers = session_.snapshot().blockers;
  EXPECT_NE(
    std::find(blockers.begin(), blockers.end(), "manipulation_recovery_required"),
    blockers.end());
  EXPECT_FALSE(session_.consume_ack());
  EXPECT_TRUE(session_.grant_ack());
  EXPECT_FALSE(session_.consume_ack());
  session_.on_peer_recovery(false);
  EXPECT_TRUE(session_.recovery_required());
  EXPECT_TRUE(session_.consume_ack());
  EXPECT_FALSE(session_.recovery_required());
}

TEST_F(SessionTest, PeerLatchDroppingAloneDoesNotRelease)
{
  begin(Intent::FULL);
  session_.on_peer_recovery(true);
  session_.on_peer_recovery(false);
  EXPECT_FALSE(session_.consume_ack());
  EXPECT_TRUE(session_.grant_ack());
  EXPECT_TRUE(session_.consume_ack());
}

TEST_F(SessionTest, PeerLatchSurvivesBeginWithoutArmingIdle)
{
  session_.on_peer_recovery(true);
  EXPECT_FALSE(session_.recovery_required());
  begin(Intent::FULL);
  EXPECT_TRUE(session_.peer_recovery_required());
}

TEST_F(SessionTest, NeckRemeasureOncePerNormalAttempt)
{
  core::BatchLimits l;
  l.per_target_timeout_s = 10.0;
  begin(Intent::FULL, l);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.begin_attempt();
  EXPECT_FALSE(session_.neck_remeasure_pending());
  session_.set_failure(core::Failure{fc::NECK_REMEASURE_PENDING, "", false, {}, {}});
  t_ += 20.0;
  EXPECT_EQ(session_.retry_verdict(1, 1), core::RetryVerdict::RETRY_NOW);
  session_.begin_attempt();
  EXPECT_TRUE(session_.neck_remeasure_pending());
  session_.set_failure(core::Failure{fc::NECK_REMEASURE_PENDING, "", false, {}, {}});
  EXPECT_EQ(session_.retry_verdict(1, 1), core::RetryVerdict::GIVE_UP);
  EXPECT_TRUE(session_.record_skip());
  EXPECT_EQ(session_.rework().back().kind, "neck_remeasure");
  EXPECT_EQ(session_.results().back().outcome, core::outcome::SKIPPED);
}

TEST_F(SessionTest, NeckRemeasureOnlyForFullIntent)
{
  begin(Intent::PREGRASP_ONLY);
  session_.on_locked_snapshot(scene({bag("a", 1.0)}));
  ASSERT_EQ(session_.select(reachable({"a"})), "a");
  session_.begin_attempt();
  session_.set_failure(core::Failure{fc::NECK_REMEASURE_PENDING, "", false, {}, {}});
  EXPECT_EQ(session_.retry_verdict(1, 3), core::RetryVerdict::GIVE_UP);
}

TEST_F(SessionTest, TimeoutAndDependencyCodes)
{
  begin(Intent::FULL);
  session_.on_locked_snapshot(scene({bag("a", 1.0), bag("b", 1.2)}));
  ASSERT_EQ(session_.select(reachable({"a", "b"})), "a");
  session_.set_failure(core::Failure{fc::TARGET_TIMEOUT, "per_target_timeout", false, {}, {}});
  EXPECT_TRUE(session_.record_skip());
  EXPECT_EQ(session_.results().back().outcome, core::outcome::SKIPPED);
  EXPECT_EQ(session_.rework().back().kind, "timeout");
  ASSERT_EQ(session_.select(reachable({"b"})), "b");
  session_.set_failure(core::Failure{fc::DEPENDENCY_UNAVAILABLE, "observe:server", false, {}, {}});
  EXPECT_FALSE(session_.record_skip());
  EXPECT_EQ(session_.results().back().outcome, core::outcome::FAILED);
  EXPECT_EQ(session_.rework().back().kind, "infrastructure");
  EXPECT_EQ(session_.abort_reason(), "stop_batch:DEPENDENCY_UNAVAILABLE:observe:server");
  EXPECT_FALSE(session_.recovery_required());
}
