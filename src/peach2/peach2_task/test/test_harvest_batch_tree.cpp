// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
// trees/harvest_batch.xml driven by pure C++ mock leaves (same ports as the ROS leaves) and
// the real logic nodes + BatchSession, on a fake clock. No ROS.
#include <gtest/gtest.h>

#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "peach2_task/bt/logic_nodes.hpp"
#include "peach2_task/bt/ports.hpp"
#include "peach2_task/core/batch_session.hpp"
#include "peach2_task/core/safety_gate.hpp"

namespace core = peach2_task::core;
namespace fc = peach2_task::core::fc;
namespace ports = peach2_task::bt::ports;
using BT::NodeStatus;
using core::Intent;

namespace
{

constexpr double kTickS = 0.02;

enum class Kind { MOVE, SNAPSHOT, BEGIN_SCENE, WAIT_LOCK, SELECT, OBSERVE, DECISION, HARVEST };

struct World
{
  double t = 0.0;
  std::shared_ptr<core::BatchSession> session;

  std::deque<core::ObservationSet> scenes;  ///< consumed per survey; the last one sticks
  std::map<std::string, core::Reach> reach_override;
  std::map<std::string, std::deque<core::ObserveOutcome>> observe;
  std::map<std::string, std::deque<core::DecisionOutcome>> decision;
  std::map<std::string, std::deque<core::HarvestOutcome>> harvest;
  std::set<std::string> observe_hangs;
  int harvest_running_ticks = 2;
  core::Enables enables{true, true, true};  ///< operator enables seen by the real safety gate
  bool begin_scene_accepts = true;
  bool plan_only = false;  ///< default HarvestTarget results are plan-only
  std::function<void(const std::string &)> on_observe;  ///< runs at every ObserveTarget start

  int moves = 0;
  int snapshots = 0;
  int begin_scenes = 0;
  std::vector<bool> snapshot_clear;
  std::vector<bool> observe_neck;
  int surveys = 0;
  int observe_calls = 0;
  int observe_halts = 0;
  int harvest_halts = 0;
  std::vector<std::string> move_targets;
  std::vector<std::string> harvested;  ///< HarvestTarget goals in order
  std::vector<int> harvest_intents;
  std::vector<double> harvest_start_t;
  std::vector<uint64_t> decision_min_revisions;
  std::vector<std::string> decision_levels;
};

class MockLeaf : public BT::StatefulActionNode
{
public:
  MockLeaf(const std::string & name, const BT::NodeConfig & config, World * world, Kind kind)
  : BT::StatefulActionNode(name, config), w_(world), kind_(kind) {}

  NodeStatus onStart() override
  {
    auto & s = *w_->session;
    switch (kind_) {
      case Kind::MOVE:
        ++w_->moves;
        w_->move_targets.push_back(getInput<std::string>("target").value());
        s.set_phase(core::Phase::SURVEYING);
        return NodeStatus::SUCCESS;
      case Kind::SNAPSHOT:
        ++w_->snapshots;
        w_->snapshot_clear.push_back(getInput<bool>("clear_previous").value());
        return NodeStatus::SUCCESS;
      case Kind::BEGIN_SCENE:
        ++w_->begin_scenes;
        if (!w_->begin_scene_accepts) {
          s.abort("survey_failed:begin_scene:rejected:busy");
          return NodeStatus::FAILURE;
        }
        s.on_scene_begun(static_cast<uint32_t>(w_->begin_scenes));
        return NodeStatus::SUCCESS;
      case Kind::WAIT_LOCK: {
          ++w_->surveys;
          core::ObservationSet set = w_->scenes.front();
          if (w_->scenes.size() > 1) {
            w_->scenes.pop_front();
          }
          s.on_locked_snapshot(set);
          return NodeStatus::SUCCESS;
        }
      case Kind::SELECT: {
          s.set_phase(core::Phase::SELECTING);
          core::ReachMap reach;
          for (const auto & id : s.reach_query_ids()) {
            const auto it = w_->reach_override.find(id);
            reach[id] = it != w_->reach_override.end() ? it->second : core::Reach{true, 0, {}};
          }
          const std::string tid = s.select(reach);
          if (tid.empty()) {
            return NodeStatus::FAILURE;
          }
          setOutput("target_id", tid);
          return NodeStatus::SUCCESS;
        }
      case Kind::OBSERVE: {
          s.set_phase(core::Phase::OBSERVING);
          ++w_->observe_calls;
          tid_ = getInput<std::string>("target_id").value();
          EXPECT_EQ(tid_, s.current_target());
          const bool neck = getInput<bool>("neck_remeasure").value();
          w_->observe_neck.push_back(neck);
          EXPECT_EQ(getInput<unsigned>("max_views").value(), neck ? 1U : 3U);
          EXPECT_EQ(neck, s.neck_remeasure_pending());
          if (w_->on_observe) {
            w_->on_observe(tid_);
          }
          if (w_->observe_hangs.count(tid_) != 0) {
            return NodeStatus::RUNNING;
          }
          core::ObserveOutcome o;
          o.converged = true;
          o.model_revision = 7;
          auto & q = w_->observe[tid_];
          if (!q.empty()) {
            o = q.front();
            q.pop_front();
          }
          setOutput<uint64_t>("model_revision", o.model_revision);
          return s.on_observe(o) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        }
      case Kind::DECISION: {
          tid_ = getInput<std::string>("target_id").value();
          EXPECT_EQ(tid_, s.current_target());
          const std::string level_name = getInput<std::string>("level").value();
          w_->decision_levels.push_back(level_name);
          w_->decision_min_revisions.push_back(getInput<uint64_t>("min_model_revision").value());
          core::DecisionLevel level = core::DecisionLevel::APPROACH;
          EXPECT_TRUE(core::decision_level_from_name(level_name, &level));
          core::DecisionOutcome d;
          d.found = true;
          d.approach_allowed = d.sleeve_allowed = d.cut_allowed = true;
          d.model_revision = 7;
          auto & q = w_->decision[tid_];
          if (!q.empty()) {
            d = q.front();
            q.pop_front();
          }
          return s.on_decision(d, level) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        }
      case Kind::HARVEST:
        s.set_phase(core::Phase::HARVESTING);
        tid_ = getInput<std::string>("target_id").value();
        EXPECT_EQ(tid_, s.current_target());
        EXPECT_EQ(getInput<std::string>("tool_id").value(), s.request().tool_id);
        w_->harvested.push_back(tid_);
        w_->harvest_intents.push_back(getInput<int>("intent").value());
        w_->harvest_start_t.push_back(w_->t);
        remaining_ = w_->harvest_running_ticks;
        return onRunning();
    }
    return NodeStatus::FAILURE;
  }

  NodeStatus onRunning() override
  {
    if (kind_ == Kind::OBSERVE) {
      return NodeStatus::RUNNING;  // only hanging observes stay RUNNING
    }
    if (remaining_-- > 0) {
      return NodeStatus::RUNNING;
    }
    core::HarvestOutcome h;
    h.target_id = tid_;
    h.outcome = core::outcome::SUCCEEDED;
    h.reached = w_->session->request().intent == Intent::FULL ?
      core::reached::RELEASED : core::reached::PREGRASP;
    if (w_->plan_only) {
      h.reached = core::reached::NONE;
      h.plan_only = true;
    }
    auto & q = w_->harvest[tid_];
    if (!q.empty()) {
      h = q.front();
      h.target_id = tid_;
      q.pop_front();
    }
    return w_->session->on_harvest(h) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    if (kind_ == Kind::HARVEST) {
      ++w_->harvest_halts;
    } else if (kind_ == Kind::OBSERVE) {
      ++w_->observe_halts;
    }
  }

private:
  World * w_;
  Kind kind_;
  std::string tid_;
  int remaining_ = 0;
};

class MockSafety : public BT::ConditionNode
{
public:
  MockSafety(const std::string & name, const BT::NodeConfig & config, World * world)
  : BT::ConditionNode(name, config), w_(world) {}

  /// Same verdict path as the ROS CheckSafety leaf, minus robot_status (mock hardware).
  NodeStatus tick() override
  {
    core::SafetyConfig cfg;
    cfg.require_robot_status = false;
    core::SafetyInputs in;
    in.enables = w_->enables;
    in.intent = w_->session->request().intent;
    const auto verdict = core::evaluate_safety(cfg, in);
    w_->session->set_safety_blockers(verdict.blockers);
    if (verdict.ok) {
      return NodeStatus::SUCCESS;
    }
    const std::string reason = "safety:" + verdict.reason();
    w_->session->set_failure(core::Failure{verdict.failure_code, reason, false, {}, {}});
    w_->session->abort(reason);
    return NodeStatus::FAILURE;
  }

private:
  World * w_;
};

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

core::HarvestOutcome harvest_fail(uint32_t code, bool recovery = false)
{
  core::HarvestOutcome h;
  h.outcome = core::outcome::FAILED;
  h.failure_code = code;
  h.recovery_required = recovery;
  h.reason = "mock";
  return h;
}

class TreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    world_.session = std::make_shared<core::BatchSession>(
      [this] {return world_.t;},
      [] {return std::chrono::system_clock::time_point(std::chrono::seconds(1790752620));});
    peach2_task::bt::register_logic_nodes(factory_, world_.session);
    World * w = &world_;
    factory_.registerNodeType<MockSafety>("CheckSafety", ports::check_safety(), w);
    factory_.registerNodeType<MockLeaf>("MoveToNamed", ports::move_to_named(), w, Kind::MOVE);
    factory_.registerNodeType<MockLeaf>(
      "BuildSceneSnapshot", ports::build_scene_snapshot(), w, Kind::SNAPSHOT);
    factory_.registerNodeType<MockLeaf>("BeginScene", ports::begin_scene(), w, Kind::BEGIN_SCENE);
    factory_.registerNodeType<MockLeaf>(
      "WaitTargetSetLocked", ports::wait_target_set_locked(), w, Kind::WAIT_LOCK);
    factory_.registerNodeType<MockLeaf>("SelectTarget", ports::select_target(), w, Kind::SELECT);
    factory_.registerNodeType<MockLeaf>(
      "ObserveTarget", ports::observe_target(), w, Kind::OBSERVE);
    factory_.registerNodeType<MockLeaf>(
      "CheckDecision", ports::check_decision(), w, Kind::DECISION);
    factory_.registerNodeType<MockLeaf>(
      "HarvestTarget", ports::harvest_target(), w, Kind::HARVEST);
    factory_.registerBehaviorTreeFromFile(PEACH2_TASK_TREE_FILE);
    world_.scenes.push_back(scene({bag("t_high", 1.45), bag("t_low", 1.05), bag("t_mid", 1.25)}));
  }

  void start(Intent intent, core::BatchLimits limits = {}, std::vector<std::string> ids = {})
  {
    core::BatchRequest r;
    r.request_id = "tree_test";
    r.intent = intent;
    r.tool_id = "adaptive_shear_v1";
    r.target_ids = std::move(ids);
    r.limits = limits;
    world_.session->begin(r, config_, nullptr);
    auto bb = BT::Blackboard::create();
    bb->set<int>("intent", static_cast<int>(intent));
    bb->set<std::string>("tool_id", r.tool_id);
    bb->set<std::string>("photo_pose", "global_photo_pose");
    bb->set<unsigned>("max_views", 3U);
    bb->set<unsigned>("retry_attempts", 2U);
    tree_ = std::make_unique<BT::Tree>(factory_.createTree("HarvestBatch", bb));
  }

  /// Ticks until the tree leaves RUNNING, `stop` returns true, or max_ticks.
  NodeStatus run(int max_ticks = 5000, const std::function<bool()> & stop = {})
  {
    for (int i = 0; i < max_ticks; ++i) {
      const NodeStatus st = tree_->tickOnce();
      world_.t += kTickS;
      if (st != NodeStatus::RUNNING) {
        return st;
      }
      if (stop && stop()) {
        return NodeStatus::RUNNING;
      }
    }
    return NodeStatus::RUNNING;
  }

  std::string finish(NodeStatus st)
  {
    const core::BatchEnd end = st == NodeStatus::SUCCESS ? core::BatchEnd::SUCCEEDED :
      core::BatchEnd::FAILED;
    return world_.session->finish(end);
  }

  core::SessionConfig config_;
  World world_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
};

}  // namespace

TEST_F(TreeTest, FullBatchAllSucceedLowerFirst)
{
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_low", "t_mid", "t_high"}));
  EXPECT_EQ(world_.harvest_intents, (std::vector<int>{2, 2, 2}));
  EXPECT_EQ(world_.decision_levels, (std::vector<std::string>(3, "approach")));
  EXPECT_EQ(world_.decision_min_revisions, (std::vector<uint64_t>(3, 7U)));
  // Initial survey, then one re-survey after the first empty round; second empty round settles.
  EXPECT_EQ(world_.surveys, 2);
  EXPECT_EQ(world_.moves, 2);
  EXPECT_EQ(world_.move_targets.front(), "global_photo_pose");
  // BeginScene only on the first survey; the re-survey merges into the same scene epoch.
  EXPECT_EQ(world_.begin_scenes, 1);
  EXPECT_EQ(world_.snapshot_clear, (std::vector<bool>{true, false}));
  EXPECT_EQ(world_.session->scene_epoch(), 1U);
  EXPECT_EQ(world_.observe_neck, (std::vector<bool>(3, false)));
  EXPECT_EQ(finish(st), "no_targets");
  const auto & c = world_.session->counts();
  EXPECT_EQ(c.discovered, 3U);
  EXPECT_EQ(c.attempted, 3U);
  EXPECT_EQ(c.succeeded, 3U);
  EXPECT_EQ(world_.session->phase(), core::Phase::COMPLETED);
}

TEST_F(TreeTest, MaxTargetsSettles)
{
  core::BatchLimits l;
  l.max_targets = 2;
  start(Intent::FULL, l);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested.size(), 2U);
  EXPECT_EQ(world_.surveys, 1);
  EXPECT_EQ(finish(st), "max_targets");
  ASSERT_EQ(world_.session->rework().size(), 1U);
  EXPECT_EQ(world_.session->rework()[0].target_id, "t_high");
  EXPECT_EQ(world_.session->rework()[0].kind, "max_targets");
}

TEST_F(TreeTest, RatioSettles)
{
  core::BatchLimits l;
  l.target_harvest_ratio = 0.5;
  start(Intent::FULL, l);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested.size(), 2U);
  EXPECT_EQ(finish(st), "ratio_reached");
}

TEST_F(TreeTest, TargetListOrderAndCompletion)
{
  start(Intent::FULL, {}, {"t_high", "t_low"});
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_high", "t_low"}));
  EXPECT_EQ(finish(st), "target_list_done");
}

TEST_F(TreeTest, SkipTargetAndContinue)
{
  core::DecisionOutcome d;
  d.found = true;
  d.approach_allowed = false;
  d.failure_code = fc::BUDGET_RADIAL_NEGATIVE;
  d.reason = "radial_margin_negative";
  world_.decision["t_low"].push_back(d);
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_mid", "t_high"}));
  EXPECT_EQ(world_.observe_calls, 3);  // SKIP_TOOL is not retried
  finish(st);
  const auto & c = world_.session->counts();
  EXPECT_EQ(c.skipped, 1U);
  EXPECT_EQ(c.succeeded, 2U);
  ASSERT_FALSE(world_.session->rework().empty());
  EXPECT_EQ(world_.session->rework()[0].target_id, "t_low");
  EXPECT_EQ(world_.session->rework()[0].kind, "tool");
  EXPECT_EQ(world_.session->results()[0].outcome, core::outcome::SKIPPED);
}

TEST_F(TreeTest, RetryViewThenSucceed)
{
  core::ObserveOutcome bad;
  bad.failure_code = fc::PERCEPTION_LOW_QUALITY;
  world_.observe["t_low"].push_back(bad);
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested.front(), "t_low");
  EXPECT_EQ(world_.observe_calls, 4);
  finish(st);
  EXPECT_EQ(world_.session->counts().succeeded, 3U);
}

TEST_F(TreeTest, RetryExhaustedSkips)
{
  core::ObserveOutcome bad;
  bad.failure_code = fc::MODEL_STALE;
  world_.observe["t_low"] = {bad, bad, bad};
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_mid", "t_high"}));
  finish(st);
  EXPECT_EQ(world_.session->results()[0].failure_code, fc::MODEL_STALE);
  EXPECT_EQ(world_.session->counts().skipped, 1U);
}

TEST_F(TreeTest, WaitPolicyPausesBeforeRetry)
{
  config_.wait_retry_s = 1.0;
  world_.harvest["t_low"].push_back(harvest_fail(fc::SWING_TOO_LARGE));
  world_.harvest["t_low"].back().outcome = core::outcome::SKIPPED;
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  ASSERT_GE(world_.harvested.size(), 2U);
  EXPECT_EQ(world_.harvested[0], "t_low");
  EXPECT_EQ(world_.harvested[1], "t_low");
  EXPECT_GE(world_.harvest_start_t[1] - world_.harvest_start_t[0], 1.0);
  finish(st);
  EXPECT_EQ(world_.session->counts().succeeded, 3U);
}

TEST_F(TreeTest, PregraspCheckpointWaitsForAck)
{
  start(Intent::PREGRASP_ONLY);
  NodeStatus st = run(5000, [this] {
        return world_.session->phase() == core::Phase::WAITING_ACK;
      });
  ASSERT_EQ(st, NodeStatus::RUNNING);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_low"}));
  EXPECT_EQ(world_.harvest_intents.front(), 1);
  // No ACK: the tree holds, nothing else is sent.
  st = run(200);
  EXPECT_EQ(st, NodeStatus::RUNNING);
  EXPECT_EQ(world_.harvested.size(), 1U);
  EXPECT_TRUE(world_.session->snapshot().recovery_required);

  EXPECT_TRUE(world_.session->grant_ack());
  const auto second_checkpoint = [this] {
      const bool waiting = world_.session->phase() == core::Phase::WAITING_ACK;
      return waiting && world_.harvested.size() == 2U;
    };
  st = run(5000, second_checkpoint);
  ASSERT_EQ(st, NodeStatus::RUNNING);
  EXPECT_EQ(world_.harvested.back(), "t_mid");
}

TEST_F(TreeTest, RecoverFailureBlocksUntilAck)
{
  world_.harvest["t_low"].push_back(harvest_fail(fc::EXEC_FAILED, true));
  start(Intent::FULL);
  NodeStatus st = run(5000, [this] {
        return world_.session->phase() == core::Phase::WAITING_ACK;
      });
  ASSERT_EQ(st, NodeStatus::RUNNING);
  EXPECT_EQ(world_.harvested.size(), 1U);  // RECOVER is never retried
  st = run(200);
  EXPECT_EQ(world_.harvested.size(), 1U);
  EXPECT_EQ(world_.session->counts().failed, 1U);

  world_.session->grant_ack();
  st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_low", "t_mid", "t_high"}));
  finish(st);
  EXPECT_EQ(world_.session->rework()[0].kind, "recovery");
}

TEST_F(TreeTest, CancelDuringHarvestHaltsLeaf)
{
  world_.harvest_running_ticks = 1000;
  start(Intent::FULL);
  NodeStatus st = run(5000, [this] {return !world_.harvested.empty();});
  ASSERT_EQ(st, NodeStatus::RUNNING);
  tree_->haltTree();
  EXPECT_EQ(world_.harvest_halts, 1);
  EXPECT_EQ(world_.session->finish(core::BatchEnd::CANCELED), "canceled");
  ASSERT_EQ(world_.session->results().size(), 1U);
  EXPECT_EQ(world_.session->results()[0].outcome, core::outcome::CANCELED);
  EXPECT_EQ(world_.session->phase(), core::Phase::ABORTED);
}

TEST_F(TreeTest, SafetyLossHaltsInFlightHarvest)
{
  world_.harvest_running_ticks = 1000;
  start(Intent::FULL);
  NodeStatus st = run(5000, [this] {return !world_.harvested.empty();});
  ASSERT_EQ(st, NodeStatus::RUNNING);
  world_.enables = core::Enables{};
  st = run(5);
  ASSERT_EQ(st, NodeStatus::FAILURE);
  EXPECT_EQ(world_.harvest_halts, 1);
  EXPECT_EQ(finish(st), "safety:enable_missing:execution,enable_missing:grasp,enable_missing:tool");
  EXPECT_EQ(world_.session->results()[0].outcome, core::outcome::FAILED);
  EXPECT_EQ(world_.session->results()[0].failure_code, fc::SAFETY_GATE_CLOSED);
}

TEST_F(TreeTest, PregraspLosingExecutionMidObserveHalts)
{
  world_.enables = core::Enables{true, false, false};
  world_.observe_hangs.insert("t_low");
  start(Intent::PREGRASP_ONLY);
  NodeStatus st = run(5000, [this] {return world_.observe_calls > 0;});
  ASSERT_EQ(st, NodeStatus::RUNNING);
  world_.enables.execution = false;
  st = run(5);
  ASSERT_EQ(st, NodeStatus::FAILURE);
  EXPECT_EQ(world_.observe_halts, 1);
  EXPECT_TRUE(world_.harvested.empty());
  EXPECT_EQ(finish(st), "safety:enable_missing:execution");
  EXPECT_EQ(world_.session->phase(), core::Phase::ABORTED);
}

TEST_F(TreeTest, FullLosingToolMidHarvestHalts)
{
  world_.harvest_running_ticks = 1000;
  start(Intent::FULL);
  NodeStatus st = run(5000, [this] {return !world_.harvested.empty();});
  ASSERT_EQ(st, NodeStatus::RUNNING);
  world_.enables.tool = false;
  st = run(5);
  ASSERT_EQ(st, NodeStatus::FAILURE);
  EXPECT_EQ(world_.harvest_halts, 1);
  EXPECT_EQ(finish(st), "safety:enable_missing:tool");
}

TEST_F(TreeTest, PregraspRunsWithExecutionOnly)
{
  config_.ack_each_pregrasp = false;
  world_.enables = core::Enables{true, false, false};
  start(Intent::PREGRASP_ONLY);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested.size(), 3U);
}

TEST_F(TreeTest, EmptySurveyLimitSettles)
{
  world_.scenes = {scene({})};
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.surveys, 2);
  EXPECT_EQ(world_.moves, 2);
  EXPECT_EQ(world_.snapshots, 2);
  EXPECT_TRUE(world_.harvested.empty());
  EXPECT_EQ(finish(st), "no_targets");
}

TEST_F(TreeTest, ResurveyFindsNewTargets)
{
  world_.scenes = {scene({}), scene({bag("late", 1.1)})};
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"late"}));
  EXPECT_EQ(world_.surveys, 3);
  EXPECT_EQ(finish(st), "no_targets");
}

TEST_F(TreeTest, SurveyOnlyNeverSelects)
{
  start(Intent::SURVEY_ONLY);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.surveys, 1);
  EXPECT_EQ(world_.observe_calls, 0);
  EXPECT_TRUE(world_.harvested.empty());
  EXPECT_EQ(finish(st), "survey_only");
  EXPECT_EQ(world_.session->counts().discovered, 3U);
}

TEST_F(TreeTest, StopBatchFailureAborts)
{
  world_.harvest["t_low"].push_back(harvest_fail(fc::SAFETY_GATE_CLOSED));
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::FAILURE);
  EXPECT_EQ(world_.harvested.size(), 1U);
  EXPECT_EQ(finish(st), "stop_batch:SAFETY_GATE_CLOSED:mock");
  EXPECT_EQ(world_.session->phase(), core::Phase::ABORTED);
}

TEST_F(TreeTest, UnreachableTargetsAreNotAttempted)
{
  world_.reach_override["t_low"] = core::Reach{false, fc::PLAN_NO_IK, "no ik at pregrasp"};
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_mid", "t_high"}));
  finish(st);
  bool found = false;
  for (const auto & e : world_.session->rework()) {
    if (e.target_id == "t_low") {
      found = true;
      EXPECT_EQ(e.kind, "unreachable");
      EXPECT_EQ(e.reason, "check_reachability:PLAN_NO_IK:no ik at pregrasp");
      EXPECT_FALSE(e.attempted);
    }
  }
  EXPECT_TRUE(found);
  EXPECT_EQ(world_.session->reachability().at("t_low").reason, "no ik at pregrasp");
  EXPECT_EQ(world_.session->reachability().at("t_low").failure_name, "PLAN_NO_IK");
}

TEST_F(TreeTest, PerTargetTimeoutSkipsHangingObserve)
{
  core::BatchLimits l;
  l.per_target_timeout_s = 1.0;
  world_.observe_hangs.insert("t_low");
  start(Intent::FULL, l);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.observe_halts, 1);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_mid", "t_high"}));
  finish(st);
  EXPECT_EQ(world_.session->rework()[0].target_id, "t_low");
  EXPECT_EQ(world_.session->rework()[0].kind, "timeout");
  EXPECT_EQ(world_.session->results()[0].outcome, core::outcome::SKIPPED);
  EXPECT_EQ(world_.session->results()[0].failure_code, fc::TARGET_TIMEOUT);
}

TEST_F(TreeTest, BeginSceneRejectedAbortsSurvey)
{
  world_.begin_scene_accepts = false;
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::FAILURE);
  EXPECT_EQ(world_.surveys, 0);
  EXPECT_EQ(finish(st), "survey_failed:begin_scene:rejected:busy");
}

TEST_F(TreeTest, OnlyLockedIdsAreHarvested)
{
  world_.scenes.front().locked_target_ids = {"t_mid"};
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_mid"}));
  EXPECT_EQ(world_.session->counts().discovered, 1U);
  EXPECT_EQ(finish(st), "no_targets");
}

TEST_F(TreeTest, NeckRemeasureThenCut)
{
  world_.harvest["t_low"].push_back(harvest_fail(fc::NECK_REMEASURE_PENDING));
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(
    world_.harvested, (std::vector<std::string>{"t_low", "t_low", "t_mid", "t_high"}));
  EXPECT_EQ(world_.observe_neck, (std::vector<bool>{false, true, false, false}));
  EXPECT_EQ(
    world_.decision_levels,
    (std::vector<std::string>{"approach", "cut", "approach", "approach"}));
  finish(st);
  EXPECT_EQ(world_.session->counts().succeeded, 3U);
  EXPECT_EQ(world_.session->counts().attempted, 3U);
}

TEST_F(TreeTest, NeckRemeasureIgnoresSpentRetryBudget)
{
  core::ObserveOutcome stale;
  stale.failure_code = fc::MODEL_STALE;
  world_.observe["t_low"].push_back(stale);
  world_.harvest["t_low"].push_back(harvest_fail(fc::NECK_REMEASURE_PENDING));
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  // Attempt 1 retry-view, attempt 2 hits pregrasp and asks for the neck: still re-measured.
  EXPECT_EQ(world_.observe_neck, (std::vector<bool>{false, false, true, false, false}));
  EXPECT_EQ(world_.harvested.size(), 4U);
  finish(st);
  EXPECT_EQ(world_.session->counts().succeeded, 3U);
}

TEST_F(TreeTest, NeckRemeasureMismatchSkipsAsApproachOnly)
{
  world_.harvest["t_low"].push_back(harvest_fail(fc::NECK_REMEASURE_PENDING));
  core::DecisionOutcome approach;
  approach.found = true;
  approach.approach_allowed = approach.sleeve_allowed = approach.cut_allowed = true;
  approach.model_revision = 7;
  core::DecisionOutcome mismatch = approach;
  mismatch.cut_allowed = false;
  mismatch.failure_code = fc::NECK_REMEASURE_MISMATCH;
  mismatch.reason = "neck moved 12 mm";
  world_.decision["t_low"] = {approach, mismatch};
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_low", "t_mid", "t_high"}));
  finish(st);
  EXPECT_EQ(world_.session->results()[0].outcome, core::outcome::SKIPPED);
  EXPECT_EQ(world_.session->results()[0].failure_code, fc::NECK_REMEASURE_MISMATCH);
  EXPECT_EQ(world_.session->rework()[0].kind, "approach_only");
}

TEST_F(TreeTest, NeckRemeasurePendingTwiceSkips)
{
  world_.harvest["t_low"] = {
    harvest_fail(fc::NECK_REMEASURE_PENDING), harvest_fail(fc::NECK_REMEASURE_PENDING)};
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(
    world_.harvested, (std::vector<std::string>{"t_low", "t_low", "t_mid", "t_high"}));
  finish(st);
  EXPECT_EQ(world_.session->results()[0].failure_code, fc::NECK_REMEASURE_PENDING);
  EXPECT_EQ(world_.session->rework()[0].kind, "neck_remeasure");
  EXPECT_EQ(world_.session->counts().skipped, 1U);
}

TEST_F(TreeTest, NeckRemeasureNotForPregraspOnly)
{
  config_.ack_each_pregrasp = false;
  world_.harvest["t_low"].push_back(harvest_fail(fc::NECK_REMEASURE_PENDING));
  start(Intent::PREGRASP_ONLY);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_low", "t_mid", "t_high"}));
  EXPECT_EQ(world_.observe_neck, (std::vector<bool>(3, false)));
}

TEST_F(TreeTest, PlanOnlyIsNotCountedAsAttempted)
{
  world_.plan_only = true;
  core::BatchLimits l;
  l.max_targets = 2;
  start(Intent::PREGRASP_ONLY, l);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_low", "t_mid"}));
  EXPECT_EQ(finish(st), "max_targets");
  const auto & c = world_.session->counts();
  EXPECT_EQ(c.attempted, 0U);
  EXPECT_EQ(c.succeeded, 0U);
  EXPECT_EQ(c.plan_only, 2U);
  EXPECT_TRUE(world_.session->results()[0].plan_only);
  EXPECT_EQ(world_.session->rework()[0].kind, "plan_only");
  EXPECT_FALSE(world_.session->rework()[0].attempted);
}

TEST_F(TreeTest, PeerRecoveryBlocksHarvestUntilAckAndLatchClear)
{
  world_.on_observe = [this](const std::string & tid) {
      if (tid == "t_low") {
        world_.session->on_peer_recovery(true);
      }
    };
  start(Intent::FULL);
  NodeStatus st = run(5000, [this] {
        return world_.session->phase() == core::Phase::WAITING_ACK;
      });
  ASSERT_EQ(st, NodeStatus::RUNNING);
  EXPECT_TRUE(world_.harvested.empty());
  EXPECT_EQ(world_.session->results()[0].failure_code, fc::RECOVERY_REQUIRED);

  ASSERT_TRUE(world_.session->grant_ack());
  st = run(200);
  EXPECT_EQ(st, NodeStatus::RUNNING);
  EXPECT_TRUE(world_.harvested.empty());

  world_.on_observe = nullptr;
  world_.session->on_peer_recovery(false);
  st = run();
  ASSERT_EQ(st, NodeStatus::SUCCESS);
  EXPECT_EQ(world_.harvested, (std::vector<std::string>{"t_mid", "t_high"}));
}

TEST_F(TreeTest, DependencyUnavailableStopsBatch)
{
  core::ObserveOutcome down;
  down.failure_code = fc::DEPENDENCY_UNAVAILABLE;
  down.message = "observe:server_unavailable";
  world_.observe["t_low"].push_back(down);
  start(Intent::FULL);
  const NodeStatus st = run();
  ASSERT_EQ(st, NodeStatus::FAILURE);
  EXPECT_TRUE(world_.harvested.empty());
  EXPECT_EQ(world_.observe_calls, 1);
  EXPECT_EQ(finish(st), "stop_batch:DEPENDENCY_UNAVAILABLE:observe:server_unavailable");
  EXPECT_EQ(world_.session->results()[0].outcome, core::outcome::FAILED);
  EXPECT_EQ(world_.session->rework()[0].kind, "infrastructure");
}
