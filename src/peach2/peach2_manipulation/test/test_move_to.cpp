#include <gtest/gtest.h>

#include <functional>
#include <string>

#include "fakes.hpp"
#include "peach2_manipulation/move_to.hpp"

namespace failure = peach2_end_effector::failure;
namespace pm = peach2_manipulation;
using peach2_fakes::FakeMotion;

namespace
{

struct MoveRig
{
  FakeMotion motion;
  pm::CommandGate gate{[]() {
      pm::GateConfig c;
      c.require_robot_status = false;
      return c;
    }()};
  double now{10.0};
  bool execution{true};
  bool cancel{false};
  pm::MoveToConfig config;
  std::function<bool(std::string *)> scene;

  pm::MoveToResult run(const pm::MoveToRequest & req)
  {
    gate.set_active(true);
    pm::MoveToDeps d;
    d.motion = &motion;
    d.gate = [this](pm::GateStage s, bool nt) {
        tick();
        return gate.check(s, now, nt);
      };
    d.enables = [this]() {
        tick();
        return gate.enables(now);
      };
    d.cancel_requested = [this]() {return cancel;};
    d.scene = scene;
    return pm::run_move_to(req, config, d);
  }

  void tick()
  {
    pm::EnablesSample s;
    s.execution = execution;
    s.received_s = now;
    gate.on_enables(s);
    gate.set_cancel(cancel);
  }
};

}  // namespace

TEST(MoveTo, NamedTargetExecutes)
{
  MoveRig rig;
  pm::MoveToRequest req;
  req.named_target = "global_photo_pose";
  const auto r = rig.run(req);
  EXPECT_TRUE(r.success) << r.message;
  EXPECT_FALSE(r.plan_only);
  EXPECT_EQ(rig.motion.executed.size(), 1U);
}

TEST(MoveTo, PoseTargetUsesFreePlanner)
{
  MoveRig rig;
  pm::MoveToRequest req;
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(0.4, 0.1, 0.6);
  req.tcp_pose = p;
  pm::PlanKind kind = pm::PlanKind::NAMED;
  double scaling = 0.0;
  rig.motion.on_plan = [&](const pm::PlanRequest & r) {
      kind = r.kind;
      scaling = r.velocity_scaling;
    };
  req.velocity_scaling = 0.9;
  EXPECT_TRUE(rig.run(req).success);
  EXPECT_EQ(kind, pm::PlanKind::FREE);
  EXPECT_DOUBLE_EQ(scaling, rig.config.max_velocity_scaling);
}

TEST(MoveTo, PlanOnlyWhenExecutionDisabled)
{
  MoveRig rig;
  rig.execution = false;
  pm::MoveToRequest req;
  req.named_target = "home";
  const auto r = rig.run(req);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.plan_only);
  EXPECT_EQ(r.failure_code, failure::SAFETY_GATE_CLOSED);
  EXPECT_EQ(r.message, "plan_only_ok");
  EXPECT_TRUE(rig.motion.executed.empty());
}

TEST(MoveTo, SceneWrittenBeforePlanning)
{
  MoveRig rig;
  bool scene_written = false;
  bool planned_after_scene = false;
  rig.scene = [&](std::string *) {
      scene_written = true;
      return true;
    };
  rig.motion.on_plan = [&](const pm::PlanRequest &) {planned_after_scene = scene_written;};
  pm::MoveToRequest req;
  req.named_target = "home";
  EXPECT_TRUE(rig.run(req).success);
  EXPECT_TRUE(planned_after_scene);
}

TEST(MoveTo, SceneFailureDoesNotPlanOrMove)
{
  MoveRig rig;
  rig.scene = [](std::string * why) {
      *why = "timeout";
      return false;
    };
  pm::MoveToRequest req;
  req.named_target = "home";
  const auto r = rig.run(req);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_code, failure::DEPENDENCY_UNAVAILABLE);
  EXPECT_EQ(r.message, "scene_update_failed:timeout");
  EXPECT_TRUE(rig.motion.planned.empty());
  EXPECT_TRUE(rig.motion.executed.empty());
}

TEST(MoveTo, NoTarget)
{
  MoveRig rig;
  const auto r = rig.run(pm::MoveToRequest{});
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.message, "no_target");
  EXPECT_TRUE(rig.motion.planned.empty());
}

TEST(MoveTo, PlanFailure)
{
  MoveRig rig;
  rig.motion.plan_fail["move_to"] = failure::PLAN_NO_IK;
  pm::MoveToRequest req;
  req.named_target = "home";
  EXPECT_EQ(rig.run(req).failure_code, failure::PLAN_NO_IK);
}

TEST(MoveTo, GateClosesDuringExecution)
{
  MoveRig rig;
  rig.motion.during_execute = [&](const std::string &) {rig.execution = false;};
  pm::MoveToRequest req;
  req.named_target = "home";
  const auto r = rig.run(req);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_code, failure::SAFETY_GATE_CLOSED);
  EXPECT_EQ(rig.motion.stop_calls, 1);
}

TEST(MoveTo, Cancel)
{
  MoveRig rig;
  rig.motion.during_execute = [&](const std::string &) {rig.cancel = true;};
  pm::MoveToRequest req;
  req.named_target = "home";
  EXPECT_EQ(rig.run(req).failure_code, failure::CANCELED);
}
