#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "peach2_end_effector/failure_codes.hpp"
#include "peach2_end_effector/io_backend.hpp"
#include "peach2_end_effector/plugins.hpp"
#include "peach2_end_effector/tool_profile.hpp"
#include "peach2_manipulation/command_gate.hpp"
#include "peach2_manipulation/decision_client.hpp"
#include "peach2_manipulation/harvest_cycle.hpp"
#include "peach2_manipulation/motion_backend.hpp"

namespace peach2_fakes
{

namespace pm = peach2_manipulation;
namespace ee = peach2_end_effector;

struct FakeClock
{
  double t{1000.0};
  double now() const {return t;}
  void sleep(double dt) {t += dt;}
};

/// Kinematics-free motion fake: every Cartesian goal maps to a unique joint vector; executing a
/// trajectory moves the "arm" to its last point and the TCP to the pose known for that point.
/// Executed trajectories that were never returned by plan() are labelled "reverse".
class FakeMotion : public pm::MotionBackend
{
public:
  std::vector<double> joints{0.0, -0.5, 1.0, 0.0, 1.2, 0.0};
  Eigen::Isometry3d tcp{Eigen::Isometry3d::Identity()};
  std::map<std::string, uint32_t> plan_fail;          ///< label -> failure code (always)
  std::map<std::string, pm::ExecResult> exec_fail;    ///< label -> result (first time only)
  std::map<std::string, Eigen::Vector3d> tcp_error;   ///< label -> TCP offset after execute
  bool validate_ok{true};
  bool force_sensing{false};
  int stop_calls{0};
  std::vector<std::string> planned;
  std::vector<std::string> executed;
  std::function<void(const std::string & label)> during_execute;
  std::function<void(const pm::PlanRequest &)> on_plan;

  pm::PlanResult plan(const pm::PlanRequest & r) override
  {
    planned.push_back(r.label);
    if (on_plan) {
      on_plan(r);
    }
    auto it = plan_fail.find(r.label);
    if (it != plan_fail.end()) {
      return {false, it->second, "fake_plan_fail", {}};
    }
    const std::vector<double> start = r.start_joints ? *r.start_joints : joints;
    std::vector<double> goal;
    Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
    if (r.kind == pm::PlanKind::NAMED) {
      goal = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
      goal_pose.translation() = Eigen::Vector3d(0.3, 0.3, 0.3);
    } else {
      goal = joints_of(r.tcp_goal);
      goal_pose = r.tcp_goal;
    }
    pm::JointTrajectory t;
    t.joint_names = {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint",
      "wrist2_joint", "wrist3_joint"};
    std::vector<double> mid(start.size());
    for (size_t i = 0; i < start.size(); ++i) {
      mid[i] = 0.5 * (start[i] + goal[i]);
    }
    t.points.push_back({0.0, start, std::vector<double>(6, 0.0), std::vector<double>(6, 0.0)});
    t.points.push_back({1.0, mid, std::vector<double>(6, 0.1), std::vector<double>(6, 0.0)});
    t.points.push_back({2.0, goal, std::vector<double>(6, 0.0), std::vector<double>(6, 0.0)});
    if (!r.start_joints) {
      poses_.push_back({start, tcp});
    }
    poses_.push_back({goal, goal_pose});
    plans_.push_back({start, goal, r.label});
    return {true, 0U, "ok", t};
  }

  pm::ExecResult execute(const pm::JointTrajectory & t, const pm::ExecOptions & o) override
  {
    const std::string label = label_of(t);
    executed.push_back(label);
    if (during_execute) {
      during_execute(label);
    }
    if (o.abort_probe) {
      if (auto abort = o.abort_probe()) {
        ++stop_calls;
        return {false, abort->failure_code, abort->reason};
      }
    }
    auto it = exec_fail.find(label);
    if (it != exec_fail.end()) {
      const pm::ExecResult r = it->second;
      exec_fail.erase(it);
      ++stop_calls;
      return r;
    }
    joints = t.points.back().positions;
    tcp = pose_of(joints);
    auto err = tcp_error.find(label);
    if (err != tcp_error.end()) {
      tcp.translation() += err->second;
      // Correction plans start from the real (offset) pose.
      poses_.push_back({joints, tcp});
    }
    return {true, 0U, "ok"};
  }

  bool validate(const pm::JointTrajectory &, std::string * why) override
  {
    if (!validate_ok && why) {
      *why = "fake_collision";
    }
    return validate_ok;
  }
  std::optional<std::vector<double>> current_joints() override {return joints;}
  std::optional<Eigen::Isometry3d> current_tcp() override {return tcp;}
  void stop() override {++stop_calls;}
  bool has_force_sensing() const override {return force_sensing;}

  static int count(const std::vector<std::string> & v, const std::string & label)
  {
    return static_cast<int>(std::count(v.begin(), v.end(), label));
  }

  static std::vector<double> joints_of(const Eigen::Isometry3d & p)
  {
    const Eigen::AngleAxisd aa(p.linear());
    const Eigen::Vector3d r = aa.angle() * aa.axis();
    return {p.translation().x(), p.translation().y(), p.translation().z(), r.x(), r.y(), r.z()};
  }

private:
  struct PosePair
  {
    std::vector<double> q;
    Eigen::Isometry3d pose;
  };
  struct PlanRecord
  {
    std::vector<double> start;
    std::vector<double> goal;
    std::string label;
  };
  std::vector<PosePair> poses_;
  std::vector<PlanRecord> plans_;

  std::string label_of(const pm::JointTrajectory & t) const
  {
    const auto & s = t.points.front().positions;
    const auto & g = t.points.back().positions;
    for (auto it = plans_.rbegin(); it != plans_.rend(); ++it) {
      if (pm::max_joint_deviation(it->start, s) < 1e-9 &&
        pm::max_joint_deviation(it->goal, g) < 1e-9)
      {
        return it->label;
      }
    }
    return "reverse";
  }
  Eigen::Isometry3d pose_of(const std::vector<double> & q) const
  {
    for (auto it = poses_.rbegin(); it != poses_.rend(); ++it) {
      if (pm::max_joint_deviation(it->q, q) < 1e-9) {
        return it->pose;
      }
    }
    return Eigen::Isometry3d::Identity();
  }
};

class FakeDecisions : public pm::DecisionClient
{
public:
  explicit FakeDecisions(std::shared_ptr<FakeClock> clock)
  : clock_(std::move(clock)) {}

  std::vector<std::optional<pm::DecisionView>> script;
  std::vector<uint64_t> min_revisions;

  std::optional<pm::DecisionView> get(
    const std::string &, const std::string &, uint64_t min_revision) override
  {
    min_revisions.push_back(min_revision);
    const size_t i = std::min(calls_, script.size() - 1);
    ++calls_;
    return script[i];
  }
  double now_s() override {return clock_->now();}
  size_t calls() const {return calls_;}

private:
  std::shared_ptr<FakeClock> clock_;
  size_t calls_{0};
};

class FakeTargets : public pm::TargetSource
{
public:
  std::optional<ee::TargetGeometry> geometry;
  std::optional<ee::TargetGeometry> get(const std::string &) override {return geometry;}
};

inline ee::TargetGeometry default_geometry()
{
  ee::TargetGeometry g;
  g.target_id = "t1";
  g.axis = Eigen::Vector3d::UnitZ();
  g.bottom = Eigen::Vector3d(0.5, 0.0, 0.80);
  g.neck = Eigen::Vector3d(0.5, 0.0, 0.88);
  g.d95_m = 0.10;
  g.length_m = 0.08;
  return g;
}

inline pm::DecisionView default_decision(double now)
{
  pm::DecisionView d;
  d.target_id = "t1";
  d.tool_id = "adaptive_shear_v1";
  d.revision = 1;
  d.valid_until_s = now + 120.0;
  d.approach_allowed = true;
  d.sleeve_allowed = true;
  d.cut_allowed = true;
  d.radial_margin_m = 0.008;
  d.axial_margin_m = 0.006;
  d.pregrasp_tcp = Eigen::Isometry3d::Identity();
  d.pregrasp_tcp.translation() = Eigen::Vector3d(0.5, 0.0, 0.77);
  d.blade_target = Eigen::Vector3d(0.5, 0.0, 0.88);
  d.insert_travel_m = 0.189;
  return d;
}

/// Full rig: real CommandGate, real AdaptiveShearV1 on MockIoBackend, fake motion / decisions.
struct CycleRig
{
  std::shared_ptr<FakeClock> clock{std::make_shared<FakeClock>()};
  FakeMotion motion;
  FakeDecisions decisions{clock};
  FakeTargets targets;
  std::shared_ptr<ee::MockIoBackend> io;
  std::unique_ptr<ee::EndEffector> tool;
  pm::CommandGate gate;
  bool execution{true};
  bool grasp{true};
  bool tool_enabled{true};
  bool heartbeat{true};
  bool cancel{false};
  pm::CycleConfig config;
  std::vector<std::string> stages;
  std::function<bool(pm::ScenePhase, const std::string &, std::string *)> scene;

  explicit CycleRig(
    ee::MockIoBackend::Config mock = {}, ee::CurrentSignatureConfig current = {})
  : gate(make_gate_config())
  {
    auto c = clock;
    io = std::make_shared<ee::MockIoBackend>(mock, [c]() {return c->now();});
    ee::EndEffectorContext ctx;
    ctx.profile = ee::load_tool_profile("adaptive_shear_v1", PEACH2_TOOL_CONFIG_DIR);
    ctx.io = io;
    ctx.current = current;
    ctx.now_s = [c]() {return c->now();};
    ctx.sleep_s = [c](double dt) {c->sleep(dt);};
    tool = std::make_unique<ee::AdaptiveShearV1>();
    tool->initialize(ctx);
    gate.set_active(true);
    targets.geometry = default_geometry();
    decisions.script = {default_decision(clock->now())};
  }

  static pm::GateConfig make_gate_config()
  {
    pm::GateConfig g;
    g.require_robot_status = false;
    return g;
  }

  void heartbeat_tick()
  {
    if (!heartbeat) {
      return;
    }
    pm::EnablesSample s;
    s.execution = execution;
    s.grasp = grasp;
    s.tool = tool_enabled;
    s.received_s = clock->now();
    gate.on_enables(s);
  }

  pm::CycleResult run(pm::CycleMode mode = pm::CycleMode::FULL, bool plan_only = false)
  {
    pm::CycleDeps d;
    d.motion = &motion;
    d.ee = tool.get();
    d.decisions = &decisions;
    d.targets = &targets;
    d.gate = [this](pm::GateStage s, bool nt) {
        heartbeat_tick();
        gate.set_cancel(cancel);
        return gate.check(s, clock->now(), nt);
      };
    d.enables = [this]() {
        heartbeat_tick();
        return gate.enables(clock->now());
      };
    d.cancel_requested = [this]() {return cancel;};
    d.now_s = [this]() {return clock->now();};
    d.sleep_s = [this](double dt) {clock->sleep(dt);};
    d.on_stage = [this](pm::CycleStage s, int, double) {stages.emplace_back(pm::to_string(s));};
    d.scene = scene;
    pm::HarvestCycle cycle(config, d);
    pm::CycleRequest req;
    req.request_id = "r1";
    req.target_id = "t1";
    req.tool_id = "adaptive_shear_v1";
    req.mode = mode;
    req.plan_only = plan_only;
    return cycle.run(req);
  }
};

}  // namespace peach2_fakes
