#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "peach2_end_effector/end_effector.hpp"
#include "peach2_manipulation/command_gate.hpp"
#include "peach2_manipulation/decision_client.hpp"
#include "peach2_manipulation/motion_backend.hpp"
#include "peach2_manipulation/staging_geometry.hpp"

namespace peach2_manipulation
{

enum class CycleStage : uint8_t
{
  PREPARE_TOOL = 0,
  TRANSIT_STAGING,
  APPROACH_PREGRASP,
  VERIFY_PREGRASP,
  INSERT,
  CUT,
  CONFIRM,
  RETREAT,
  TRANSIT_RELEASE,
  RELEASE,
  DONE,
};

const char * to_string(CycleStage stage);

/// Which bags are collision objects while planning / executing (主审跨包决定).
enum class ScenePhase : uint8_t
{
  APPROACH = 0,   ///< every bag incl. the current target (transit, staging -> pregrasp)
  CONTACT = 1,    ///< current target removed (insert, cut, retreat, release transit)
};

const char * to_string(ScenePhase phase);

enum class CycleMode : uint8_t {PREGRASP_ONLY = 0, FULL = 1};

/// Same numeric values as peach2_interfaces/msg/HarvestResult.
enum class Outcome : uint8_t {SUCCEEDED = 0, SKIPPED = 1, FAILED = 2, CANCELED = 3};
enum class Reached : uint8_t
{
  NONE = 0, PREGRASP = 1, INSERTED = 2, CUT_CONFIRMED = 3, RETREATED = 4, RELEASED = 5,
};

struct CycleRequest
{
  std::string request_id;
  std::string target_id;
  std::string tool_id;
  CycleMode mode{CycleMode::FULL};
  /// CheckReachability: never execute even when execution is enabled.
  bool plan_only{false};
};

struct CycleConfig
{
  double transit_velocity_scaling{0.1};
  double transit_acceleration_scaling{0.1};
  double approach_speed_mps{0.05};
  double retreat_speed_mps{0.03};
  double staging_distance_m{0.20};
  int roll_samples{7};
  ResidualTolerance residual;
  int max_corrections{1};
  double model_shift_tol_m{0.01};
  double insert_lateral_tol_m{0.005};
  double start_tolerance_rad{0.02};
  std::string release_named_target{"harvest_stow"};
  bool pregrasp_only_retreat{true};
  int cut_retry_max{1};
  double confirm_margin_s{0.5};
  double execute_timeout_scale{2.0};
  double execute_timeout_margin_s{5.0};
};

struct CycleResult
{
  std::string target_id;
  std::string tool_id;
  Outcome outcome{Outcome::FAILED};
  Reached reached{Reached::NONE};
  uint32_t failure_code{0};
  std::string reason;
  bool recovery_required{false};
  bool plan_only{false};
  double cycle_time_s{0.0};
  std::vector<std::string> stage_names;
  std::vector<double> stage_times_s;
  double radial_margin_m{0.0};
  double axial_margin_m{0.0};
};

struct CycleDeps
{
  MotionBackend * motion{nullptr};
  peach2_end_effector::EndEffector * ee{nullptr};
  DecisionClient * decisions{nullptr};
  TargetSource * targets{nullptr};
  std::function<GateVerdict(GateStage, bool new_trajectory)> gate;
  std::function<EffectiveEnables()> enables;
  std::function<bool()> cancel_requested;
  std::function<double()> now_s;                  ///< steady clock
  std::function<void(double)> sleep_s;
  std::function<void(CycleStage, int, double)> on_stage;   ///< optional feedback
  std::function<void(const std::string &)> log;            ///< optional
  /// Optional: bring the planning scene to `phase` for `target_id`; false = not applied.
  /// The caller restores the scene after run() returns (the cycle never does).
  std::function<bool(ScenePhase phase, const std::string & target_id, std::string * why)> scene;
};

/// One harvest cycle (方案 §10.1). Pure orchestration over injected seams:
/// PREPARE_TOOL -> TRANSIT_STAGING -> APPROACH_PREGRASP -> VERIFY_PREGRASP -> [PREGRASP_ONLY]
/// -> INSERT -> CUT -> CONFIRM -> RETREAT -> TRANSIT_RELEASE -> RELEASE -> DONE.
///
/// Scene: APPROACH before anything is planned, CONTACT before the insert is planned; a scene
/// update failure is DEPENDENCY_UNAVAILABLE.
///
/// With execution disabled (or request.plan_only) the same planning chain runs without
/// executing anything or touching the tool; `plan_only` is set on every result. A fully planned
/// chain is SKIPPED / NONE with reason `planned` (reached stays NONE), so a consumer that ignores
/// `plan_only` never counts it as harvested; any plan-only failure carries a non-NONE code.
class HarvestCycle
{
public:
  HarvestCycle(CycleConfig config, CycleDeps deps);

  CycleResult run(const CycleRequest & request);

  const CycleConfig & config() const {return config_;}

private:
  struct Run;
  struct Selection;

  void begin_stage(Run & run, CycleStage stage);
  CycleResult finish(Run & run, Outcome outcome, uint32_t code, const std::string & reason);
  CycleResult plan_chain(Run & run);
  bool set_scene(const Run & run, ScenePhase phase, std::string & why);
  std::optional<Selection> select_roll(Run & run, uint32_t & code, std::string & reason);
  ExecResult exec_segment(
    Run & run, JointTrajectory trajectory, std::optional<PlanRequest> replan, GateStage stage,
    bool into_canopy);
  bool retreat(Run & run, std::string & why, uint32_t & stop_code);
  CycleResult fail_in_canopy(Run & run, Outcome outcome, uint32_t code, const std::string & reason);
  std::optional<DecisionView> requery(
    Run & run, const DecisionView & previous, uint32_t & code, std::string & reason);
  void note_tool_fault(Run & run);
  bool canceled(Run & run);
  void log(const std::string & msg) const;

  CycleConfig config_;
  CycleDeps deps_;
};

}  // namespace peach2_manipulation
