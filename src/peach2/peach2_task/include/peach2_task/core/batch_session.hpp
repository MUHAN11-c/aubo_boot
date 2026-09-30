// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "peach2_task/core/batch_policy.hpp"
#include "peach2_task/core/enables_policy.hpp"
#include "peach2_task/core/ledger.hpp"
#include "peach2_task/core/selection.hpp"

/// State of one batch, shared by every BT node of the tree. Zero ROS: ROS leaves convert
/// their action/service answers into the plain structs below and call the matching `on_*`,
/// test doubles call the very same methods, so the batch logic is exercised without DDS.
namespace peach2_task::core
{

/// Mirror of BatchState.phase.
enum class Phase : uint8_t
{
  IDLE = 0, SURVEYING = 1, SELECTING = 2, OBSERVING = 3, HARVESTING = 4, WAITING_ACK = 5,
  PAUSED = 6, COMPLETED = 7, ABORTED = 8,
};
const char * phase_name(Phase phase);

/// Mirror of HarvestResult.OUTCOME_* / REACHED_*.
namespace outcome
{
constexpr uint8_t SUCCEEDED = 0;
constexpr uint8_t SKIPPED = 1;
constexpr uint8_t FAILED = 2;
constexpr uint8_t CANCELED = 3;
}  // namespace outcome
namespace reached
{
constexpr uint8_t NONE = 0;
constexpr uint8_t PREGRASP = 1;
constexpr uint8_t INSERTED = 2;
constexpr uint8_t CUT_CONFIRMED = 3;
constexpr uint8_t RETREATED = 4;
constexpr uint8_t RELEASED = 5;
}  // namespace reached
const char * outcome_name(uint8_t value);
const char * reached_name(uint8_t value);

struct Failure
{
  uint32_t code = fc::NONE;
  std::string reason;
  bool recovery_required = false;
  std::optional<Policy> policy_override;  ///< open target closed by an aborted batch (no code)
  std::string rework_override;

  bool active() const {return code != fc::NONE || !reason.empty() || recovery_required;}
  /// recovery_required wins over the code; a code-less failure is a plain skip.
  Policy policy() const;
};

/// Mirror of HarvestResult.
struct HarvestOutcome
{
  std::string target_id;
  std::string tool_id;
  uint8_t outcome = outcome::FAILED;
  uint8_t reached = reached::NONE;
  uint32_t failure_code = fc::NONE;
  std::string reason;
  bool recovery_required = false;
  bool plan_only = false;  ///< execution disabled: planned, nothing moved or actuated
  double cycle_time_s = 0.0;
  std::vector<std::string> stage_names;
  std::vector<double> stage_times_s;
  double radial_margin_m = 0.0;
  double axial_margin_m = 0.0;
};

struct ObserveOutcome
{
  bool converged = false;
  uint32_t failure_code = fc::NONE;
  uint64_t model_revision = 0;
  uint32_t n_views = 0;
  std::string message;
};

enum class DecisionLevel : uint8_t { APPROACH, SLEEVE, CUT };
bool decision_level_from_name(const std::string & name, DecisionLevel * out);

struct DecisionOutcome
{
  bool found = false;
  bool expired = false;  ///< valid_until already passed when received
  bool approach_allowed = false;
  bool sleeve_allowed = false;
  bool cut_allowed = false;
  uint64_t model_revision = 0;
  uint32_t failure_code = fc::NONE;
  std::string reason;
  double radial_margin_m = 0.0;
  double axial_margin_m = 0.0;
};

struct BatchRequest
{
  std::string request_id;
  Intent intent = Intent::PREGRASP_ONLY;
  std::string tool_id;
  std::vector<std::string> target_ids;
  BatchLimits limits;
};

struct SessionConfig
{
  SelectionConfig selection;
  bool ack_each_pregrasp = true;  ///< PREGRASP_ONLY dry run: operator ACK after every target
  double wait_retry_s = 5.0;      ///< [s] pause before retrying a WAIT-policy failure
};

enum class RetryVerdict : uint8_t { RETRY_NOW, RETRY_AFTER_WAIT, GIVE_UP };
enum class BatchEnd : uint8_t { SUCCEEDED, FAILED, CANCELED };

struct StateSnapshot
{
  std::string request_id;
  Phase phase = Phase::IDLE;
  std::string current_target_id;
  std::vector<std::string> blockers;
  Counts counts;
  bool recovery_required = false;
  std::string message;
};

class BatchSession
{
public:
  using Clock = std::function<double()>;  ///< monotonic [s]
  using WallClock = std::function<std::chrono::system_clock::time_point()>;

  BatchSession();
  BatchSession(Clock clock, WallClock wall_clock);

  /// `ledger` may be null (no disk output).
  void begin(BatchRequest request, SessionConfig config, std::unique_ptr<Ledger> ledger);
  bool active() const {return active_;}
  /// Records an open target, writes final ledger + rework, returns termination_reason.
  std::string finish(BatchEnd end);

  const BatchRequest & request() const {return request_;}
  const SessionConfig & config() const {return config_;}
  double now() const {return clock_();}

  // ---- phase / status ----
  void set_phase(Phase phase);
  Phase phase() const {return phase_;}
  void set_message(const std::string & message);
  void set_safety_blockers(const std::vector<std::string> & blockers);
  StateSnapshot snapshot() const;
  /// Bumped on every observable change; the node republishes BatchState when it moves.
  uint64_t revision() const {return revision_;}

  // ---- survey / selection ----
  /// BeginScene accepted on the first survey: later locks must carry this epoch.
  void on_scene_begun(uint32_t scene_epoch);
  bool scene_begun() const {return scene_begun_;}
  uint32_t scene_epoch() const {return scene_epoch_;}
  void on_locked_snapshot(ObservationSet set);
  bool has_locked_snapshot() const {return has_snapshot_;}
  /// Eligible ids to send to CheckReachability (claimed excluded, target list applied).
  std::vector<std::string> reach_query_ids() const;
  /// Picks, claims and starts the per-target clock. Empty when nothing is selectable.
  std::string select(const ReachMap & reach);
  /// True (and settles "no_targets") when the consecutive empty-survey limit is reached.
  bool on_empty_round();
  /// False (and settles) when max_targets / ratio / explicit list completion stops the batch.
  bool gate_allows_next();
  const std::map<std::string, std::string> & last_filtered() const {return last_filtered_;}

  // ---- current target ----
  const std::string & current_target() const {return current_target_;}
  /// Starts an attempt; it is a neck re-measure attempt when the previous one failed with
  /// policy REMEASURE_NECK (see neck_remeasure_pending()).
  void begin_attempt();
  uint32_t attempts() const {return attempts_;}
  /// True during an attempt that must run ObserveTarget(neck_remeasure) + CheckDecision(cut).
  bool neck_remeasure_pending() const {return neck_phase_;}
  bool target_deadline_exceeded() const;
  void set_failure(Failure failure);
  const Failure & failure() const {return failure_;}
  bool on_observe(const ObserveOutcome & observed);
  uint64_t model_revision() const {return model_revision_;}
  bool on_decision(const DecisionOutcome & decision, DecisionLevel level);
  bool on_harvest(const HarvestOutcome & result);
  RetryVerdict retry_verdict(uint32_t attempts_done, uint32_t max_attempts) const;
  void record_success();
  /// False when the failure policy is STOP_BATCH (batch aborted).
  bool record_skip();

  // ---- recovery ----
  bool recovery_required() const {return recovery_required_;}
  bool ack_granted() const {return ack_granted_;}
  const std::string & recovery_reason() const {return recovery_reason_;}
  void require_recovery(const std::string & reason);
  /// Operator ACK forwarded successfully. False if no recovery was pending.
  bool grant_ack();
  /// WaitForAck: true once the pending recovery has been ACKed and manipulation no longer
  /// latches recovery_required (clears it).
  bool consume_ack();
  /// /peach/manipulation/recovery_required. Survives begin(); a latch raised during a batch
  /// arms the task-side ACK gate. Only a task ACK releases the gate, never the latch dropping.
  void on_peer_recovery(bool required);
  bool peer_recovery_required() const {return peer_recovery_;}

  // ---- termination ----
  void settle(const std::string & reason);
  bool settled() const {return settled_ && abort_reason_.empty();}
  const std::string & settle_reason() const {return settle_reason_;}
  void abort(const std::string & reason);
  const std::string & abort_reason() const {return abort_reason_;}
  std::string termination_reason() const;

  // ---- results ----
  const Counts & counts() const {return counts_;}
  const std::vector<HarvestOutcome> & results() const {return results_;}
  const std::vector<ReworkEntry> & rework() const {return rework_;}
  const std::vector<TargetRecord> & records() const {return records_;}
  /// Latest CheckReachability answer per target (reasons included), as written to the ledger.
  const std::map<std::string, ReachabilityRecord> & reachability() const {return reachability_;}
  uint32_t ledger_write_failures() const {return ledger_failures_;}
  const std::string & last_ledger_error() const {return last_ledger_error_;}

private:
  void bump() {++revision_;}
  void start_target(const std::string & target_id);
  void close_target(TargetRecord record, HarvestOutcome result);
  TargetRecord base_record() const;
  LedgerDoc ledger_doc() const;
  void write_ledger();
  std::vector<ReworkEntry> final_rework(BatchEnd end) const;

  Clock clock_;
  WallClock wall_clock_;
  std::unique_ptr<Ledger> ledger_;
  BatchRequest request_;
  SessionConfig config_;
  bool active_ = false;
  uint64_t revision_ = 0;

  Phase phase_ = Phase::IDLE;
  std::string message_;
  std::vector<std::string> safety_blockers_;

  bool scene_begun_ = false;
  uint32_t scene_epoch_ = 0;
  ObservationSet snapshot_;
  bool has_snapshot_ = false;
  std::vector<std::string> discovered_ids_;
  std::set<std::string> discovered_set_;
  std::set<std::string> claimed_;
  std::map<std::string, Reach> unreachable_;
  std::map<std::string, ReachabilityRecord> reachability_;
  std::map<std::string, std::string> last_filtered_;
  uint32_t empty_rounds_ = 0;

  std::string current_target_;
  uint32_t attempts_ = 0;
  double target_started_s_ = 0.0;
  std::chrono::system_clock::time_point target_started_wall_;
  TargetDeadline deadline_;
  Failure failure_;
  uint64_t model_revision_ = 0;
  std::optional<DecisionOutcome> last_decision_;
  std::optional<HarvestOutcome> last_harvest_;
  bool target_plan_only_ = false;  ///< any HarvestTarget result of this target was plan-only
  bool neck_phase_ = false;        ///< current attempt is a neck re-measure attempt
  bool neck_remeasured_ = false;   ///< a neck re-measure followed the latest normal attempt

  bool recovery_required_ = false;
  bool ack_granted_ = false;
  bool peer_recovery_ = false;
  std::string recovery_reason_;

  bool settled_ = false;
  std::string settle_reason_;
  std::string abort_reason_;
  std::string termination_reason_;
  std::string started_utc_;
  std::string finished_utc_;

  Counts counts_;
  std::vector<HarvestOutcome> results_;
  std::vector<TargetRecord> records_;
  std::vector<ReworkEntry> rework_;
  uint32_t ledger_failures_ = 0;
  std::string last_ledger_error_;
};

}  // namespace peach2_task::core
