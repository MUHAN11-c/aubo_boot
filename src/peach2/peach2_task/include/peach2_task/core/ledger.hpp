// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "peach2_task/core/batch_policy.hpp"

/// Run ledger: runs/<request_id>/ledger.json and rework.json, always written atomically
/// (write <file>.tmp, fsync, rename). One directory per batch; an existing directory is never
/// reused, so a request_id can never silently merge two batches.
namespace peach2_task::core
{

struct RequestIdResult
{
  bool ok = false;
  std::string id;
  std::string error;
};

/// Directory-name safe: [A-Za-z0-9._-], 1..128 chars, no leading '.', never "." / "..".
bool is_safe_request_id(const std::string & id);

/// Empty -> auto_<UTC yyyymmddThhmmss_uuuuuuZ>; otherwise validated as-is (never rewritten).
RequestIdResult resolve_request_id(
  const std::string & requested, std::chrono::system_clock::time_point now);

/// 2026-09-30T07:17:00.123456Z
std::string utc_iso8601(std::chrono::system_clock::time_point t);

struct TargetRecord
{
  std::string target_id;
  std::string tool_id;
  std::string outcome;   ///< SUCCEEDED | SKIPPED | FAILED | CANCELED
  std::string reached;   ///< NONE | PREGRASP | INSERTED | CUT_CONFIRMED | RETREATED | RELEASED
  uint32_t failure_code = 0;
  std::string failure_name;
  std::string policy;
  std::string reason;
  std::string rework_kind;  ///< empty on success
  bool recovery_required = false;
  bool plan_only = false;   ///< planned only; not counted as attempted
  uint32_t attempts = 0;
  uint64_t model_revision = 0;
  double cycle_time_s = 0.0;
  std::vector<std::string> stage_names;
  std::vector<double> stage_times_s;
  double radial_margin_m = 0.0;
  double axial_margin_m = 0.0;
  std::string started_utc;
  std::string finished_utc;
};

struct ReworkEntry
{
  std::string target_id;
  std::string kind;
  std::string reason;
  uint32_t failure_code = 0;
  bool attempted = false;
};

/// Latest CheckReachability answer for one target.
struct ReachabilityRecord
{
  std::string target_id;
  bool reachable = false;
  uint32_t failure_code = 0;
  std::string failure_name;
  std::string reason;  ///< CheckReachability.reasons[i]
};

struct LedgerDoc
{
  std::string request_id;
  std::string intent;
  std::string tool_id;
  std::vector<std::string> target_list;
  BatchLimits limits;
  uint32_t scene_epoch = 0;        ///< BeginScene epoch of the first survey; 0 = not begun
  std::string started_utc;
  std::string finished_utc;       ///< empty while running
  std::string termination_reason;  ///< empty while running
  Counts counts;
  std::vector<std::string> discovered_ids;
  std::vector<ReachabilityRecord> reachability;  ///< sorted by target_id
  std::vector<TargetRecord> targets;
};

std::string serialize_ledger(const LedgerDoc & doc);
std::string serialize_rework(const std::string & request_id, const std::vector<ReworkEntry> & e);

/// Writes `content` to `path` via `path`.tmp + fsync + rename(2).
bool atomic_write_file(
  const std::filesystem::path & path, const std::string & content, std::string * error);

class Ledger
{
public:
  static constexpr const char * kLedgerFile = "ledger.json";
  static constexpr const char * kReworkFile = "rework.json";

  /// Creates runs_dir (parents allowed) and runs_dir/request_id (must not exist).
  static std::unique_ptr<Ledger> create(
    const std::filesystem::path & runs_dir, const std::string & request_id, std::string * error);

  /// True if runs_dir/request_id already exists (used to reject a goal before accepting it).
  static bool exists(const std::filesystem::path & runs_dir, const std::string & request_id);

  const std::filesystem::path & dir() const {return dir_;}
  bool write(const LedgerDoc & doc, std::string * error) const;
  bool write_rework(
    const std::string & request_id, const std::vector<ReworkEntry> & entries,
    std::string * error) const;

private:
  explicit Ledger(std::filesystem::path dir)
  : dir_(std::move(dir)) {}

  std::filesystem::path dir_;
};

}  // namespace peach2_task::core
