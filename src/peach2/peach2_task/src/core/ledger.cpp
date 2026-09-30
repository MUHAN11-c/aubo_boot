// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/core/ledger.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <system_error>

#include <nlohmann/json.hpp>

namespace peach2_task::core
{
namespace
{

using nlohmann::json;

std::tm to_utc_tm(std::chrono::system_clock::time_point t)
{
  const std::time_t secs = std::chrono::system_clock::to_time_t(t);
  std::tm tm{};
  gmtime_r(&secs, &tm);
  return tm;
}

long micros_of(std::chrono::system_clock::time_point t)
{
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
    t.time_since_epoch()).count();
  return static_cast<long>(((us % 1000000) + 1000000) % 1000000);
}

json record_json(const TargetRecord & r)
{
  return json{
    {"target_id", r.target_id},
    {"tool_id", r.tool_id},
    {"outcome", r.outcome},
    {"reached", r.reached},
    {"failure_code", r.failure_code},
    {"failure_name", r.failure_name},
    {"policy", r.policy},
    {"reason", r.reason},
    {"rework_kind", r.rework_kind},
    {"recovery_required", r.recovery_required},
    {"plan_only", r.plan_only},
    {"attempts", r.attempts},
    {"model_revision", r.model_revision},
    {"cycle_time_s", r.cycle_time_s},
    {"stage_names", r.stage_names},
    {"stage_times_s", r.stage_times_s},
    {"radial_margin_m", r.radial_margin_m},
    {"axial_margin_m", r.axial_margin_m},
    {"started_utc", r.started_utc},
    {"finished_utc", r.finished_utc},
  };
}

}  // namespace

bool is_safe_request_id(const std::string & id)
{
  if (id.empty() || id.size() > 128 || id.front() == '.') {
    return false;
  }
  for (const char ch : id) {
    const bool ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
      (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.';
    if (!ok) {
      return false;
    }
  }
  return true;
}

RequestIdResult resolve_request_id(
  const std::string & requested, std::chrono::system_clock::time_point now)
{
  RequestIdResult out;
  if (requested.empty()) {
    const std::tm tm = to_utc_tm(now);
    char buf[64];
    std::snprintf(
      buf, sizeof(buf), "auto_%04d%02d%02dT%02d%02d%02d_%06ldZ", tm.tm_year + 1900,
      tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, micros_of(now));
    out.ok = true;
    out.id = buf;
    return out;
  }
  if (!is_safe_request_id(requested)) {
    out.error = "request_id must match [A-Za-z0-9._-]{1,128} and not start with '.'";
    return out;
  }
  out.ok = true;
  out.id = requested;
  return out;
}

std::string utc_iso8601(std::chrono::system_clock::time_point t)
{
  const std::tm tm = to_utc_tm(t);
  char buf[64];
  std::snprintf(
    buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%06ldZ", tm.tm_year + 1900, tm.tm_mon + 1,
    tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, micros_of(t));
  return buf;
}

std::string serialize_ledger(const LedgerDoc & doc)
{
  json targets = json::array();
  for (const auto & r : doc.targets) {
    targets.push_back(record_json(r));
  }
  json reachability = json::object();
  for (const auto & r : doc.reachability) {
    reachability[r.target_id] = json{
      {"reachable", r.reachable},
      {"failure_code", r.failure_code},
      {"failure_name", r.failure_name},
      {"reason", r.reason},
    };
  }
  json j{
    {"schema", "peach2_task/ledger/2"},
    {"request_id", doc.request_id},
    {"intent", doc.intent},
    {"tool_id", doc.tool_id},
    {"target_list", doc.target_list},
    {"policy", {
        {"max_targets", doc.limits.max_targets},
        {"target_harvest_ratio", doc.limits.target_harvest_ratio},
        {"per_target_timeout_s", doc.limits.per_target_timeout_s},
        {"empty_survey_limit", doc.limits.empty_survey_limit},
      }},
    {"scene_epoch", doc.scene_epoch},
    {"started_utc", doc.started_utc},
    {"finished_utc", doc.finished_utc.empty() ? json(nullptr) : json(doc.finished_utc)},
    {"termination_reason",
      doc.termination_reason.empty() ? json(nullptr) : json(doc.termination_reason)},
    {"counts", {
        {"discovered", doc.counts.discovered},
        {"attempted", doc.counts.attempted},
        {"succeeded", doc.counts.succeeded},
        {"skipped", doc.counts.skipped},
        {"failed", doc.counts.failed},
        {"plan_only", doc.counts.plan_only},
      }},
    {"discovered_ids", doc.discovered_ids},
    {"reachability", reachability},
    {"targets", targets},
  };
  return j.dump(1) + "\n";
}

std::string serialize_rework(const std::string & request_id, const std::vector<ReworkEntry> & e)
{
  json entries = json::array();
  for (const auto & entry : e) {
    entries.push_back(
      json{
        {"target_id", entry.target_id},
        {"kind", entry.kind},
        {"reason", entry.reason},
        {"failure_code", entry.failure_code},
        {"attempted", entry.attempted},
      });
  }
  json j{
    {"schema", "peach2_task/rework/1"},
    {"request_id", request_id},
    {"entries", entries},
  };
  return j.dump(1) + "\n";
}

bool atomic_write_file(
  const std::filesystem::path & path, const std::string & content, std::string * error)
{
  const std::filesystem::path tmp = path.string() + ".tmp";
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  auto fail = [&](const char * what) {
      if (error != nullptr) {
        *error = std::string(what) + " " + tmp.string() + ": " + std::strerror(errno);
      }
      return false;
    };
  if (fd < 0) {
    return fail("open");
  }
  const char * data = content.data();
  std::size_t left = content.size();
  while (left > 0) {
    const ssize_t n = ::write(fd, data, left);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      ::close(fd);
      return fail("write");
    }
    data += n;
    left -= static_cast<std::size_t>(n);
  }
  if (::fsync(fd) != 0) {
    ::close(fd);
    return fail("fsync");
  }
  if (::close(fd) != 0) {
    return fail("close");
  }
  if (::rename(tmp.c_str(), path.c_str()) != 0) {
    return fail("rename");
  }
  return true;
}

std::unique_ptr<Ledger> Ledger::create(
  const std::filesystem::path & runs_dir, const std::string & request_id, std::string * error)
{
  if (!is_safe_request_id(request_id)) {
    if (error != nullptr) {
      *error = "unsafe request_id '" + request_id + "'";
    }
    return nullptr;
  }
  std::error_code ec;
  std::filesystem::create_directories(runs_dir, ec);
  if (ec) {
    if (error != nullptr) {
      *error = "cannot create " + runs_dir.string() + ": " + ec.message();
    }
    return nullptr;
  }
  const std::filesystem::path dir = runs_dir / request_id;
  // create_directory returns false (no error) when it already exists: never reuse a run dir.
  const bool created = std::filesystem::create_directory(dir, ec);
  if (ec || !created) {
    if (error != nullptr) {
      *error = ec ? "cannot create " + dir.string() + ": " + ec.message() :
        "run directory already exists: " + dir.string();
    }
    return nullptr;
  }
  return std::unique_ptr<Ledger>(new Ledger(dir));
}

bool Ledger::exists(const std::filesystem::path & runs_dir, const std::string & request_id)
{
  std::error_code ec;
  return std::filesystem::exists(runs_dir / request_id, ec);
}

bool Ledger::write(const LedgerDoc & doc, std::string * error) const
{
  return atomic_write_file(dir_ / kLedgerFile, serialize_ledger(doc), error);
}

bool Ledger::write_rework(
  const std::string & request_id, const std::vector<ReworkEntry> & entries,
  std::string * error) const
{
  return atomic_write_file(dir_ / kReworkFile, serialize_rework(request_id, entries), error);
}

}  // namespace peach2_task::core
