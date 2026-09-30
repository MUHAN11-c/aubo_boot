// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "peach2_task/core/ledger.hpp"

namespace core = peach2_task::core;
namespace fs = std::filesystem;

namespace
{

class TempDir : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() /
      ("peach2_task_ledger_" + std::to_string(::getpid()) + "_" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(root_);
  }
  void TearDown() override {fs::remove_all(root_);}

  static nlohmann::json read_json(const fs::path & p)
  {
    std::ifstream in(p);
    std::stringstream ss;
    ss << in.rdbuf();
    return nlohmann::json::parse(ss.str());
  }

  fs::path root_;
};

std::chrono::system_clock::time_point at(int64_t unix_s, int64_t us)
{
  return std::chrono::system_clock::time_point(
    std::chrono::seconds(unix_s) + std::chrono::microseconds(us));
}

}  // namespace

TEST(Ledger, SafeRequestIds)
{
  EXPECT_TRUE(core::is_safe_request_id("field_pregrasp_001"));
  EXPECT_TRUE(core::is_safe_request_id("a.b-c_D9"));
  EXPECT_FALSE(core::is_safe_request_id(""));
  EXPECT_FALSE(core::is_safe_request_id("."));
  EXPECT_FALSE(core::is_safe_request_id(".."));
  EXPECT_FALSE(core::is_safe_request_id(".hidden"));
  EXPECT_FALSE(core::is_safe_request_id("../escape"));
  EXPECT_FALSE(core::is_safe_request_id("a/b"));
  EXPECT_FALSE(core::is_safe_request_id("with space"));
  EXPECT_FALSE(core::is_safe_request_id(std::string(129, 'a')));
  EXPECT_TRUE(core::is_safe_request_id(std::string(128, 'a')));
}

TEST(Ledger, ResolveRequestId)
{
  const auto t = at(1790752620, 123456);  // 2026-09-30T07:17:00.123456Z
  const auto autoid = core::resolve_request_id("", t);
  ASSERT_TRUE(autoid.ok);
  EXPECT_EQ(autoid.id, "auto_20260930T071700_123456Z");
  EXPECT_TRUE(core::is_safe_request_id(autoid.id));

  const auto given = core::resolve_request_id("run_1", t);
  ASSERT_TRUE(given.ok);
  EXPECT_EQ(given.id, "run_1");

  const auto bad = core::resolve_request_id("../x", t);
  EXPECT_FALSE(bad.ok);
  EXPECT_FALSE(bad.error.empty());
  EXPECT_EQ(core::utc_iso8601(t), "2026-09-30T07:17:00.123456Z");
}

TEST_F(TempDir, CreateRefusesExistingDirectory)
{
  std::string err;
  EXPECT_FALSE(core::Ledger::exists(root_, "r1"));
  auto ledger = core::Ledger::create(root_, "r1", &err);
  ASSERT_TRUE(ledger) << err;
  EXPECT_EQ(ledger->dir(), root_ / "r1");
  EXPECT_TRUE(core::Ledger::exists(root_, "r1"));
  EXPECT_FALSE(core::Ledger::create(root_, "r1", &err));
  EXPECT_NE(err.find("already exists"), std::string::npos);
  EXPECT_FALSE(core::Ledger::create(root_, "../r2", &err));
}

TEST_F(TempDir, WritesLedgerAtomically)
{
  std::string err;
  auto ledger = core::Ledger::create(root_, "r1", &err);
  ASSERT_TRUE(ledger) << err;

  core::LedgerDoc doc;
  doc.request_id = "r1";
  doc.intent = "PREGRASP_ONLY";
  doc.tool_id = "adaptive_shear_v1";
  doc.limits.max_targets = 3;
  doc.started_utc = "2026-09-30T07:17:00.000000Z";
  doc.counts.attempted = 1;
  doc.counts.succeeded = 1;
  doc.counts.plan_only = 2;
  doc.scene_epoch = 5;
  doc.reachability.push_back(
    core::ReachabilityRecord{"target_3", false, 31, "PLAN_COLLISION", "hits peach_bag_1"});
  core::TargetRecord r;
  r.target_id = "target_1";
  r.outcome = "SUCCEEDED";
  r.reached = "PREGRASP";
  r.stage_names = {"approach"};
  r.stage_times_s = {1.5};
  doc.targets.push_back(r);
  r.target_id = "target_4";
  r.plan_only = true;
  doc.targets.push_back(r);
  ASSERT_TRUE(ledger->write(doc, &err)) << err;

  const auto path = root_ / "r1" / core::Ledger::kLedgerFile;
  ASSERT_TRUE(fs::exists(path));
  EXPECT_FALSE(fs::exists(path.string() + ".tmp"));
  auto j = read_json(path);
  EXPECT_EQ(j["schema"], "peach2_task/ledger/2");
  EXPECT_EQ(j["request_id"], "r1");
  EXPECT_TRUE(j["finished_utc"].is_null());
  EXPECT_TRUE(j["termination_reason"].is_null());
  EXPECT_EQ(j["policy"]["max_targets"], 3);
  EXPECT_EQ(j["counts"]["succeeded"], 1);
  EXPECT_EQ(j["targets"][0]["target_id"], "target_1");
  EXPECT_EQ(j["targets"][0]["stage_times_s"][0], 1.5);
  EXPECT_EQ(j["targets"][0]["plan_only"], false);
  EXPECT_EQ(j["targets"][1]["plan_only"], true);
  EXPECT_EQ(j["counts"]["plan_only"], 2);
  EXPECT_EQ(j["scene_epoch"], 5);
  EXPECT_EQ(j["reachability"]["target_3"]["reachable"], false);
  EXPECT_EQ(j["reachability"]["target_3"]["failure_code"], 31);
  EXPECT_EQ(j["reachability"]["target_3"]["failure_name"], "PLAN_COLLISION");
  EXPECT_EQ(j["reachability"]["target_3"]["reason"], "hits peach_bag_1");

  doc.finished_utc = "2026-09-30T07:20:00.000000Z";
  doc.termination_reason = "completed";
  ASSERT_TRUE(ledger->write(doc, &err)) << err;
  j = read_json(path);
  EXPECT_EQ(j["termination_reason"], "completed");

  core::ReworkEntry e{"target_2", "tool", "radial", 23, true};
  ASSERT_TRUE(ledger->write_rework("r1", {e}, &err)) << err;
  const auto rw = read_json(root_ / "r1" / core::Ledger::kReworkFile);
  EXPECT_EQ(rw["schema"], "peach2_task/rework/1");
  EXPECT_EQ(rw["entries"][0]["kind"], "tool");
  EXPECT_EQ(rw["entries"][0]["failure_code"], 23);
}

TEST_F(TempDir, AtomicWriteReportsErrors)
{
  std::string err;
  EXPECT_FALSE(core::atomic_write_file(root_ / "missing_dir" / "f.json", "{}", &err));
  EXPECT_NE(err.find("open"), std::string::npos);
}
