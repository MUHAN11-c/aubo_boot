# M0 契约冻结 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 冻结新架构跨包契约：`peach_interfaces` IDL、不可变 profile schema、canonical JSONL 事件 schema、golden MCAP 回放夹具，全部带失败先行测试（门 G0）。

**Architecture:** 新建唯一 IDL 包 `peach_interfaces`，不桥接 `peach_pose_msgs` / `peach_harvest_msgs`。Profile 与事件 schema 以 YAML/JSON Schema 文件为权威源，Python 纯函数校验 hash 与字段，零 ROS import。Golden MCAP 写入最小 ObservationFrame 等价通道，供后续 M2 回放钉死。

**Tech Stack:** ROS 2 Jazzy rosidl、ament_cmake、pytest、`mcap` Python 库（若 apt 无则用 `rosbag2_py` 写 MCAP）。

## Global Constraints

- 真机驱动栈冻结只读；禁止 `/aubo_dashboard/startup`；本里程碑不启动运动。
- Python：`/home/mu/Desktop/aubo_e5_jazzy_ws/aubo_py3.12/bin/python`；numpy 1.26.4。
- C++17；lint：C++ 100 列，Python flake8 99 列单引号。
- 不修改冻结驱动包；不保留 `legacy`/`v2` 并存目录。
- 权威关节顺序：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。

---

## File map

- Create: `src/peach_interfaces/package.xml`
- Create: `src/peach_interfaces/CMakeLists.txt`
- Create: `src/peach_interfaces/msg/*.msg`（枚举与核心类型）
- Create: `src/peach_interfaces/srv/BeginScene.srv`、`ControlTask.srv`
- Create: `src/peach_interfaces/action/{RunHarvest,SurveyScene,BuildTargetModel,ExecuteTarget}.action`
- Create: `src/peach_interfaces/schema/profile.schema.json`
- Create: `src/peach_interfaces/schema/event.schema.json`
- Create: `src/peach_interfaces/profiles/analysis_only_v1.yaml`
- Create: `src/peach_interfaces/test/test_idl_constants.cpp`
- Create: `src/peach_interfaces/test/test_profile_schema.py`
- Create: `src/peach_interfaces/test/test_event_schema.py`
- Create: `src/peach_interfaces/test/test_golden_mcap.py`
- Create: `src/peach_interfaces/test/golden/observation_frame_v1.mcap`（测试生成后入库）
- Create: `src/peach_common_py/` 仅在 M1；M0 不收缩 `peach_core`

---

### Task 1: Scaffold `peach_interfaces` so a constant test fails

**Files:**
- Create: `src/peach_interfaces/package.xml`
- Create: `src/peach_interfaces/CMakeLists.txt`
- Create: `src/peach_interfaces/msg/JobIntent.msg`
- Test: `src/peach_interfaces/test/test_idl_constants.cpp`

**Interfaces:**
- Produces: `peach_interfaces::msg::JobIntent::{ANALYZE_ONLY=0, EXECUTE=1}`

- [ ] **Step 1: Write the failing test**

```cpp
#include <gtest/gtest.h>
#include "peach_interfaces/msg/job_intent.hpp"

TEST(IdlConstants, JobIntentValues)
{
  EXPECT_EQ(peach_interfaces::msg::JobIntent::ANALYZE_ONLY, 0);
  EXPECT_EQ(peach_interfaces::msg::JobIntent::EXECUTE, 1);
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_interfaces
colcon test --packages-select peach_interfaces --event-handlers console_direct-
```

Expected: FAIL package not found or missing header.

- [ ] **Step 3: Write minimal implementation**

`JobIntent.msg`:

```
uint8 ANALYZE_ONLY=0
uint8 EXECUTE=1
uint8 value
```

ament_cmake + rosidl_generate_interfaces；gtest 链接生成消息。

- [ ] **Step 4: Run test to verify it passes**

- [ ] **Step 5: Commit**

```bash
git add src/peach_interfaces
git commit -m "feat(peach_interfaces): scaffold JobIntent IDL and constant test"
```

---

### Task 2: Freeze remaining enumerations

**Files:** `msg/JobState.msg`、`TargetQuality.msg`、`TargetOutcome.msg`、`MatchStatus.msg`

- [ ] **Step 1: Extend `test_idl_constants.cpp` with expected numeric values from the 2026-08-19 spec (JobState, quality grades, outcomes, AMBIGUOUS).**
- [ ] **Step 2: Run tests — expect missing headers.**
- [ ] **Step 3: Add `.msg` files with matching constants.**
- [ ] **Step 4: Tests pass; commit.**

---

### Task 3: Fact-only messages (no selected/run_id)

**Files:** `msg/ObservationFrame.msg`、`SceneSnapshot.msg`、`ShapeHypothesis.msg`、`TargetModel.msg`

- [ ] **Step 1: Write a gtest that compiles the messages and asserts they have no `selected`, `completed`, or `run_id` fields (string-scan generated headers or static_assert via SFINAE).**
- [ ] **Step 2: Fail on missing types.**
- [ ] **Step 3: Add fact-only msgs: pose + 6-value covariance, label id, quality enum. No harvest plan fields.**
- [ ] **Step 4: Pass and commit.**

---

### Task 4: Four actions + two services

**Files:**
- `action/RunHarvest.action`（goal: request_id/scene_key/profile_id/intent/selection_mode）
- `action/SurveyScene.action`
- `action/BuildTargetModel.action`
- `action/ExecuteTarget.action`
- `srv/BeginScene.srv`
- `srv/ControlTask.srv`（idempotent, `expected_state_seq`）

- [ ] **Step 1: gtest includes generated action/srv headers; assert ControlTask has `expected_state_seq`.**
- [ ] **Step 2: Fail missing.**
- [ ] **Step 3: Add IDL; CMake lists all files in rosidl_generate_interfaces.**
- [ ] **Step 4: Pass and commit.**

---

### Task 5: Immutable profile schema + hash

**Files:** `schema/profile.schema.json`、`profiles/analysis_only_v1.yaml`、`test/test_profile_schema.py`

- [ ] **Step 1: pytest: load YAML, validate against JSON Schema, SHA256 of canonical JSON is stable.**
- [ ] **Step 2: Fail missing files.**
- [ ] **Step 3: Add schema (permissions, velocity scales, tool profile ids, no live overrides).**
- [ ] **Step 4: Pass and commit.**

Use `aubo_py3.12/bin/python`. Zero ROS import in the test module.

---

### Task 6: Canonical JSONL event schema

**Files:** `schema/event.schema.json`、`test/test_event_schema.py`

Required fields: `event_id`, `ts`, `job_id`, `scene_epoch`, `state_seq`, `code`, `payload`.

- [ ] **Step 1: pytest rejects events missing `state_seq` or `scene_epoch`.**
- [ ] **Step 2: Fail missing schema.**
- [ ] **Step 3: Add schema + sample valid/invalid fixtures.**
- [ ] **Step 4: Pass and commit.**

---

### Task 7: Golden MCAP ObservationFrame fixture

**Files:** `test/test_golden_mcap.py`、`test/golden/observation_frame_v1.mcap`

- [ ] **Step 1: pytest opens golden MCAP, reads one ObservationFrame-equivalent record, asserts label mask encoding is mono16 and no per-target mask copies.**
- [ ] **Step 2: Fail missing file.**
- [ ] **Step 3: Generate golden with rosbag2_py or mcap writer; check in the binary.**
- [ ] **Step 4: Pass and commit.**

---

### Task 8: G0 gate

- [ ] **Step 1: `colcon test --packages-select peach_interfaces` 全绿，skip=0。**
- [ ] **Step 2: Confirm no dependency on `peach_pose_msgs` / `peach_harvest_msgs` in package.xml.**
- [ ] **Step 3: Commit any CMake/test wiring leftovers.**

**G0 exit:** IDL 可构建、常量钉死、profile/event schema 有单测、golden MCAP 可回放。不实现感知/重建/技能节点。
