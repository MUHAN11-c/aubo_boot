# Reconstruction Dispatch and Cycle-Time Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ensure reconstruction is bound before any observation motion, remove the reconstruction lock inversion, and complete a quality-gated no-SetIO grasp cycle in 45–60 seconds when scene geometry permits.

**Architecture:** Keep reconstruction as the geometry owner and the task executor as the sole batch dispatcher. Shorten the reconstruction critical section, add an explicit Build-start feedback handshake before `OBSERVE_ONLY`, then tune only the number and budget of observation moves while retaining four-view and geometry-quality gates.

**Tech Stack:** ROS 2 Jazzy, rclpy actions/Lifecycle, Python 3.12, MoveIt 2/MTC, `generate_parameter_library`, YAML.

**Spec:** `docs/superpowers/specs/2026-08-21-reconstruction-dispatch-cycle-design.md`

## Global Constraints

- Keep reconstruction minimum views at 4 and retain baseline, depth coverage, refined RMSE, and inlier-ratio gates.
- Keep free-space velocity/acceleration scaling at 0.10 and contact scaling at 0.05.
- Keep `tool.enabled=false`; do not call SetIO.
- Do not modify the read-only real-driver stack or add business tests under package `test/` directories.
- Joint order remains `shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`.
- Real motion starts only after process, robot-status, Lifecycle, camera, TF, and parameter preflight checks pass.

---

### Task 1: Release the Reconstruction State Lock Before Auto Drive

**Files:**
- Modify: `src/peach_target_reconstruction/peach_target_reconstruction/reconstruction_node.py:532-610`

**Interfaces:**
- Consumes: `_state_lock`, `_target_masks`, `_auto_drive()`, `PeachTargetObservationArray`.
- Produces: `_on_target_observations(msg) -> None` with cache mutation under lock and auto-drive after lock release.

- [ ] **Step 1: Record the failing structural check**

Run a one-off AST/source check that locates `_auto_drive()` inside the lexical `with self._state_lock` block in `_on_target_observations`.

```bash
python3 -c "import ast,pathlib; p=pathlib.Path('src/peach_target_reconstruction/peach_target_reconstruction/reconstruction_node.py'); t=ast.parse(p.read_text()); f=next(n for n in ast.walk(t) if isinstance(n,ast.FunctionDef) and n.name=='_on_target_observations'); print(any(isinstance(x,ast.Call) and getattr(x.func,'attr','')=='_auto_drive' for w in f.body if isinstance(w,ast.With) for x in ast.walk(w)))"
```

Expected before change: `True`.

- [ ] **Step 2: Move auto drive outside the lock**

Replace early returns in the locked cache section with a local `drive_auto` flag. Preserve all cache semantics, then call:

```python
        if drive_auto:
            self._auto_drive()
```

after leaving `with self._state_lock:`. Do not move shared-state writes outside the lock.

- [ ] **Step 3: Verify the lock boundary**

Run the structural check again and require `False`, then run:

```bash
aubo_py3.12/bin/python -m py_compile \
  src/peach_target_reconstruction/peach_target_reconstruction/reconstruction_node.py
```

Expected: AST check `False`; compilation exits 0.

### Task 2: Gate Observation Motion on Build Start Feedback

**Files:**
- Modify: `src/peach_task_executor/peach_task_executor/executor_node.py:78-82,432-475,760-825`
- Modify: `src/peach_task_executor/config/executor.yaml`
- Modify: `src/peach_task_executor/peach_task_executor/executor_parameters.yaml`
- Regenerate: `src/peach_task_executor/peach_task_executor/executor_parameters.py`

**Interfaces:**
- Consumes: `_build_feedback`, `BuildTargetModel` handle, feedback states `COLLECTING`/`READY`, cancellation flags.
- Produces: `_wait_build_started(handle, timeout_s) -> bool` and parameter `build_start_timeout_s: double`.

- [ ] **Step 1: Record the failing dispatch-order check**

Run a source-order probe and verify `_cmd_dispatch` currently calls `_send_action(...OBSERVE_ONLY...)` without `_wait_build_started` appearing first.

```bash
python3 -c "import inspect,sys; sys.path.insert(0,'src/peach_task_executor'); from peach_task_executor.executor_node import PeachTaskExecutor; s=inspect.getsource(PeachTaskExecutor._cmd_dispatch); print('_wait_build_started' in s and s.index('_wait_build_started') < s.index('_send_action'))"
```

Expected before change: `False`.

- [ ] **Step 2: Add the generated parameter contract**

Add this entry to `executor_parameters.yaml` and `2.0` to `config/executor.yaml`:

```yaml
  build_start_timeout_s: {
    type: double,
    default_value: 2.0,
    description: "Build 目标绑定并进入 COLLECTING 的等待上限；超时禁止观察运动。",
    validation: { gt<>: [0.0] }
  }
```

Regenerate `executor_parameters.py` using the package's existing parameter-generation build path; do not hand-edit generated validation logic unless the repository generator requires it.

- [ ] **Step 3: Implement the handshake**

Add a bounded wait that succeeds on `COLLECTING` or `READY`, returns false if the Build result finishes, cancellation/skip is requested, or the deadline expires:

```python
    def _wait_build_started(self, handle, timeout_s: float) -> bool:
        result_fut = handle.get_result_async()
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while time.monotonic() < deadline:
            if str(self._build_feedback.get('status') or '') in (
                    'COLLECTING', 'READY'):
                return True
            if result_fut.done() or self._cancel or self._peek_skip():
                return False
            time.sleep(0.02)
        return False
```

Call it immediately after storing `build_handle` and before constructing/sending the observe goal. On failure, cancel Build, record `failure_code='build_start_timeout'`, apply `Event.BUILD_FAILED`, and return without motion.

- [ ] **Step 4: Verify the order and syntax**

Re-run the source-order probe and require `True`. Compile `executor_node.py` and validate both YAML files with `yaml.safe_load` under `aubo_py3.12`.

### Task 3: Bound Observation Moves Without Lowering Geometry Quality

**Files:**
- Modify: `src/peach_manipulation_skills/config/approach_grasp.yaml:55-105`
- Modify: `src/peach_manipulation_skills/src/approach_grasp_node_parameters.yaml:180-280`
- Modify: `docs/field_test.md`

**Interfaces:**
- Consumes: initial photo-pose frame plus active-view feedback.
- Produces: `scan.min_effective_views=3`, `scan.maximum_moves=4`, `scan.time_budget_s=35.0`; quality minimum remains 4.

- [ ] **Step 1: Change only pacing parameters**

Set the three scan parameters to `3`, `4`, and `35.0` in runtime config and parameter defaults. Keep `quality.minimum_views=4`, baseline/depth/refit thresholds, and all velocity scaling unchanged.

- [ ] **Step 2: Add field acceptance criteria**

Document Build-start latency `<2 s`, normal observation budget `<=35 s`, single-target target `45–60 s`, four-view minimum, and explicit no-SetIO requirement in `docs/field_test.md`.

- [ ] **Step 3: Validate parameter parity**

Use `yaml.safe_load` to assert runtime and schema defaults agree, and assert quality minimum remains 4 and `tool.enabled` remains false.

### Task 4: Build and Offline Verification

**Files:**
- Verify all files from Tasks 1–3.

**Interfaces:**
- Consumes: modified source and generated parameters.
- Produces: installed Release artifacts ready for real hardware.

- [ ] **Step 1: Run repository checks**

Run `git diff --check`, Python compilation, interface-manifest validation, YAML parity checks, and confirm no `.orig`/`.rej` files.

- [ ] **Step 2: Build the full workspace**

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected: all packages finish; warnings are recorded separately from failures.

- [ ] **Step 3: Verify installed values**

Source `install/setup.bash` and assert Build startup timeout is 2.0, scan values are 3/4/35, quality minimum is 4, and execution/grasp/tool defaults remain false.

### Task 5: Full Real-Robot Acceptance Run

**Files:**
- Create runtime evidence under existing `web_runs/` and `harvest_runs/` only; do not delete prior data.
- Update: `web_runs/field_test_20260821/log.md` with observed evidence.

**Interfaces:**
- Consumes: real AUBO controller `169.254.10.98`, Percipio `169.254.10.110`, active hand-eye YAML.
- Produces: one completed no-SetIO target cycle and precision/trajectory/safety report.

- [ ] **Step 1: Preflight and launch**

Check residual processes, ping both devices, confirm robot status (`e_stopped=0`, `motion_possible=1`, `in_error=0`), active controllers/Lifecycle, camera rate, and wrist-camera TF. Launch real hardware with camera; do not auto-run.

- [ ] **Step 2: Enable the authorized profile**

Set executor execution, skill execution, and grasp to true; read back `tool.enabled=false`. Capture a fresh robot-status sample before sending the goal.

- [ ] **Step 3: Run one complete PICK_ALL batch with monitoring**

Send a unique request ID. Require Build `COLLECTING` before the first orbit motion. Monitor each trajectory result, effective/reconstruction views, refit metrics, MTC approach/insert, skipped tool IO, retreat, recovery status, and action termination.

- [ ] **Step 4: Compare precision and timing**

From JSONL/ledger, compute initial-to-refined entry and neck displacement, axis angular difference, travel difference, RMSE, inlier ratio, Build-start delay, observation time, contact time, and total cycle time. Do not claim direction/location accuracy if refined output is absent.

- [ ] **Step 5: Stop safely and report**

Ctrl+C the launch, require `RobotMoveStop`/clean hardware deactivation, verify no ROS residuals, and write the field log. Classify quality, efficiency, scheduling, trajectory, and safety separately as PASS/FAIL with evidence.
