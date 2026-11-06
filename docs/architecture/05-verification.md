# 验证与证据

> 本页保留 2026-10-27 的历史轮次、命令和证据；没有在本次文档整理中重跑这些硬件或运行验收。当前方法及判定见包级 testing.md。

## 已执行

### 构建/测试

```bash
colcon build --packages-select peach_common peach_vision_core peach_scene_perception \
  peach_target_reconstruction peach_supervisor peach_scene_obstacles \
  peach_arm peach_bringup peach_system_tests peach_harvester
colcon test --packages-select peach_common peach_vision_core peach_scene_perception \
  peach_target_reconstruction peach_supervisor peach_scene_obstacles \
  peach_bringup peach_arm
```

结果：10 包构建成功；新包官方 lint/测试通过；`peach_bringup`/`peach_arm` lint 通过；接口清单通过。

### 真实相机 + RViz

```bash
run_live_camera_mock.py --models 0 --rounds 0 --soak-seconds 20 --domain 101 --rviz
```

- camera/depth/joints fresh，`rviz_alive=true`；
- scene/reconstruction/supervisor/arm Active；
- scene 干净退出；
- `residual_pids=[]`；
- runner `passed=false` 仅因 `move_group` 退出 `-11`。

### 感知基准

`benchmark_perception.py`：13.84 FPS，p50 72.83 ms，5 检测/4 候选。

### 力仿真

- `force_sim_node` + static `base_link→tcp`；
- `/force/sample --once` 返回完整 5 通道帧；
- `serial_force` lint 通过。

## 证据路径

- `runs/peach_refactor_20261027/phase1.md`
- `runs/peach_refactor_20261027/live_camera_rviz2/run/report.json`
- `runs/peach_refactor_20261027/live_camera_rviz2/run/stack.log`

## 本轮补充回归（2026-10-27，不替代 10×30×1800）

- 14 包构建；10 包官方 lint/test 全通过。
- 接口清单 59 active + 5 reserved。
- 校准身份单轮：`model_count=1`、`target_1`、`success=true`、`pregrasp_completed=true`。
- 真实相机 15 s soak：4 目标、camera/depth/joints fresh。
- 感知基准：13.74 FPS，p50 71.87 ms，p95 78.90 ms。
- 唯一失败：外部 `move_group` shutdown `-11`；`residual_pids=[]`。自动 10×30×1800 仍按 `testing.md` 记录未通过。
