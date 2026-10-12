# peach_system_tests

隔离域 `launch_testing` 与 mock 故障矩阵。不含真机、不自动 `RunHarvest`。

**测试流程指导（本栈怎么跑、命名、验收门、真机纪律）：[doc/testing.md](doc/testing.md)**——2026-10-10 自仓库级 `docs/testing.md` 重写入包（用户裁定）。

## 测什么

- `test_preflight.py`：启动预检零 ROS（argv basename，不误伤编辑器路径）
- `test_mock_launch.py`：isolated launch_testing 起 mock `harvest_system`；使能默认关、无自动 goal；`/joint_states` 关节序；lifecycle Active
- `test_e1_supervisor_chain.py`：E1 批次链（域 90）——注入器 `test/perception_sim.py` 真发 RunHarvest 走完 FSM
- `test_tool_profile_smoke.py`：三末端逐档冒烟（域 91/92/93）——TF 对档案 ±2mm、`tool.profile_id`、imu_follow 接线
- `test_scene_obstacles.py`：场景障碍快照链（域 94）——合成点云+假目标+假 apply 端
- `test_replay_approach.py`：回放塔——零 ROS 纯几何，现场真袋/分层/随机语料对照 `replay_baselines.json` 冻结基线（改接近/融合/护栏/包络必跑）
- `test_perf_baseline.py` / `test_baseline_inventory.py`：`perf_baseline.json` / `baseline_inventory.json` schema 守卫

## 系统测驱动脚本（scripts/，对已起栈的人工复核）

2026-09-29 起测试相关统一收进本包（不再散落 /tmp）。均不代授权、不建
假发布器，`ROS_DOMAIN_ID` 由环境决定：

- `drive_harvest.py`：操作台 `SetEnables` + `RunHarvest` goal 发送器
  （`--intent 2` SURVEY_ONLY / `--intent 0 --grasp` 预抓取接近链）
- `probe_topics.py`：相机流帧率探针（**RELIABLE 订阅**——发布端
  RELIABLE 时 BEST_EFFORT 会假性 0 帧，09-29 勘定）
- `check_planning_scene.py`：`GetPlanningScene` 巡检——快照 box 数 +
  ACM 豁免条目（「起点碰撞但本该豁免」类问题用它判）
- `gen_baseline_inventory.py`：生成重构基线快照 `test/baseline_inventory.json`
  （参数/接口/函数索引，只锁 schema 不锁数值）
- 冻结相机一次感知→干跑 FULL（不进默认 `colcon test`，需 bag + GPU）：
  `frozen_camera_replay.py`、`launch/frozen_camera_grasp.launch.py`、
  `config/frozen_one_view.yaml`（`min_views=1`、基线门 0°、
  `reconstruct_in_trajectory`、单视预算 `d_inner=0.25`）、`drive_frozen_grasp.py`、`run_frozen_grasp.py`
  （域 81，抓取时钟从 goal 受理起 180 s，出生在 `global_photo_pose`，
  离开拍照位后 `~/pause` 停发图像和点云，`skip_reconstruction:=false`，
  `grasp.allow_stale_anchor`，
  `moveit.obstacle_guard_enabled=false` 以免冻结点云 box 拦套入，
  第一颗终局后 action cancel，`tool=false` 跳过 SetIO）。默认 bag
  `runs/peach_camera_capture_20260930/stereo_10s`。成功须 `succeeded≥1`。
  `test_frozen_camera_bag.py` 核 overlay + bag 帧，缺 bag 则 skip 帧测。

## 构建 / 测试

```bash
colcon test --packages-select peach_system_tests
# 冻结相机一次感知 + 套入干跑（mock，域 81；需 bag 与 GPU；墙钟 180 s）：
ros2 run peach_system_tests run_frozen_grasp.py
```

`colcon test` 绿 ≠ 套袋验收。BSD-3-Clause。
