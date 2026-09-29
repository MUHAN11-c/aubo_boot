# peach_system_tests

隔离域 `launch_testing` 与 mock 故障矩阵。不含真机、不自动 `RunHarvest`。

## 测什么

- `test_preflight.py`：启动预检零 ROS（argv basename，不误伤编辑器路径）
- `test_mock_launch.py`：isolated launch_testing 起 mock `harvest_system`；使能默认关、无自动 goal；`/joint_states` 关节序；lifecycle Active
- `test_replay_approach.py`：回放塔——零 ROS 纯几何，现场真袋/分层/随机语料对照 `replay_baselines.json` 冻结基线（改接近/融合/护栏/包络必跑）

## 系统测驱动脚本（scripts/，对已起栈的人工复核）

2026-09-29 起测试相关统一收进本包（不再散落 /tmp）。均不代授权、不建
假发布器，`ROS_DOMAIN_ID` 由环境决定：

- `drive_harvest.py`：操作台 `SetEnables` + `RunHarvest` goal 发送器
  （`--intent 2` SURVEY_ONLY / `--intent 0 --grasp` 预抓取接近链）
- `probe_topics.py`：相机流帧率探针（**RELIABLE 订阅**——发布端
  RELIABLE 时 BEST_EFFORT 会假性 0 帧，09-29 勘定）
- `check_planning_scene.py`：`GetPlanningScene` 巡检——快照 box 数 +
  ACM 豁免条目（「起点碰撞但本该豁免」类问题用它判）

## 构建 / 测试

```bash
colcon test --packages-select peach_system_tests
```

`colcon test` 绿 ≠ 套袋验收。BSD-3-Clause。
