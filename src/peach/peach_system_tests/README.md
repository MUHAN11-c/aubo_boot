# peach_system_tests

手工复核脚本和官方 ament lint，不进入产品 launch。冻结/联合 runner 面向 mock，通用 `drive_harvest.py` 没有硬件模式核验，仍须遵守操作员授权。launch 本身默认不发 goal；驱动脚本按夹具显式发送 RunHarvest。`colcon test` 只跑本包 lint，行为结果看日志、账本和记录。

**测试流程指导（本栈怎么跑、命名、验收门、真机纪律）：[doc/testing.md](doc/testing.md)**

## 系统测驱动脚本（scripts/，对已起栈的人工复核）

正式测试入口在本包；/tmp 中的试验脚本不作为交付入口。各脚本不代授权；
已有栈工具沿用环境中的 `ROS_DOMAIN_ID`，联合 runner 默认域 86，冻结 wrapper 默认域 81：

- `drive_harvest.py`：操作台 `SetEnables` + `RunHarvest` goal 发送器
  （`--intent 2` SURVEY_ONLY / `--intent 0 --grasp` 预抓取接近链）
- `benchmark_perception.py`：离线 RGB-D `.npz` 感知基准；最终周期统计用
  Ultralytics `Results.speed`（preprocess/inference/postprocess）
- `probe_topics.py`：相机流帧率探针（默认 **RELIABLE 订阅**，
  `--no-reliable` 可做 BEST_EFFORT 对照；全零仍退出 0，须查打印计数）
- `check_planning_scene.py`：`GetPlanningScene` 巡检——快照 box 数 +
  ACM 豁免条目（「起点碰撞但本该豁免」类问题用它判）
- `gen_baseline_inventory.py`：导出当前参数/接口/函数索引，默认写到包内 `output/baseline_inventory.json`，不做历史回归判定
- `analyze_harvest_record.py <run>`：离线生成 PNG、CSV 与 `summary.json`，默认写到 `<run>/analysis`
- `run_live_camera_mock.py`：真实相机、mock 臂与 RViz 联合入口，见下方说明
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
  缺 bag 时不要把这一轮当成通过。

冻结驱动只取消自身 request；遇到恢复等待保留 `recovery_required`，不自动 ACK。
它在第一颗终局后主动取消批次，退出 0 仅表示本夹具的单果成功条件、暂停回放与墙钟门通过；
不要求 RunHarvest 自然 SUCCEEDED，也不证明 PICK_ALL 全部完成或 L7 真实剪切。
批次 action status、result.success 与单果 summary 必须分别保留；自有 mock 栈由 wrapper 收尾。

## 构建 / 测试

```bash
colcon test --packages-select peach_system_tests
# 冻结相机一次感知 + 套入干跑（mock，域 81；需 bag 与 GPU；墙钟 180 s）：
ros2 run peach_system_tests run_frozen_grasp.py
```

`colcon test` 绿 ≠ 套袋验收。BSD-3-Clause。

## 当前维护入口

职责、源码地图、生命周期/并发与失败处理见 [包架构](doc/architecture.md)，各工具的请求、产物和退出码边界见 [输入输出](doc/io.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。

真实相机+mock+RViz联合入口为 `scripts/run_live_camera_mock.py`。`config/live_camera_mock.yaml` 只保留固定真实相机TF、mock关节初值等环境事实；显示使用 `peach_bringup/config/harvest_app.rviz`。runner不生成速度/超时/质量门overlay，使用产品默认或 `--task-params-file` 指定的同一标准ROS作业配置；该文件也可原样用于real launch的 `task_params_file`。默认PREGRASP_ONLY按实际读回要求level2，FULL不切刀要求level6；不能把预抓取通过称作完整采摘。运行前显式设置测试使能，任何模型/抓取失败或人工恢复等待立即停止后续，不自动ACK。`--pick-all` 要求该轮全部账本目标达到作业完成档，不能只成功一颗就算整轮成功。清理自有进程组并把记录收进 `output/records/`；历史高速/跳重建结果不证明当前产品参数通过。

冻结 runner 默认无头，显式 `--rviz` 才开桌面；只清自有 PGID，`--skip-launch` 保留外部栈。旧 README 记载过真实联合短测 1/1 达到 level6、RViz 存活、cleanup=[]；本段未附轮次路径，保留为历史陈述，不作为当前源码已通过的证据。正式 10轮/30模型/1800秒仍须按 testing.md 的逐轮记录和验收门判定。
