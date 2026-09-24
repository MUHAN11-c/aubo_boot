# 双末端感知→抓取验证战役（2026-09-22）

**口径（用户裁定）**：≥5 轮「真实相机感知 → supervisor 选果 → peach_arm 完整抓取 FULL 干跑」（`skip_reconstruction:=true`，用感知原结果；重建仅后续精化件）。每轮同步录制全量 bag（observability 会话 bag，100G 预算）+ rviz2 窗口视频。全程 mock 控制：不动真机、不 SetIO、`tool.enabled` 恒 false。

方案依据：[reports/2026-09-22-real-cam-mock-dual-tool-validation/report.md](../../reports/2026-09-22-real-cam-mock-dual-tool-validation/report.md)（v1.0；本轮差异：B3 门升级为 ≥5 轮 FULL、战役产物集中本目录）。

## 目录约定

| 路径 | 内容 | 入库 |
|------|------|------|
| `README.md` | 本台账：轮次表 + 门结论 + 真机前置清单 | 是 |
| `corpora.yaml` | 案册注册表（E2E 阶段二）：seed/规模/expect 来源单源，解析与实跑驱动对表 | 是 |
| `bags.md` | bag↔轮次↔分析结论↔purge 记录索引（bag 本体在 `runs/session_*/bag`） | 是 |
| `analysis/` | 每轮指标（`<rid>/`：per_round_summary.md/json、stability json 等） | 是 |
| `scripts/` | 战役专用脚本（run_round.sh / per_round_summary.py / stability_metrics.py） | 是 |
| `videos/` | rviz 录屏 mp4（`<rid>_rvizwin.mp4`） | 否（本地留存） |

## 环境准备（一次性）

- 录屏增强（可选，本机缺，装了才能自动提窗）：`sudo apt install wmctrl xdotool`；未装时 `record_rviz_harvest.sh` 降级可用，需手动保持 RViz 在前台不遮挡。
- 每轮启动前/后清进程（MUST）：`pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe|ffmpeg'`。
- **DDS 域隔离**：本战役栈统一走 `CAMPAIGN_DOMAIN`（默认 61，`campaign/scripts/launch_stack.sh`），与并发 agent 会话（曾见 33/46）隔离；preflight 是进程表级互斥，对面栈活着时本战役**等位不抢**（2026-09-22 曾误杀邻会话 RSP 一次，纪律：别人的活进程只报告不动手）。

## 标准启动命令（真相机 + mock 控制）

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=true camera_frontend:=stereo skip_reconstruction:=true \
  tool_profile:=hollow_cylinder_v1 autostart:=false
```

拍照位自洽（感知轮必做）：初始 `harvest_stow` 与拍照位差 0.22rad（wrist2）→ 先发一次 Survey（intent:2）把 mock 关节对到 `global_photo_pose`，之后物理相机/TF/感知三方自洽，再发接触轮。

## 轮次台账

| # | request_id | 类型 | 工具 | 场景 | 门结果 | 产物 |
|---|-----------|------|------|------|--------|------|
| — | sim_20260924_1038/1049/1055 | M1 网格×3 轮 | hollow | mock 无相机 | 最好 19/20（tilt/deep_left 逐轮翻转=边界非确定性；near_horizontal 新词表三轮全 matched；无 hang 无码 0） | jsonl×3 + m1_grid.log |
| — | sim_20260924_1103/1107/1116 | M2/M3/M4 | hollow | mock 无相机 | M2 83%、M3 67%、M4 37%（无码 0/挂起 0/绕行比过；失败全归因三族边界） | m1_m5_report_20260924.md + cross_validation_20260924_rerun.md（互证门✅） |
| — | （待填，逐轮追加） | | | | | |

轮次类型：`e2e_survey_`（只扫）/ `e2e_unrefined_`（PREGRASP）/ `e2e_full_unrefined_`（FULL 干跑）+ 时间戳；`request_id` 不复用。

## 验收门

- **P1 感知稳定**：锁定时延 percipio≤8s / stereo≤2s；3σ 抖动 entry/bottom/neck≤5mm、轴角≤1.5°、袋长≤8mm；10min 零 ID 切换；tf_stale<1%；无>2s 帧缺口；low_quality 恢复≤10s。
- **注入矩阵**：M1 网格期望分类 100% 一致；M2 PREGRASP≥95%；M3 FULL≥90% completion_level≥6、绕行比≤1.70；失败 100% 有 failure_code。
- **互证门（E2E 阶段二，2026-09-24 起）**：`scripts/cross_validate.py` 对照解析先验↔实跑，#3（解析成/实跑败）全部归因关闭、#5（无码失败）为零；案册同源按 `corpora.yaml`。grid 20 例首份产物 `analysis/injection/cross_validation_20260924.md`（✅ 过，三例 #3 已归因）。expect 词表新增 `deny_guardrail`（护栏拒=SLEEVE_PLAN_FAILED 5），M1 分母随 deep_left→skip_cartesian、near_horizontal→deny_guardrail 裁定更新。
- **5 轮完整抓取（核心门）**：`e2e_full_unrefined_` ≥5 轮，每轮 ≥3 目标；SUCCEEDED 且 completion_level≥6 占比 ≥90%；无 300s hang；`harvest.grasped=false`；tool 恒关。
- **P2-IMU（adaptive）**：I1–I7 七步全过，servo status=0。
- **真机前置**：对照 docs/testing.md 真机授权前检查清单；全部门过 + A 级问题清零 + 分支推 Gitee 后才申请授权。

## 单轮 runbook

1. 前清 pgrep → 启动 launch（上命令）→ 冒烟：`ros2 control list_controllers`、`/joint_states`、`tf2_echo world tcp`、lifecycle 全 active。
2. `campaign/scripts/run_round.sh <rid>`（或手工）：`record_rviz_harvest.sh <rid> <dur>` 起 rviz 录屏 → `ros2 action send_goal /peach_supervisor/run_harvest …` 发意图。
3. 轮毕 Ctrl+C 停栈 → pgrep 复核无残留。
4. `python3 scripts/collect_round.py <rid>` → `peach_bag_report runs/session_*/bag` → `campaign/scripts/per_round_summary.py <rid>` → 结果入 `analysis/<rid>/`，mp4 拷入 `videos/`。
5. 更新本台账与 `bags.md` → `python3 scripts/purge_analyzed_bags.py runs/session_<该轮>` 回收 bag。
