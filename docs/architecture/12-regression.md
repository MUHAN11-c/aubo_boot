# 全功能回归矩阵

## 目标

把 Web、仿真、RViz、真实相机、重建、调度和记录放在同一张可执行矩阵里；每项独立判定，不用短测成功替代长测。

## 矩阵

| 项 | 命令/入口 | 判定 | 本轮证据 |
|---|---|---|---|
| 构建 | `colcon build --packages-select ...` | 14 包成功 | `/tmp` 构建日志；`phase1.md` |
| 官方 lint/test | `colcon test` 10 包 | 全绿 | `phase1.md` |
| 接口 | `check_interface_manifest.py` | 59 active+5 reserved | 通过 |
| GPL 参数 | `ros2 param describe/set` | 描述/约束/跨字段 | 六个包已完成 |
| Web 页面 | `/`、`/monitor.html`、`/debug.html`、`/analysis.html`、`/force.html` | 200，CSP，数据绑定 | `web_review/` |
| Web 长动作 | `POST /api/debug/run_harvest_action` | 动作中 `/api/state` 仍刷新 | `web_rviz_102246` |
| 力页 | `/api/force`、CAD 背景、五点/CoP | 在线/无数据均可解释 | `force_fullbg2.png` |
| RViz | `harvest_app.rviz` | 机器人/TF/相机/标记/TCP | `rviz_complete.png` |
| 真实相机 | 15 s soak | camera/depth/joints fresh | `scene_gpl_live` |
| 模型 | 1 模型 | Build SUCCEEDED/identity | `full_regression2` model |
| PREGRASP | 单轮 | L2 + recovery | `final_e2e2` |
| FULL L6 | Web + mock + RViz | SUCCEEDED, completion 6 | `web_rviz_102246` |
| 记录 | session bag | `/api/state.record` + `ros2 bag info` | `web_rviz_102246` |
| 清理 | 停栈后 `pgrep` | residual 空 | 各轮 report |

参考来源见 [13-references.md](13-references.md)。

## 文档中的历史基线（testing.md）

| 轮次 | 结果 | 口径 |
|---|---|---|
| `live_mock_20261016_183842`（doc 记录） | 5/5 level6，`passed=true`，residual=[] | 短测，不替代 10×30×1800；当前 runs 目录不在工作区 |
| `live_mock_unified_rviz_20261021_1813` | 3/3 level6，RViz 托管，camera fresh | 行程修正后短测 |
| `AB_A_full_20261021_1829` | 6 次尝试，2 次 level6（33.3%） | 全重建 A 侧 |
| `AB_B_skip_20261021_1832` | 8/8 level6（100%） | skip B 侧；仍是 L6，不是 L7 |
| `goal_skip_4slot_20261022_0908` | 4/4 level6，`passed=true` | 按 priority 4 颗不同目标 |
| `goal_skip_4slot_b_20261022_0915` | 4/4 账本 level6，但 slot4 camera fresh=false，`passed=false` | 抓取成功/证据采样失败须分开 |
| 10×30×1800（文档记录） | 30 模型；10 轮中 6 level6、4 质量跳过；soak 约21 min，RViz 退出 | **未通过**，不能宣称全量验收 |

这些是 `src/peach/peach_system_tests/doc/testing.md` 的记录，不是本次临时轮次；新轮次只作为补充证据。

## 重建开关验证（2026-10-27）

- `reconstruction_enabled:=false`：`/peach_target_reconstruction_node` 不存在；lifecycle 仍 active；`/peach_supervisor skip_reconstruction=true`；`/peach_arm quality.allow_unrefined_geometry=true`；startup facts 记录 `reconstruction_enabled=false`。证据 `param_arch/final_disabled.log`。
- `reconstruction_enabled:=true skip_reconstruction:=true`：重建 Active；`capture.auto_mode=true`（不再被 skip 跨包改写）；supervisor skip=true；arm unrefined=true。证据 `param_arch/recon_skip.log`。
- `task_params_file` 含 `skip_reconstruction`/`allow_unrefined_geometry`：启动期报错并拒绝，未启动任何子进程。证据 `param_arch/conflict_rejected3.log`。
- 注意：`capture.auto_mode` 的“停止并清空积分”仍需有相机帧的行为测试；本次只验证参数值/进程开关。

## 已知未通过/需区分

- 自动 runner `full_regression2`（`--models 1 --rounds 1 --rviz`）出现 **测试目标选择与账本 report target_id 不一致**（report target_1，臂日志 target_2），该轮判失败；不能把它算作产品抓取失败，也不能算测试通过。需要修 runner 的 scene/target 记录后再跑。
- `web_rviz_102246` 是手动 Web 驱动、完整相机+mock+RViz 的成功轮（L6，33.456 s），但它不是自动 runner 的 10×30×1800 长测。
- tool=false 的 L6 不等于 L7 真剪切；mock 无 SetIO。
- `move_group` shutdown `-11` 仍是外部 MoveIt 退出问题。

## 结论口径

- Web/RViz/单轮 FULL：已验证。
- 自动全功能 runner：仍有 target 记录不一致，未通过。
- 30 模型/10 轮/30 分钟长测：文档已记录未通过（6/10 level6、4 质量跳过、soak 约21min、RViz 退出）。
- 当前新 `web_rviz_102246` 是 Web 手动触发的 L6 单轮，不是自动 10×30×1800。
