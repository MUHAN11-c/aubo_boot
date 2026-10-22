# 调试台、测试证据与 5 点力

本页补 SKILL 之外最常被问的三件事：**怎么调参/看数**、**mock 联合测到底证明了什么**、**力传感现在能记录什么**。运行事实仍以 `peach_observability/doc/io.md`、`peach_system_tests/doc/testing.md`、`serial_force/README.md` 为准。

## 调试台 `/debug.html`

由 `peach_observability` 在 8090 托管，静态资源来自顶层 `peach_web`（`debug.html/js/css` 已进 HTTP 白名单）。只是调试客户端，不是第二控制面；不新增 SetIO，不绕过调度使能或臂侧 `ExecutionAuthority`。

| 页签 | 内容 |
|------|------|
| 测试台 | 4 个 runner 预设（3×1 首颗 / 3×1 / 10×4 PICK_ALL / 10×1），生成并复制命令；读最新一跑显示逐轮与逐颗 |
| 参数调节 | `PARAM_GROUPS` 白名单读写；运动类/静态参数标注；测试快档/稳定档 |
| 实时状态 | 目标 ID/优先级/2D/3D/掩膜、流新鲜度、使能、自检 |
| 逐轮分析 | 按 batch 展示 outcomes/fruits，掩膜/检测调试图 |
| 审计/结果 | 本次会话调试结果与参数写入结果 |

页面不在浏览器里“点一下就开始跑”：runner 命令由终端执行，调试台只负责生成/复制和回读。

### `/api/params`

- `GET /api/params`：按白名单节点返回 `groups`，每项含 `value/present/node_running/error`；节点未运行或参数未声明也返回，不报 500。
- `POST /api/params`：body `{"changes":[{"node","name","value"}]}`，只接受 `param_panel.PARAM_GROUPS`；按类型和上下界校验。非法项逐条拒绝：全部失败 400、部分成功 207、全部成功 200。
- 白名单节点：`peach_supervisor`、`peach_arm`、`peach_scene_perception_node`、`peach_target_reconstruction_node`、`peach_observability`。
- 观测节点自身参数走本地 `set_parameters`；其余走 `/<node>/{list,get,set}_parameters`（`rcl_interfaces`），不是自定义控制通道。
- `debug.enabled=false` → 503；运动类（臂速度/加速度/行程/位姿门）在 `debug.motion_enabled=false` → 423。静态参数（`skip_reconstruction`、`capture.auto_mode`、`record.enabled/level`）标“重启”，写后不会热生效。
- 每次写入（含被拒）落 `runs/debug_audit/<YYYYMMDD>.jsonl`；读取不审计。
- 观测节点写后同步 `_params` 缓存（`_on_param_change`）；否则快照和 `debug_command` 仍读旧值。

参数写入只改 ROS 参数，不等于重新授权。真机速度上限仍是 0.1；网页把 `velocity_scaling` 改到 1.0 也不会让臂越过命令门。

## 真实相机 + mock runner

入口：`ros2 run peach_system_tests run_live_camera_mock.py`。**mock-only**：`wait_state` 会拒绝 `hardware_mode != 'mock'`，不能拿它跑真机。

```bash
ros2 run peach_system_tests run_live_camera_mock.py \
  --rounds 10 --models 30 --soak-seconds 1800 --domain 86 \
  --output runs/live_mock_YYYYMMDD_HHMMSS
```

常用开关：

| 开关 | 作用 |
|------|------|
| `--rounds N` / `--models N` / `--soak-seconds S` | 抓取轮 / 模型样本 / 可视化稳定性预算 |
| `--rviz` / `--no-rviz` | 默认 RViz；`--no-rviz` 只用于无头排障，不算可视化验收 |
| `--first-target` | 每轮都抓 `priority=1`，用于同一位姿重复验证 |
| `--pick-all` | 每轮发 `PICK_ALL/AUTO` 多目标，复现完整 Build 轮 |
| `--skip-reconstruction` / `--no-skip-reconstruction` | A/B：跳过 Build/精化 vs 全重建 |

`config/live_camera_mock.yaml` 的 `skip_reconstruction: true` 同时打开调度跳过和臂 `quality.allow_unrefined_geometry`；`--models > 0` 时模型统计仍单独 Build，不受抓取轮 skip 影响。固定采集位校准是 test-only，不写产品手眼文件；相机移动后必须重测。速度 overlay 是 mock 专用（接触 0.4、转移 0.8），产品/真机 `peach_arm.yaml` 仍 0.1。

### 选目标与 ID

runner 2026-10-22 起按场景 `priority`（1..N）选，不再按 `target_id` 排序后过滤；`LOST` 的 memory 候选也参与，是否执行由下游门决定。每轮记录：

- `grasp_slot`：抓取顺序 1..N（`priority` 位置）；
- `target_id`：**场景源 ID，也是 2D/3D/账本 ID**（不再另造 `grasp_id` 重映射）。

所以“4-slot 4/4”指 4 颗不同物理目标，不是同一颗重复抓 4 次。`target_id` 仍是感知注册顺序；`priority` 是锁定集的抓取顺序，两者不一定相等。2D 调试图在确认后显示 `target_N #order`，3D marker 只跟当前检测走。若要发布 ID 本身等于抓取顺序（`target_<priority-1>`），还需锁定时重映射和反查服务，当前未实现。

### 产物与证据口径

- 输出目录一跑一个：`runs/<run>/records/{session_*,harvest_*,round_*}`；`report.json` 记 `consolidated_records`。分析页只列顶层 run，不再把同一次 runner 的 session/round 分成多个选项。
- `report.json` 原子更新，含 `phase`/`updated_at`；中断保留最后完整状态。
- `passed=true` 需要抓取轮账本达到 level 6、相机/关节新鲜证据成立、soak 实际采样达预算且样本健康、清理无残留。
- `passed=false` 不等于抓取失败：4-slot 第二次 4/4 账本 level6，但 slot 4 的相机新鲜门 false，`full_level6_rounds=3`、`passed=false`。先看 `round` 账本终态，再看 camera 证据。
- 2026-10-22 10×4 PICK_ALL：40 行中 37 行 level6（92.5%）；失败为 rounds 1/2 `target_3` `SKIPPED_UNREACHABLE`（round 1 MTC 弦偏离 0.17182>0.12 m；round 2 工具筒体-果实胶囊间隙 -0.0106 m）和 round 9 `target_1` `SKIPPED_QUALITY`（`failure_code=4`，预抓取残差门）。相机门在 rounds 3/4 false，属于测试采样，不是抓取失败；因此 `full_level6_rounds=8/10`、整跑 `passed=false`。
- A/B（2026-10-21）：A 全重建目标级 2/6（33.3%），失败为固定相机多视 refit；B skip 8/8。结论是固定相机多视不足，不是“全重建不可行”。

## 一键分析

`/analysis.html` 读 `runs/`。`/api/analysis/records` 只列顶层记录；有 `report.json` 的 run 通过 `request_id` 关联 `records/` 下 round 账本，聚合 outcomes/rates/geometry。`motion_completed` 统计 `outcome=0 && completion_level>=6`；`harvest_confirmed` 统计 `completion_level>=7`。所以页面上的“完成率”是干跑/套入完成，不是采摘确认。

## 真机单目标（与 runner 分开）

真机不能用 `run_live_camera_mock.py`。用生产整栈：

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=real robot_ip:=169.254.10.98 \
  camera_enabled:=true force_enabled:=true autostart:=false rviz:=true
```

- 速度/加速度/转移 ≤ 0.1；`autostart=false`；执行/接触/工具默认关，按授权逐档 `SetEnables`。
- 单目标：`RunHarvest` 用 `INTENT_PICK_SELECTED=1` + `target_ids: ["target_N"]`（必须是当前锁定集里的目标）；空 `target_ids` 会退回按 priority 选。
- 完整抓取含套入/撤退/刀具：`execute_pregrasp_only=false`、`tool=true`；成功看 `completion_level=7`/`harvest_confirmed`，不是 action `SUCCEEDED` 或页面 allowed。
- 硬件急停在柜/示教器，不经 ROS；现场隔离、工具状态、目标确认由操作员负责。

## 5 点力

`serial_force` 随 `harvest_system` 起（`force_enabled` 默认 true），不进 lifecycle：

| 话题 | 类型 | 含义 |
|------|------|------|
| `/force/points` | `std_msgs/Float64MultiArray` | 5 点力，SI 牛顿，顺序通道 1/2/3/5/7 |
| `/force/travel` | 同上 | 行程 mm，0.1 mm/LSB；哨兵通道 NaN |
| `/force/ad` | 同上 | 原始 AD，哨兵通道 NaN |
| `/force/status` | `std_msgs/UInt8MultiArray` | 每通道状态（正常/断线悬空/满量程等） |

- 100 Hz，32 字节 v3 帧：`A5 | ch | gL gH | mm | aL aH | ... | sum`；克→牛 `N=g*9.80665/1000`。
- 通道点位：5 上、1 右上、2 右下、3 左下、7 左上（果实侧；臂侧镜像 1↔7、2↔3）。半径约 16.3 mm，膜片在 `tcp z=-72.29 mm`。
- 设备 `/dev/force`（CH340/CH343 udev）；一个口只能开一份，勿与整栈并行起第二个节点。`/diagnostics` 看串口与 100 Hz 帧率。
- `scripts/force_web.py` 是 8099 调试查看器（5 点+CoP），只订话题、不碰串口；**不是记录器**。
- 截至 2026-10-22，仓库里**没有**“变化→归零”力窗记录器或力周期分析脚本；需要时另加入口并落 `runs/<run>/force_window*`，不能把 `force_web` 当证据。`serial_force_sim` 是独立 mock 力闭环仿真，不读写串口。

## 收尾

runner 只清自己创建的进程组；观察性/栈/力节点停止后核对 `pgrep` 与 8090/8099 端口。8099 留给 force_web，runner 不占用、不清理。记录目录不要手工拆散：一跑一个顶层文件夹，分析页和报告都按这个约定关联。
