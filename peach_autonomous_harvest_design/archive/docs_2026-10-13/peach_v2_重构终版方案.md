# Peach v2：室外套袋桃三末端剪切采摘 · 完全重构终版方案

初版日期：2026-09-30；评估与实施基线完善：2026-10-08。性质：本文件是重构设计、决策依据与验收计划，实际存放于 `docs/peach_v2_重构终版方案.md`；不是部署快照，不新增第四份活文档。现行采摘核的成对方案见 [`peach_v1_现行终版方案.md`](peach_v1_现行终版方案.md)。

**阅读顺序：** 先看 §18 实施基线与交付门，再看 §1–§14 专题设计；v1/v2 的详细取舍和本轮证据见补充评审 R0–R18。§18 是本轮修订后的实施约束；与早期候选方案冲突时按 §18 收敛。当前运行事实仍以源码和三份活文档为准。文中“目标/拟/候选/待标定”均不表示已实现或已验收。
依据：六路并行审查（视觉 / 臂与轨迹 / 调度与契约 / 三末端与安全 / 论文调研 / GitHub 选型）+ 主审逐条复核源码。
范围：按用户要求**忽略现行项目结构约束**，完全重构；但以下四条是物理与法规事实，不随重构改变：

1. 急停走柜体/示教器硬件回路，ROS 只做应用护栏。
2. 未经人授权不动真机、不 SetIO。
3. 带刀单元按工业应用隔离作业区（ISO 10218-2:2025），不宣称协作。
4. Jazzy 的 `cv_bridge` 按 numpy 1.x 编译，锁 numpy 1.26.4（否则 ABI 崩）。

---

## 目录

1. [核心需求拆解与成功口径](#1-核心需求拆解与成功口径)
2. [现行系统审查结论（已复核）](#2-现行系统审查结论已复核)
3. [重构总体架构](#3-重构总体架构)
4. [感知：检测、关键点、深度、袋轴与袋颈](#4-感知检测关键点深度袋轴与袋颈)
5. [目标建模与许可预算（重写）](#5-目标建模与许可预算重写)
6. [运动：IK、规划、轨迹、避障](#6-运动ik规划轨迹避障)
7. [三种末端：插件接口与动作序列](#7-三种末端插件接口与动作序列)
8. [任务编排：行为树 + 分阶段运动](#8-任务编排行为树--分阶段运动)
9. [室外环境干扰对策](#9-室外环境干扰对策)
10. [安全架构](#10-安全架构)
11. [接口（IDL）精简版](#11-接口idl精简版)
12. [参数单源与初值表](#12-参数单源与初值表)
13. [测试金字塔与验收门](#13-测试金字塔与验收门)
14. [分期落地计划](#14-分期落地计划)
15. [实施时的模型分工](#15-实施时的模型分工)
16. [开放问题（必须实测才能定）](#16-开放问题必须实测才能定)
17. [参考资料](#17-参考资料)
18. [实施基线、契约与交付验收](#implementation-baseline)

补充评审：[2026-10-08 v1/v2 框架与算法综合评估](#review-20261008)（含实现核对、算法假设、风险、取舍及验收清单）。

---

## 1. 核心需求拆解与成功口径

**一句话需求：** 固定座（将来上底盘）AUBO E5 + 腕载 RGB-D，在室外桃园里对套袋桃完成「找到 → 对准袋轴 → 套入 → 剪断袋颈/果柄 → 撤出 → 放果」，三种末端可换：

| 末端 | 原理（按档案与 CAD 推断） | D_inner | L_insert | L_blade（TCP→刃面） |
|------|---------------------------|---------|----------|----------------------|
| `shear_v1` 连杆剪 | 刀组在法兰 +Y 与相机同侧，连杆对切，张口小 | 0.080 | 0.030 | 0.030 |
| `bite_shear_v1` 咬合剪 | 双刃对夹，220 mm 长喉道导轨把枝引入刀区；**非轴对称** | 0.104 | 0.030 | 0.037 |
| `adaptive_shear_v1` 自适应剪 | Ø136 圆盘 + 拉钩弹簧被动贴果，MPU6050 驱动姿态跟随 | 0.120 | 0.090 | 0.079 |

**分项成功口径（按 Bac 2014 分项法，区分「可达果」与「全部果」）：**

| 分项 | 首期目标（固定座，可达果） | 依据 |
|------|---------------------------|------|
| 检测召回（套袋桃） | ≥ 95% | 套袋检测文献 mAP 90%+ |
| 定位（袋轴 + 袋颈，95%） | 横向 ≤ 8 mm，轴角 ≤ 4°；袋颈轴向 v2.0 融合 ≤ 10 mm、近距重测后 ≤ 5 mm（v2.1 关键点目标 ≤ 3 mm） | 末端容差反推（§5）；v2.0 无关键点模型 |
| 套入成功 | ≥ 85% | — |
| 剪断成功（一次） | ≥ 80%，重试后 ≥ 90% | — |
| 整体采摘成功 | 60–75% | SWEEPER 61%、Harvey 76.5%、HAVS 87.5% 同档 |
| 每果周期 | ≤ 25 s（首期），≤ 15 s（二期） | SWEEPER 24 s、HAVS 13.2 s |
| 损伤（果/枝/袋撕裂） | ≤ 5% | Bac 2014 平均 5% |

**决定成败的前置实测（先于写代码）：** 果园抽样 ≥ 200 袋，量袋径分布（d50/d95）、袋颈高度与直径、扎口到结果枝距离、袋轴与重力夹角分布、风速与摆幅。桃是短果柄，袋通常扎在结果枝上——「剪柄」实际是剪袋颈/扎口/短枝。**这组数据决定三把末端各自的适用率，也决定 §5 预算常数。**

---

## 2. 现行系统审查结论（已复核）

主审对标「已复核」的条目逐行看过源码；其余为子审查结论，标注「待实证」的需要现场日志或真机确认。

### 2.1 P0：导致核心需求根本不可达

| # | 问题 | 证据 | 状态 |
|---|------|------|------|
| P0-1 | **剪切许可恒为负。** 轴向余量 = 捕获半宽 0.008 − 安全余量 0 − (袋颈下限 ≥0.003 + 刃面 0.002 + 机器人 0.002 + 目标运动 0.003) ≤ **−0.002 m**，`cut_ok` 永远 false，TOOL 门永不放行；结构性诊断又没把袋颈下限算进去，原因字段误导为"这颗袋不好" | `tool_budget.py:20-35, 59-66, 85-102`；`refine.py:1081-1089`；`target_reconstruction.yaml:89-94` | 已复核 |
| P0-2 | **唯一能真正下刀的链路是几何校验最少的那条。** `skip_reconstruction` 的未精化链不做轴向校验，TOOL 门只查 `pregrasp_verified` + `tool_enabled` | `cycle.cpp:153-183`；`target_cache.cpp:507` | 子审查 |
| P0-3 | **剪切确认可能读回自己的输出。** 下刀写 `io_fun=3, pin=0`；反馈读 `tool_io_states` 的 pin 0；驱动按地址合并工具 IO 状态、不区分方向 | `motion.cpp:82-104, 329-332`；`aubo_e5_hardware.cpp:1606-1611, 1711-1716` | 已复核前两处，驱动侧**待真机实证** |
| P0-4 | **默认末端（adaptive）的套入不会动臂。** launch 硬写 `motion_enabled: 'false'`；真机 Servo 写 JTC 话题，而 real 只起透传控制器；臂不动却判 STALLED 并置 recovery | `harvest_system.launch.py:292-298`；`moveit_servo.yaml:14`；`bringup.launch.py:90,104` | 已复核 launch |
| P0-5 | **接触令牌新鲜窗与作业链时长错配。** 令牌窗 = `effectiveTargetMaxAgeS()`（3–10 s），`model_stamp` 是派发前心跳；FULL 链实测 30–60 s → CONTACT 阶段大概率 EXPIRED；反而无令牌的快照回退路径不查新鲜度 | `cycle.cpp:355-369, 97-151, 153-186`；`target_reconstruction.yaml:70` | 已复核；EXPIRED 比例**待日志实证** |
| P0-6 | **避障只保护相机。** 臂身和刀对场景障碍全豁免；staging 点离果仅约 8–10 cm；返程倒放在两条接近路径下都失效，出冠由新规划 PTP/OMPL 完成且无果胶囊审查 | `peach_arm.yaml:123`；`acm_policy.hpp:29-32`；`grasp_task.cpp:914-1007, 1423-1471`；`motion.cpp:831-926` | 子审查 |
| P0-7 | **刃面位置被当成 TCP。** URDF `cutting_plane` 与 TCP 零偏移，档案 L_blade 0.030/0.037/0.079 在臂侧回退行程计算里被忽略；adaptive 档刀口会落到颈下约 9 cm（果体内） | `tcp.xacro:26-31`；`motion.cpp:216-232` | 子审查 |
| P0-8 | **重建采帧门结构性死锁。** 邻目标距离门与视角无关（两袋 <15 cm 永不开门，180 s 超时）；漂移门 0.04 m 小于同行注释认定的风摆 5–6 cm | `capture.py:761-797`；`target_reconstruction.yaml:32-35` | 子审查 |

### 2.2 P1：可靠性、正确性、安全纵深

- **感知：** 地标覆盖轴向后角误差仍停在 20° → 单帧几乎总是 REOBSERVE（`pose_pipelines.py:328, 383, 462-469`）；χ² 门因不传协方差退化成 6 cm 欧氏球（`pipeline.py:414-425`，`identity.py:131-134`）；身份锚点取袋底袋颈中点、会被 2D 长度延伸推动 → ID 跳变；TF 精确查询失败退回 latest 仍按 base 系输出。
- **重建：** 观测回调里做 ICP/TSDF，与 `/joint_states` 共用默认互斥组 → 静止门拿过期速度（`target_reconstruction_node.py:391-400, 752-789`）；预算项一半没从 yaml 注入，用的是代码默认值；95% 统计口径混乱（MAD 未 ×1.4826×1.96，误差项又线性相加）。
- **深度前端：** stereo TIME_SYNC 失败只打警告（时间戳变设备时钟，重建 TF 全失败）；时域中值无运动补偿；confidence 无人消费；IR 曝光固定、无 HDR，室外未验证。
- **场景障碍：** 点云/TF/关节角三者时间不对齐；目标胶囊跨批次只增不清；单个飞点即生成 6 cm 方块；超限按索引截断而非按距离。
- **臂：** 倒放轨迹加速度取反（时间反转时加速度不应变号，`trajectory_guard.hpp:75-112`）；ACM 只加不删且整表读改写有竞态；ContactMonitor 定时器与 reset 竞态、无动力学补偿、撤退段不受守护；MoveTo 与作业周期可并发并共享 MGI；关闭使能不停在途运动；允许 5% 短插仍下刀；起点容差 0.15 rad；CheckReachability 与 staging IK 都不查场景碰撞；imu_follow 行程上限 0.09 m < 臂侧 0.2 m；取消不直接停 imu_follow。
- **刀具：** 周期开始不确认刀已张开；剪后闭刀回收纳位、开刀要重新过感知令牌（令牌过期 → 果卡刀里）；`set_io` 服务无授权，web 调试页可直接写 DO。
- **调度：** FSM + reducer + EventHold 三轨并存；RunHarvest 命令循环在 action execute 回调内同步阻塞 180 s 级；`require_managed_stack` 三处默认值不一致；bond 默认关（本机已装 `ros-jazzy-bondpy`，launch 注释「缺 bondpy」已过期）。

### 2.3 现行值得保留的思想（重构后仍用）

- 停走式感知节拍（与主动 RGB-D 室外物理特性匹配）。
- 纯核零 ROS（FSM、几何、预算可单测）。
- 逐目标动态许可（「按这颗袋的误差给不给剪」的思想正确，只是算错了）。
- 场景障碍用 Survey 快照而不是 live octomap（腕载相机 + 停走下更稳）。
- 默认干跑停预抓取、故障后禁止 resume 原轨迹、ACK 才恢复。
- 精确 stamp TF 积分。

### 2.4 过度设计（重构时删）

- 身份七元组 + Clearance 复制 GraspDecision + 臂侧令牌/快照双路授权。
- FSM/reducer/EventHold 三轨。
- 感知单帧 ACCEPT/REOBSERVE/REJECT 裁决（与多视融合裁决重复）。
- `*.impl` dict 注册表、兼容 shim、legacy lifecycle 节点。
- fast/conservative 两套观察实现分在调度与臂两侧。

---

## 3. 重构总体架构

### 3.1 设计原则

1. **一个事实只有一个源。** 工具几何只在工具档案；误差常数只在标定文件；许可只在一处计算、一处复检。
2. **闭环优先于精度。** 最后 10 cm 用近距重测 + 视觉对中，不靠一次看准走到底（SWEEPER、HAVS 实证）。
3. **机械容差吸收剩余误差。** 喇叭口、导轨、被动柔顺，比感知多挤 2 mm 更便宜。
4. **全臂受查，只按阶段局部豁免。** 豁免由 MTC stage 局部修改、自动回滚，不写全局 ACM。
5. **剪切确认必须独立于剪切命令。** 独立传感器 + 第二判据。
6. **每层可替换：** 末端、检测器、规划器走 pluginlib；默认实现直接构造。

### 3.2 包切分（对齐 UR / Nav2 / MoveIt 主流）

```
src/
  peach_description/        # URDF/xacro：臂+快换+三末端（含真实 cutting_plane 偏移、惯量）
  peach_bringup/            # 唯一整栈入口；预检；lifecycle_manager+bond
  peach_moveit_config/      # SRDF、pick_ik、OMPL/Pilz/STOMP、控制器映射
  peach_interfaces/         # 精简 IDL（§11）
  peach_perception/         # C++ 组件容器：前端同步/深度质量/点云；Python 推理进程（TensorRT）
  peach_target_model/       # 目标跟踪 + 多视融合 + 袋轴/袋颈估计 + 许可预算（纯核+节点）
  peach_scene/              # 场景硬障碍快照 → PlanningScene（全臂受查）
  peach_end_effector/       # pluginlib：EndEffector 基类 + 三个插件 + 刀具 IO 驱动封装
  peach_manipulation/       # C++：MTC 任务工厂、命令门、执行守护、接触监测
  peach_task/               # BT.CPP v4 行为树 + BehaviorTree.ROS2 节点插件 + 账本
  peach_observability/      # /diagnostics 聚合、会话 bag、8090 只读面板
  peach_calibration/        # 手眼、TCP、刃面、IO 反证测试工具
  peach_system_tests/       # launch_testing + Gazebo Harmonic 系统测
```

依赖单向：`interfaces ← 所有能力包`；能力包之间不互相 import 业务代码；驱动包（`aubo_e5_hardware` 等）保持厂商边界。

### 3.3 进程与数据流

```mermaid
flowchart LR
  subgraph cam [相机前端]
    Stereo[立体深度+置信度]
  end
  subgraph perc [peach_perception]
    Sync[同步+深度质量门]
    Infer[YOLO检测+MobileSAM TensorRT]
  end
  subgraph model [peach_target_model]
    Track[3D 多目标跟踪 KF]
    Fuse[多视 TSDF+袋轴/袋颈估计]
    Budget[许可预算 RSS]
  end
  subgraph scene [peach_scene]
    Snap[硬障碍快照]
  end
  subgraph arm [peach_manipulation]
    Gate[命令门]
    MTC[MTC 任务工厂]
    EE[EndEffector 插件]
  end
  Task[peach_task 行为树]
  Stereo --> Sync --> Infer --> Track --> Fuse --> Budget
  Stereo --> Snap
  Budget -->|TargetModel+Decision| Task
  Task -->|HarvestTarget action| Gate --> MTC --> EE
  Snap -->|PlanningScene| MTC
  Track -->|Observations| Task
```

软件架构见 [images/peach2_sw.png](images/peach2_sw.png)（本仓 L0–L3：传感 → 事实 → 编排 → 命令门 → 臂；`observability` 只读横切，不进系统 launch）。本节 mermaid 为跨包话题/动作数据流，两图互补。

**进程划分：**

| 进程 | 内容 | 语言 | 理由 |
|------|------|------|------|
| camera | 立体前端 | C++ | 高带宽 |
| perception_container | 同步、深度质量、点云裁剪（composable，intra-process） | C++ | 零拷贝 |
| perception_infer | v2.0：现有 YOLO 检测 + MobileSAM；v2.1：YOLO11-seg + 关键点（TensorRT FP16） | Python | 模型生态；进程隔离防 CUDA 崩溃拖死全栈 |
| target_model | 跟踪 + 融合 + 预算 | Python（Open3D）/ C++ 热点 | 算法迭代快 |
| scene | 硬障碍快照 | Python | 低频 |
| manipulation | move_group 客户端 + MTC + 命令门 + 末端插件 | C++ | 实时性、MoveIt API |
| task | BT 执行器 + 账本 | C++ | BT.CPP 原生 |
| observability | 诊断聚合、bag、8091 只读面板 | Python | 只读 |

生命周期：perception_container / target_model / scene / manipulation / task 全部 LifecycleNode，由 `nav2_lifecycle_manager` 托管，**bond 默认开**（`bond_timeout: 4.0`）。

实现现状（2026-10-08 源码核对，R18·S7）：当前 `peach2_bringup/peach2_system.launch.py` 起的是 perception **单** LifecycleNode（未拆 container/infer），observability 不随系统 launch 启动（包内自带 launch）；托管名单与顺序＝perception → target_model → scene → manipulation → task（`camera_enabled=false` 时省略 perception/scene）。上表 container/infer 拆分与 observability 进程为**目标架构**，待 M2 感知热路径落地。

### 3.4 车载硬件与计算架构

**计算拓扑一句话：现场主控 = Jetson Orin NX 16GB 单板承载整机栈（§3.3 全部八个进程）；RK3588 只作压力过大时的前端采集/预处理分流预案（预留，未实装、不作首期依赖）；开发、仿真与测试留在开发工控机（Ubuntu 24.04 + Jazzy + RTX 3090），不随车。** 与 `docs/design.md`（V6.4 硬件口径）同源；性能锚点全部是开发机实测，NX 部署后须专项重标（见下文部署约束）。

#### 3.4.1 硬件栈

上层为数据来源、下层为执行器；层内并排为同层设备；侧盒是旁路。硬件：人机 → Orin NX → 相机 / 柜 / 刀 / IMU **对等**连接（相机不夹在算力与柜之间）；急停回路不经计算栈。软件见图 [peach2_sw.png](images/peach2_sw.png)。型号与约束在 §3.4.2。

![Peach v2 硬件栈](images/peach2_hw.png)

**源文件与维护**：双图统一图源 [images/peach_v2_arch.drawio](images/peach_v2_arch.drawio)（draw.io 双页：页 1 软件栈 / 页 2 硬件栈）。导出命令：`drawio -x -f png -s 2 -b 20 --page-index 1 -o images/peach2_sw.png docs/images/peach_v2_arch.drawio`（硬件页 `--page-index 2` → `peach2_hw.png`）；可在 diagrams.net / VSCode drawio 插件中拖拽精修后按同名覆盖重导出，避免正文引用断链。

#### 3.4.2 硬件清单（型号 / 接口 / 角色）

| 单元 | 型号与关键规格 | 接口 / 链路 | 角色 |
|------|---------------|-------------|------|
| **现场主控** | **Jetson Orin NX 16GB**：8× Cortex-A78AE + Ampere GPU（1024 CUDA / 32 Tensor Core），100 TOPS 量级（INT8 稀疏），16 GB LPDDR5 102.4 GB/s，功耗 10/15/25 W 档 | 双 GbE、USB 3.2、UART、GPIO、M.2 | §3.3 八进程全部车载承载；唯一有权与柜对话的计算单元（经 manipulation） |
| 前端计算（**预留**） | RK3588：8 核（4×A76+4×A55）+ 6 TOPS NPU（RKNN） | GbE → NX；相机侧 GigE/MIPI | 相机采集 / 时间同步 / 深度质量预筛 /（可选）检测前置，为 NX 减负 |
| RGB-D 前端 | Percipio PS800-E1：散斑结构光双目，基线 62.2 mm，L/R 1280×960 mono8 硬件同帧；主机侧 SGBM 13.5–13.7 fps；深度 uint16 × 0.25 mm | GigE（camport4，legacy GigE2.0） | 停走式感知前端；v2 §4.1 改造承载（置信度 / 静止中值 / 曝光） |
| 控制柜 | AUBO E5 柜：六关节透传控制、工具 IO（DO/DI）、`RobotStatus`（抱闸 / motion_possible / 急停**状态**） | GbE 链路本地 `169.254.10.98`，aubo SDK | 驱动栈只读边界；所有运动与 SetIO 只经 manipulation 命令门 |
| 示教器 / 急停 | AUBO 示教器；急停按钮盒（ISO 13850 Cat 0/1，IEC 60204-1） | 柜侧安全回路 | 上电 / 抱闸 / 保护停止复位；急停切断驱动电源，**不经 ROS** |
| 刀具执行器 | shear_v1 连杆剪；bite_shear_v1（BST-YF 推杆 + 喉道导轨）；adaptive_shear_v1（Ø136 圆盘 + 拉钩 + 电机） | 柜工具 DO（快换后统一走 quick_changer） | `EndEffector` 三插件（§7.1）；DO 与反馈 DI **必须分物理通道**（§7.2 反证测试） |
| 刀具反馈 | 限位开关 DI（刀闭合确认）、执行器电流曲线（剪断第二判据） | 柜工具 DI / 电流 | `confirm_cut()` 独立于剪切命令（P0-3 对策） |
| IMU / 力 | MPU6050（adaptive 刀姿态跟随）+ 末端 5 点力 `/force/points` | **同一 USB-UART（CH343 `/dev/imu`，115200）**：IMU 75 Hz + 力 100 Hz 同口时分复用（serial_imu `feed_mux`） | 摆动估计（§4.3）/ 接触监测（§6.4）；风摆 A⊥、A∥ 进许可预算（§5.1） |
| 腕六维力（**预留**） | 待选型（v2 §16 开放问题 5，强烈建议） | 多为以太网 | §6.4 接触监测首选硬件 |
| 风速计（**预留**） | 待选型 | UART / GPIO | §9 风速 > 5 m/s 暂停批次（BT `CheckSafety`） |
| 供电 | 台架：市电；上底盘后：车载电池 / 发电机（容量按柜 + NX 25 W + 相机 + 末端峰值定） | — | 果园离网作业前置（M7） |

#### 3.4.3 进程 → 计算单元映射（与 §3.3 进程表对应）

| §3.3 进程 | 默认部署 | RK3588 分流后 |
|-----------|----------|----------------|
| camera（stereo 驱动） | NX（GigE 直连） | RK3588 |
| perception_container（同步/深度质量/点云） | NX 容器 | 采集与深度质量预筛 → RK3588；点云裁剪留 NX |
| perception_infer（YOLO + MobileSAM TensorRT） | NX 容器（GPU） | 检测前置（RKNN 6 TOPS）可选拆分，语义分割仍留 NX |
| target_model / scene / manipulation / task / observability | NX 容器（不拆分） | 不拆分 |

#### 3.4.4 RK3588 分流预案（触发条件与边界）

**这是预案不是首期依赖**（对齐 §6.3 GPU 选项的写法：不作首期、实测后定案）：

- **触发条件（满足其一且经 NX 专项实测确认）**：① NX CPU 持续占用 > 80% 且套入窗内感知节拍抖动超预算；② SGBM + 推理同板争抢致 `perception_infer` P95 帧延迟超过停走节拍预算（阈值待 NX 重标后定）；③ 后续接入更高分辨率 / 帧率前端。
- **分流内容**：相机驱动与时间同步、深度质量门、（可选）检测前置；**许可预算、融合、MTC、命令门绝不离开 NX**——单点安全与「一个事实只有一个源」不受分流影响。
- **链路**：RK3588 → NX 走 GbE，只传 ROI / 深度结果 / 检测框，不常发全帧点云（对齐 §4.1「点云按需」）；两机同交换机、同 `ROS_DOMAIN_ID`，或 RK3588 侧仅跑驱动进程桥接。
- **代价**：多一台整机的供电、启动、时钟同步（PTP/chrony）与运维面；**先测单板，证明不够再分流**。

#### 3.4.5 部署约束（NX 专项，进果园前须过）

1. **操作系统 / ABI**：JetPack 6.x 宿主为 Ubuntu 22.04，Jazzy 官方发行是 24.04——NX 上整机栈跑在 **Docker 容器**（`ros:jazzy` arm64 镜像 + CUDA/TensorRT 设备直通）或源编；属部署前置验证项（与 design.md 第 10 章同一挂账）。
2. **numpy == 1.26.4** ABI 红线（Jazzy cv_bridge 按 numpy 1.x 编译）在 NX 容器内同样生效。
3. **性能锚点重标**：§12/§14 的节拍数字与 design.md 表 2-2 锚点均开发机（3090）实测；YOLO / MobileSAM TensorRT FP16、SGBM、Frangi、bag 落盘 I/O 四类须在 NX 重出锚点后再冻结停走节拍参数。
4. **8091 只读面板**默认回环监听：操作台访问经 SSH 隧道或显式绑定内网 + 白名单；不开放公网。
5. **多机时钟**：RK3588 分流落地时，相机帧 stamp 的权威时钟源须唯一（NX chrony 主 / RK3588 从或相机自身 TIME_SYNC=HOST 语义保持），避免 §4.1 时间同步目标被分流破坏。

---

## 4. 感知：检测、关键点、深度、袋轴与袋颈

### 4.1 深度前端

| 项 | 现行 | 重构 |
|----|------|------|
| 时间同步 | TIME_SYNC 失败只警告 | 失败即拒启；诊断发布主机↔设备时钟偏差 |
| 置信度 | 只写进点云，无人消费 | 独立 `depth/confidence`（mono8，与深度逐像素对齐）；下游按置信度加权 |
| 时域滤波 | 无运动补偿中值 | 只在关节静止（按图像 stamp 插值 `/joint_states` 判定）时启用 k=3 中值；运动中直接输出单帧 |
| 曝光 | IR 固定 990 | IR 自动曝光 + 彩色 ROI 测光（以袋 ROI 为准）；强光档双曝光交替 |
| SGBM | P2=3200，uniqueness 6 | P2 回到 32·bs²·ch 量级，uniqueness 10–15，加左右一致性检查；边缘飞点用深度梯度 + 置信度剔除 |
| 兜底深度 | 无 | 强光档可选 FoundationStereo（实时版，TensorRT）对左右 IR 生成备用视差；只在置信度覆盖 < 50% 时启用，需自测 |
| 发布 QoS | 全 RELIABLE depth 10，常发 7 MB 点云 | 图像 SensorDataQoS；点云按需（service 触发取帧），不常发 |
| 物理遮光 | 无 | 末端遮光罩 + 背光面作业（成本最低、收益最大） |

### 4.2 2D：基线用现有模型（v2.0），关键点模型作后续升级（v2.1）

**现有资产（2026-09-30 核实）：** `src/peach/peach_harvester/model/best.pt` 是 **YOLO 检测模型**（task=detect，两类 `peach_bag` / `peach_nobag`，只出框，不出掩膜/关键点）；`mobile_sam.pt` 是 MobileSAM（框提示分割）。袋颈/袋底现在由 `vision/common/bag_landmarks.py` 用几何推出：点云主轴 + 沿轴 12 段半径剖面（90 分位），收缩比 0.85 判断哪端是颈。**v2.0 不依赖任何新模型**，只重写这条链的质量与不确定度。

**v2.0 基线链（每帧）：**

1. **检测：** 现有 `best.pt`，只保留 `peach_bag` 进执行候选（`peach_nobag` 仅显示）。conf 0.35 / NMS 0.5 沿用现行。导出 TensorRT FP16（Ultralytics `export(format='engine')`），RTX 3090 上预期 < 10 ms。
2. **分割：** MobileSAM 以检测框为提示。改进点：
   - 每帧图像编码只算一次，多框共享 embedding（现行每帧重算编码器是热点）；
   - 框外扩 10% 作提示，另加框中心正点 + 框外四角负点，减少袋口/枝条被切掉；
   - 掩膜后处理：∩ 有效深度 ∩ 置信度 ≥ 阈值 → 取与框中心深度连通的最大分量 → 形态学开闭。
3. **2D 质量分（代替关键点可见性）：** 掩膜面积占框比、掩膜是否触边（截边）、上端（颈侧）掩膜宽度剖面是否完整、有效深度覆盖率、与邻框 IoU。
4. **2D 颈/底粗定位（新增，给 3D 做先验与交叉验证）：** 对掩膜做 PCA 取长轴 → 沿长轴 20 段量掩膜宽度 → 宽度剖面平滑后：**宽端的外缘中点 = 2D 袋底**，**窄端宽度最小处（收缩段）= 2D 袋颈**，窄端最末端 = 2D 扎口；方向用重力（图像中 base −Z 的投影）消歧。输出像素坐标 + 由剖面陡峭度给出的置信度。

**v2.1 升级（有数据后再做）：** 用 v2.0 在果园跑出的「掩膜 + 几何颈/底/扎口」作为伪标签，人工抽检修正后训练 YOLO11-seg + 三关键点（K1 扎口 / K2 袋颈 / K3 袋底）；替换第 1、2、4 步，接口不变（输出同样是掩膜 + 三个带置信度的像素点）。这样关键点模型的训练数据来自生产链自动积累，不需要单独大规模标注。

### 4.3 3D：单帧观测 → 多目标跟踪

- **单帧 3D：** 掩膜 ∩ 有效深度 ∩ 高置信度 → 点集。袋底/袋颈 3D 取法：
  - **3D 剖面法（主）：** 沿用 `bag_landmarks` 思路，但轴由「点云主轴 + 重力先验」给出，剖面半径改用切片 95 分位，颈 = 窄端连续收缩段起点，底 = 宽端 5 分位位置的轴上点（不是左右缘极值中点，后者对飞点敏感）；
  - **2D 反投影（辅）：** §4.2 第 4 步的 2D 颈/底在局部深度中值（3×3、置信度加权）上反投影；
  - 两者偏差 > 15 mm → 该帧颈/底标低质量，不进融合。
- **协方差：** 深度噪声模型 σ_z ∝ z²/(f·b) + 剖面分箱宽度（颈的轴向不确定度至少半个分箱）传播到 3D；每个观测带协方差。
- **TF：** 只按图像 stamp 精确查询，失败即丢该帧（不退 latest）。
- **跟踪：** 每目标一个 KF，状态 = 袋底位置 + 速度（风摆），观测 = 3D 袋底（袋底比袋颈稳定：颈常被叶/枝遮挡）；关联用**真正的马氏距离**（带协方差）+ 匈牙利 + 类别约束；新生需 N=3 帧确认，丢失 TTL 按时间（不按帧）。
- **摆动估计：** 停走静止窗 1–2 s 内对袋底轨迹做正弦拟合/功率谱，得到摆幅 A 与周期 T，写入观测，供许可预算与择时触发使用。

### 4.4 袋轴与袋颈估计（多视融合后）

1. **当前基线：** 单帧切片几何得到地标；按相机位姿聚合视点，再 Huber 融合底/颈/轴。默认最少视点为 1，不宣称每次经过 2–3 个独立视角。主动补视见 §4.5 候选设计。
2. **袋轴：** 当前切片圆中心线与多视轴融合，重力锥 45° 是待验证先验，不再标注为本轮已实测分布。记录夹紧/回退/轴线不一致；超出模型适用域应拒绝或补视，不能只修正数值后继续。
3. **袋颈/剪点：** 颈来自窄端收缩几何；颈到真实剪点的关系需要 M0。近距轴向 3–5 mm 等为目标/假设，不是已测成绩；近距视野和深度量程也要验证。
4. **袋径：** 当前切片半径与跨视点 d95 统计用于任务几何；完整套入包络需另验证，不将 d95 解释成所有截面的最大宽度。
5. **不确定度：** 当前 Huber/MAD 与协方差传播保留作基线；分量 1.96 倍不等于联合径向 95%。用 §18.4/§18.8 的覆盖验证决定修订。RANSAC 残差 bootstrap 仅为备选，不写成当前实现。
6. **稠密候选：** 旧点云融合/TSDF（参考体素 3 mm）保留离线对照；只有 A4 显示改善任务几何或场景表达时才接入主链。
7. **数据积累：** 保存成功和失败的 RGB、掩膜、地标与实测结果。几何伪标签只作训练候选，独立真值评测集不可直接由同一算法生成。

### 4.5 视点规划

本节为主动观察目标设计，当前 ObserveTarget 主要等待收敛/近距复测，不能按 NBV 已落地评价。到视点上限但未满足几何门应失败/跳过，不因“已拍够次数”获得许可。

- 粗：拍照位全景，锁定目标集。
- 精：对当前目标生成 3 个候选视点（沿当前视线靠近到 0.35–0.45 m；绕袋轴 ±30°；优先能看到袋颈侧的视角，即略低于袋颈、仰视袋口），用「信息增益（掩膜完整度 + 颈侧剖面完整度 + 深度覆盖）− α·移动代价」排序；每个候选先做**全臂碰撞检查 + IK**，不可达直接剔除（不试错）。
- 停止准则：`σ_lateral95 < 6 mm ∧ θ95 < 4°`（接近许可）且 `σ_axial95 < 10 mm`（v2.0 放宽，剪切精度交给近距重测与末端判据），或视点数 = 3。

---

## 5. 目标建模与许可预算（重写）

### 5.1 公式（RSS 合成，单一事实源）

以下 RSS 公式保留为当前实现的对照基线，**不是已经校准的联合 95% 保证**。仅在误差项统计口径一致、相关性可忽略、传播近似有效时才可使用相应简化；固定偏差、峰值摆动和未建模边界不能未经验证直接混入 RSS。修订后的误差分类与许可约束见 §18.4。实测常数来自标定产物；设计参考值必须带状态并阻止对应实物许可。

**径向（套入）：**

\[
m_r = \tfrac12 D_\text{inner} - \tfrac12 d_{95} - c_\text{wall} - \sqrt{\sigma_{lat}^2 + (L\sin\theta_{95})^2 + e_\text{tcp}^2 + e_\text{handeye}^2 + e_\text{runout}^2 + A_{\perp}^2}
\]

**轴向（剪切）：**

\[
m_a = w_\text{capture} - \sqrt{\sigma_{ax}^2 + e_\text{blade}^2 + e_\text{robot}^2 + A_{\parallel}^2}
\]

其中 \(A_\perp, A_\parallel\) 来自 §4.3 实测摆幅（不再用常数 3 mm）；\(w_\text{capture}\) 是**台架实测的刃口捕获半宽**（不是名义值 8 mm）。

**用现行常数对比（仅算术示意，不授权动作）：** 线性相加 `3+2+2+3 = 10 mm > 8 mm`；RSS `√(3²+2²+2²+3²) ≈ 5.1 mm < 8 mm`。这只能说明两种合成方式产生不同数值，不能证明后一种符合误差分布。必须同时标定捕获带、误差相关性、几何偏差与实际误许可率，不能通过换公式把不可行条件“算成可行”。

### 5.2 轴向误差的工艺解法（比算法更有效）

- **末端近距重测：** 到预抓取位后，腕相机在 0.15–0.25 m 处再拍一次，只重测袋颈（v2.0 用掩膜宽度剖面 + 3D 剖面；v2.1 用关键点）。近距下同样的像素/分箱误差对应的毫米误差小得多，预期 σ_ax 从多视融合的 5–10 mm 降到 3–5 mm（v2.1 关键点可到 ~2 mm）。若近距重测与融合结果偏差 > 10 mm，放弃剪切、只保留接近，入补采清单。
- **捕获带加宽：** bite 的长喉道导轨可把有效捕获半宽做到 ±15 mm 以上（需台架确认）；adaptive 圆盘贴果后用接触/IMU 判断到位，把轴向误差从视觉转移到力觉。
- **套入深度闭环：** 套入不是走固定 travel，而是「走到刃面对准袋颈」，由末端插件的到位判据（接触、力、行程）结束。

### 5.3 许可消息（单一计算权威，按阶段有界复检）

`GraspDecision` 由 target_model 唯一计算。当前实现携带 `model_revision` 与 `valid_until`，`validity_s=120` 是当前配置，不作为未来接触期限的已验收结论。目标设计将模型保存、接触观测和复测结果期限分开（§18.3）。CONTACT 与 TOOL 分别做有界服务查询；超时/失联按拒绝处理，不能阻塞取消回调。查询返回“最新”不等于观测足够新，发布/查询心跳不得重置观测时间。复测必须绑定模型变化与当前作业条件。

---

## 6. 运动：IK、规划、轨迹、避障

### 6.1 IK

- **基线：** 保留当前已配置的 MoveIt IK 链作为对照。pick_ik/trac_ik 是候选，接入前核对 Jazzy 包/API 与相同目标集成功率、连续性、耗时。IKFast 仅在真实运动学模型验证后考虑，不预设 AUBO 一定有固定 8 组解析解。
- **选解代价：** 腕关节离限位距离、可操作度（条件数 > 阈值拒）、相对当前关节的加权距离、刀口滚转约束（§7）。
- **可达性检查 = 执行同一口径：** 选果阶段 `CheckReachability` 使用执行层同一规划后端的 plan-only（含场景碰撞和工具约束），不另写只查 IK 的旁路；场景变化后不能复用旧可达结论直接执行。

### 6.2 规划分段（当前 MoveIt 阶段执行器；MTC 为候选演进）

当前可执行主线是 `HarvestCycle + MoveGroupInterface`，先补齐其正确性与 FULL 验证。下图是 MTC 候选阶段表达，不要求为框架一致性立即重写。只有需要多阶段联合搜索/场景差分且基准证明收益时再迁移；tool IO 仍由受门控的工具协议执行，不因 MTC stage 自动获得授权。

```
CurrentState
 → Connect[OMPL RRTConnect∥STOMP 竞赛, 全臂受查]      # 冠外自由空间 → staging
 → MoveTo[staging: 袋轴下方, 距袋底 0.15–0.25 m, 冠外] # 滚转梯子作为 generator
 → [视觉近距重测（task 层插入观测，§4.5）]
 → MoveRelative[Pilz LIN 沿袋轴 → 预抓取, 近果降速]
 → ModifyPlanningScene[仅允许 末端前端 × 当前目标袋]  # stage 局部 ACM
 → EndEffector.insert()                               # 插件决定：LIN / Servo / 导纳
 → EndEffector.cut() + confirm()
 → MoveRelative[Pilz LIN 沿轴撤出, fraction 必须 = 1]
 → ModifyPlanningScene[恢复]
 → Connect[OMPL, 全臂受查] → 放果位
```

- **staging 距离：** 从 8–10 cm 加到 15–25 cm（冠外），并要求 staging→预抓取的直线段在 PlanningScene 里无碰。
- **Cartesian 完成度：** 套入/撤出 `min_fraction = 1.0`，短插不下刀。
- **时间参数化：** 统一 TOTG/Ruckig，Cartesian 段额外受笛卡尔限速（套入 ≤ 0.03 m/s，冠内 ≤ 0.1 m/s）。
- **起点容差：** `allowed_start_tolerance` 回到 0.01–0.02 rad。
- **出冠：** 记录**实际执行过的全部段**（含走廊），出冠用时间反转（速度取反、加速度不变）+ 执行前重新碰撞检查；失败再规划新路径且仍全臂受查。

### 6.3 避障：分层场景（全臂受查）

| 层 | 内容 | 来源 | 碰撞策略 |
|----|------|------|----------|
| L1 固定 | 立柱、地面、底座、支架 | URDF / 静态配置 | 全臂 |
| L2 硬障碍 | 主干、粗枝（估计直径 ≥ 12 mm）、邻果袋 | Survey 快照点云 → 聚类 → 胶囊/盒拟合 | 全臂 + 末端 |
| L3 候选软障碍 | 叶片、细枝 | 分割与几何估计，含未知类 | 未验证分类和接触容差前按占用/拒绝接触处理；代价规划只能作为优化，不能代替碰撞与接触约束 |
| L4 目标 | 当前目标袋 | target_model 胶囊 | 接近段受查；仅在套入 stage 对末端前端局部豁免 |

**快照实现要点：**
- 触发时按需取一帧深度 + 按图像 stamp 查 TF 与 `/joint_states`（三者同时刻）。
- 体素化 3 cm，每体素至少 N=5 点（剔飞点）；自身滤除用 collision mesh + 余量 + 体素半对角。
- 硬/软分类：用 2D 分割的 branch/leaf 掩膜投影到体素（`peach_vegetation` 的枝叶掩膜正好用上）；无分割时退回按局部 PCA 线性度 + 厚度估计。
- 胶囊按批次清空；超上限按距 TCP 远近截断（近的保留）。
- 每个目标精化后同帧重写（去掉该目标 L4 邻域内的 L2 体素）。

**GPU 选项（二期）：** 本机 RTX 3090 + CUDA 12.8。`isaac_ros_nvblox/cumotion` 本机 apt 不可用、且对 CUDA 版本有锁定，不作首期依赖；可选 pip 版 cuRobo（研究库，CUDA 12）只用于「批量评估候选接近方向（无碰 IK 每秒数千次）」，不当生产执行规划器。

### 6.4 接触监测（全程，含撤出）

- **首选硬件：** 腕部六维力传感器（或末端 5 点力 `/force/points` 已有，接起来用）。
- **无 F/T 时：** 关节电流 + 动力学模型（重力补偿 + 摩擦标定）残差，阈值按阶段设；监测在控制循环侧（ros2_control chainable 或独立 100 Hz 节点），不在 50 ms wall timer 上。
- **动作：** 超阈值 → 请求停止当前段并撤销继续接触许可 → 上报 `CONTACT_ABORT`、保留阶段与接触证据。只有撤退方向、工具夹持状态、路径和权限重新确认后才允许受控撤退；禁止无条件沿轴退 2 cm（可能加重卡滞或掉果）。

---

## 7. 三种末端：插件接口与动作序列

### 7.1 `EndEffector` pluginlib 接口（C++）

```cpp
namespace peach_end_effector {
class EndEffector {
public:
  virtual ~EndEffector() = default;
  virtual void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                          const ToolProfile & profile) = 0;
  /// 刃面在 TCP 系中的位姿（真实 L_blade 偏移），用于把「刃面对准袋颈」换算成 TCP 目标。
  virtual Eigen::Isometry3d blade_in_tcp() const = 0;
  /// 刀口绕袋轴的允许滚转区间（轴对称返回全圈）。
  virtual RollConstraint roll_constraint(const TargetModel & t) const = 0;
  /// 该目标能否由本末端作业（袋径、颈长、遮挡、预算）。
  virtual Feasibility feasible(const TargetModel & t, const Budget & b) const = 0;
  /// 周期前置：张开刀并用独立反馈确认。
  virtual Result prepare() = 0;
  /// 生成套入 stage（LIN / Servo 窗 / 导纳），返回 MTC stage 或执行器。
  virtual InsertPlan insert(const TargetModel & t) = 0;
  virtual Result cut() = 0;
  /// 独立于命令的剪断确认（传感器 + 第二判据）。
  virtual CutVerdict confirm_cut(std::chrono::milliseconds timeout) = 0;
  /// 失败处理：开刀、退回预抓取。
  virtual Result abort_safe() = 0;
  /// 放果：不依赖感知许可，只依赖 Active∧robotReady∧¬cancel∧tool_enabled。
  virtual Result release() = 0;
};
}
```

插件：`ShearV1`、`BiteShearV1`、`AdaptiveShearV1`；`plugins.xml` 导出；`tool_profile` 参数选类名。

### 7.2 刀具 IO 与反馈（三把共用硬件规范）

- **输出与反馈分离：** 命令 DO（如工具 DO0）与反馈 DI（刀闭合限位开关，另一引脚）必须是不同物理通道；上线前做**断开执行器反证测试**（DO 置高、执行器断电，反馈必须保持低）。
- **状态机：** `UNKNOWN → OPEN_CONFIRMED → CLOSING → CLOSED_CONFIRMED → OPENING → OPEN_CONFIRMED`；任何超时 → `FAULT`（刀具锁定，须人工 ACK）。
- **剪断第二判据（至少一种）：** 执行器电流曲线（剪断时电流先升后骤降）、力传感器突变、剪后 1 cm 轻拉测试（拉力/电流无明显上升 = 已断）。
- **IO 授权：** 所有 SetIO 只经 manipulation 命令门；`aubo_io_controller` 的 set_io 服务用 SROS2 权限限制只允许 manipulation 进程调用；8090/web 调试页不得写 IO。

### 7.3 各末端动作序列

**公共前段：** staging → 近距重测 → LIN 到预抓取 → `prepare()`（开刀确认）→ 残差验证（TCP 与期望预抓取的偏差 ≤ 3 mm / 2°）。

| 步骤 | shear_v1 | bite_shear_v1 | adaptive_shear_v1 |
|------|----------|---------------|--------------------|
| 适用 | 仅细颈袋：\(d_{95} <\) ≈ 41 mm（径向余量约 4 mm）；首期作备选 | 袋颈暴露、\(d_{95}\) < ≈ 65 mm | 主力：\(d_{95}\) < ≈ 81 mm |
| 滚转约束 | 刀组必须背离相机同侧枝条（按 L2 障碍方向选滚转） | 钳口闭合方向 ⟂ 果柄/枝方向（v2.0 用袋颈上方枝条掩膜/点云的主方向，缺失时用袋轴；v2.1 用扎口关键点处枝方向） | 圆盘轴对称，滚转自由（避障优先） |
| 套入 | Pilz LIN 沿轴，0.02 m/s；行程由真实 `blade_in_tcp` 变换求得，见 §18.2，不手写偏移符号 | Pilz LIN 沿轴，0.02 m/s；喉道导轨引枝入刀区 | 候选力/导纳推进：接触力与姿态限幅须台架标定；IMU 只提供姿态信息，不代替力觉，输出须经同一命令门 |
| 到位判据 | 行程完成 + 无接触异常 | 行程完成 + 喉道底部接触（力/电流） | 贴果力稳定 0.3 s |
| 稳定 | 停 0.2–0.3 s | 停 0.2–0.3 s | 停 0.3 s（等袋摆衰减） |
| 剪切 | DO 闭合 → DI 闭合 → 电流判据 | 推杆闭合 → 行程到底 DI → 电流判据 | 电机闭合 → DI + 电流曲线 |
| 失败 | 开刀 → 沿轴撤 → 最多重试 1 次（重测袋颈后） | 同左 | 同左；贴果力异常直接中止 |
| 撤出 | LIN 沿轴 0.03 m/s，**刀保持闭合夹住果柄段**（若工艺需要夹持）或开刀后撤（按末端是否带接果） | 同左 | 同左 |
| 放果 | 放果位 `release()`，不依赖感知许可 | 同左 | 同左 |

**关于 adaptive 的 IMU 跟随：** 现行独立 `imu_follow` 未统一进命令门，默认运动关闭；真机控制映射仍需专项验证。目标是受限控制输入、同一仲裁和断流停止，Servo/导纳只在许可窗口启用。**不预先认定短段 FJT 能模拟稳定的流式控制**：须核对控制器抢占/缓冲/停止语义与端到端延迟；未证明可行时保持该能力不可用，不修改只读驱动来补洞。

---

## 8. 任务编排：行为树 + 分阶段运动

### 8.1 为什么换 BT.CPP v4

- BT.CPP 与 ROS 动作适配提供异步组织方式；是否非阻塞取决于叶节点实现，必须测试 tick、halt 和下游取消，不将框架选型等同于阻塞问题自动解决。
- 采摘有大量「失败→换视点→再试→跳过」的恢复逻辑，BT 的 Fallback/RetryUntilSuccessful/ReactiveSequence 比 FSM 表 + reducer + EventHold 三轨更直接，可在 Groot2 可视化。
- 批次层用 BT，运动执行层拥有接触阶段序列；当前为 HarvestCycle，MTC 是后续候选。不要把每个 LIN 拆成大量 BT 叶子，也不要让 BT 绕过命令门。

### 8.2 主树（XML 骨架）

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="HarvestBatch">
    <ReactiveSequence>
      <CheckSafety/>                        <!-- robotReady ∧ enables ∧ ¬estop状态 ∧ 风速<阈值 -->
      <Sequence>
        <Survey pose="photo"/>              <!-- MoveTo + 场景快照 + 目标锁定 -->
        <KeepRunningUntilFailure>
          <Sequence>
            <SelectTarget target="{tid}"/>  <!-- 可达性=MTC plan-only 同口径 -->
            <Fallback>
              <SubTree ID="HarvestOne" tid="{tid}"/>
              <RecordSkip tid="{tid}"/>
            </Fallback>
          </Sequence>
        </KeepRunningUntilFailure>
      </Sequence>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="HarvestOne">
    <Sequence>
      <RetryUntilSuccessful num_attempts="2">
        <Sequence>
          <ObserveTarget tid="{tid}" max_views="3"/>    <!-- 近距补拍直到 σ 达标 -->
          <CheckDecision tid="{tid}" level="approach"/>
        </Sequence>
      </RetryUntilSuccessful>
      <HarvestTarget tid="{tid}" mode="{mode}"/>        <!-- action → manipulation（MTC 整链） -->
      <VerifyHarvest tid="{tid}"/>                      <!-- 回拍照位看袋是否消失 -->
    </Sequence>
  </BehaviorTree>
</root>
```

`mode ∈ {PREGRASP_ONLY, FULL}`，默认 PREGRASP_ONLY；恢复需要人 ACK 时 BT 进入 `WaitForAck` 节点，不自动继续。

### 8.3 账本

每批一个 `runs/<request_id>/`：`ledger.json`（逐目标分项结果、失败码、耗时、末端、预算余量）、`rework.json`（补采清单）、会话 bag（MCAP）。

---

## 9. 室外环境干扰对策

| 干扰 | 影响 | 对策（按优先级） |
|------|------|------------------|
| 直射强光 | 结构光/主动立体深度失效，彩色过曝 | 末端遮光罩；背光面优先的作业路径；近距（0.3–0.5 m）补拍；IR 自动曝光 + 彩色 ROI 测光；强光档 FoundationStereo 兜底；深度覆盖 < 50% 时该视点作废 |
| 逆光/阴影 | 检测漏框、掩膜残缺、颈端剖面缺失 | v2.0：检测 conf 逆光档下调（不低于 0.30）、SAM 正负点提示、掩膜质量分门、多视融合；v2.1：训练数据覆盖 + 关键点置信度门 |
| 风致摆动 | 定位偏差、套入撞袋 | 停走窗估计摆幅；摆幅 > 阈值（如 15 mm）等待或跳过；择时触发（低速相位进套入）；喇叭口机械容差；套入全程接触监测 |
| 风速大 | 整体不可作业 | 风速计（或 IMU/视觉摆幅代理）；> 5 m/s 暂停批次（BT `CheckSafety`） |
| 叶片遮挡 | 袋颈不可见 | NBV 视点；L3 软障碍不进碰撞允许轻推；遮挡率 > 阈值跳过入补采 |
| 细枝 | 挂枝、撕袋 | L2 硬障碍阈值（≥ 12 mm）；细枝靠接触监测兜底；撤出沿原轴 |
| 雨后湿袋 | 反光、形变、袋贴果 | 袋径用切片 95 分位；湿袋类别训练；形变余量标定 |
| 温度 | 激光器热漂移、电机过热 | 相机温度诊断；工作时长/占空比限制；刀具电机过流保护 |
| 粉尘/雨 | 镜头污染、IP 等级 | 镜头遮罩与定期清洁检测（深度覆盖率骤降告警）；雨天不作业 |
| 地面不平（上底盘后） | 基座倾斜 | 底盘 IMU 倾角门；base_link 与 world 倾角进入规划 |
| 光照变化快（云） | 前后帧不一致 | 同一视点内曝光锁定；帧间亮度突变丢帧 |

---

## 10. 安全架构

### 10.1 三层

| 层 | 内容 | 实现 |
|----|------|------|
| 功能安全 | 急停（Cat 0/1）、保护停止、安全限速、作业区隔离 | 柜体 + 示教器 + 物理围栏/光幕（带刀单元**必须**隔离）；ISO 10218-2:2025 应用风险评估；上底盘后叠加 ISO 18497-1…4:2024、ISO 3691-4 |
| 应用护栏 | 命令门、使能、超时、接触中止、刀具状态机 | manipulation 单一命令门（下述） |
| 网络安全 | 谁能调 set_io / 运动 action | SROS2 权限：只有 manipulation 能调 set_io；只有 task 能调 HarvestTarget |

### 10.2 单一命令门（所有运动与 IO 必经）

条件：`Active ∧ robot_status(drives_powered ∧ ¬estop ∧ ¬in_error, 年龄 < 0.3 s) ∧ motion_possible（仅新轨迹下发前查）∧ enables 依赖链(execution → grasp → tool) ∧ ¬cancel ∧ 阶段许可`（源码核对 S2：robot_status 无独立 protective_stop 字段，保护停止并入柜 in_error 位；motion_possible 非持续条件）。

- **关门即停：** 任何条件变假 → 取消在途轨迹 + 硬件 `RobotMoveStop`（失败 `robotMoveFastStop`）+ 末端 `abort_safe()`；不是等下一阶段边界。
- **使能心跳：** 广播源心跳丢失 → 视为关门（不是回落本地参数）。
- **流式输入（Servo/IMU/导纳）：** 命令超时 0.1 s 即零速并暂停；进程死 → bond 触发 lifecycle 降级。
- **故障恢复：** 急停/保护停止后，取消并丢弃所有在途 goal；人确认现场 → 示教器复位 → 刀具状态机回 `UNKNOWN` 必须重新 `prepare()` 确认张开 → 人 ACK → 才允许新周期。

### 10.3 刀具专项

- 默认 `tool.enabled=false`；PREGRASP_ONLY 不需要刀。
- 输出/反馈物理分离 + 反证测试（§7.2）。
- 刀闭合电平型执行器（推杆/电机）设最长通电时间，超时断电 + FAULT。
- 急停复位后工具 DO 的保持行为**须真机核实**并写进启动自检。

---

## 11. 接口（IDL）精简版

| 名字 | 类型 | 说明 |
|------|------|------|
| `/peach/perception/observations` | `peach_interfaces/TargetObservationArray` | 每目标：id、类别、袋底/袋颈/扎口 3 个点（PointStamped+协方差+来源 `geometry`/`keypoint`，v2.0 几何、v2.1 关键点，字段不变）、掩膜 ROI、摆幅、质量分 |
| `/peach/target_model/models` | `TargetModelArray`（latched） | 袋轴、袋底、袋颈、袋径 d95、σ95、revision |
| `/peach/target_model/get_decision` | srv `GetDecision(target_id, tool_id) → GraspDecision` | 许可唯一出口（同步查询，拿最新 revision） |
| `/peach/target_model/observe` | action `ObserveTarget(target_id, max_views)` | 视点规划 + 补拍 + 融合，反馈 σ 收敛 |
| `/peach/scene/snapshot` | srv `BuildSceneSnapshot() → n_objects` | 按需取帧，写 PlanningScene |
| `/peach/manipulation/harvest_target` | action `HarvestTarget(target_id, mode, tool_id)` | 反馈：阶段、剩余时间；结果：`HarvestResult`（分项成功、FailureCode） |
| `/peach/manipulation/move_to` | action `MoveTo(named_or_pose)` | 赶路，全臂受查 |
| `/peach/manipulation/check_reachability` | srv | 调 MTC plan-only，与执行同口径 |
| `/peach/manipulation/acknowledge_recovery` | srv | 人工 ACK |
| `/peach/task/run_batch` | action `RunBatch(request_id, intent, policy)` | 批次入口 |
| `/peach/task/state` | `BatchState`（latched） | 批次态、当前目标、阻塞原因 |
| `/peach/enables` | `Enables`（latched + 1 Hz 心跳） | execution/grasp/tool |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 全节点 |

**身份：** 对外作业追踪以 `request_id`、`target_id`、`model_revision` 为主，保留观测已有 `scene_epoch` 的隔离语义，并记录工具/标定版本及 ROS action goal UUID。无需重建七元组和 Clearance 平行令牌，但不能以“精简”删除重启、换批和重复目标的区分能力。拟补字段见 §18.3，不能视为当前 IDL 已具备。

**FailureCode：** 按分项归类（感知 1x、建模 2x、规划 3x、执行 4x、末端 5x、安全 6x），每个码有唯一处理策略（重试/跳过/恢复）。

---

## 12. 参数单源与初值表

**规则：** 几何只在 `peach_description/config/tools/<tool>.yaml`；误差常数只在 `peach_calibration/results/<tool>_<date>.yaml`；运行参数只在各包 `config/<node>.param.yaml`（C++ 用 generate_parameter_library，Python 用 yaml + schema 校验）；launch 只传 `tool_profile` 和部署开关。同一量出现在两处 = CI 失败（加一个参数一致性检查脚本）。

上句路径为早期目标命名，不要求为路径整齐搬迁现有数据。当前工具档案来自 `aubo_description/config/<tool>.yaml`，v2 标定在 `peach2_calibration/results/<tool>.yaml`，节点配置以各包实际文件为准。单源指唯一权威与受控注入，合法生成/展开副本不算第二权威；参数一致性检查是待实施门，不宣称当前 CI 已具备。

**关键初值（待标定项已标注）：**

| 参数 | 初值 | 来源/说明 |
|------|------|-----------|
| 工作距离（全景/近距/末端重测） | 0.6–0.8 / 0.35–0.45 / 0.15–0.25 m | 相机额定量程 + 近距补拍文献 |
| 跟踪确认帧 / TTL | 3 帧 / 3 s | — |
| 融合体素 | 3 mm | 现行值 |
| 视点停止 σ | 横向 6 / 轴向 5 mm / 4° | §1 定位口径 |
| 静止门（关节速度，按图像 stamp 插值） | < 0.02 rad/s | — |
| staging 离袋底 | 0.15–0.25 m | 冠外 |
| 冠内笛卡尔限速 / 套入 / 撤出 | 0.10 / 0.02–0.03 / 0.03 m/s | 近果降速 |
| Cartesian min_fraction | 1.0 | 不许短插 |
| allowed_start_tolerance | 0.02 rad | — |
| 接触阈值（力） | 待标定，初值 8 N 中止 | adaptive 贴果 1–3 N |
| 摆幅上限（允许套入） | 15 mm | 待实测 |
| 风速暂停 | 5 m/s | 待实测 |
| 刃口捕获半宽 w_capture | **待台架**（每把） | 名义值不得用于放行 |
| e_blade / e_robot / e_tcp / e_handeye | **待台架** | 标定工具产物 |
| 剪切反馈超时 | 1.5 s（按执行器实测 ×1.5） | 单源在工具档案 |
| 模型有效期（当前配置） | 120 s | 待分离模型保存期/接触观测期/复测期；不能由整条动作时长直接推导接触新鲜度 |
| robot_status 最大年龄 | 0.3 s | — |
| 流式命令超时 | 0.1 s | Servo/导纳 |
| bond_timeout | 4.0 s | Nav2 默认值本身即 4.0 s（2026-10-08 核验，R18） |

---

## 13. 测试金字塔与验收门

| 层 | 工具 | 测什么 | 门 |
|----|------|--------|-----|
| Lint | ament_lint_auto | 风格 | CI 必过 |
| 纯核单测 | pytest / gtest | 预算公式（含 RSS、结构性诊断）、跟踪关联、袋轴估计（合成点云）、BT 节点逻辑、刀具状态机、倒放轨迹（加速度符号） | CI 必过，覆盖率 ≥ 80%（纯核） |
| 参数一致性 | 脚本 | 同一量多处定义、档案字段全部被消费或显式标注 | CI 必过 |
| 集成 | launch_testing（isolated domain）+ mock_components | lifecycle Active、bond、QoS 兼容、PREGRASP 链、取消路径、关门即停 | CI 必过 |
| 系统（仿真） | Gazebo Harmonic + gz_ros2_control + 程序化桃园（已有 `peach_sim` Blender 资产可导出网格） | 端到端 FULL（刀用仿真 IO）、风摆注入、遮挡注入、强光深度噪声注入 | 每日；成功率回归门 |
| 台架（M0） | 实机 + 挂袋假果架 | 标定 w_capture、e_*；IO 反证测试；剪断判据；三末端各 50 次套入/剪切 | 进果园前必过 |
| 果园 | 实机 + bag + 账本 | §1 分项口径 | 阶段目标 |

**进果园前的硬门：** IO 反证测试通过；三末端台架剪断成功率 ≥ 90%；急停/保护停止恢复流程演练通过；PREGRASP_ONLY 果园 30 次定位误差达标。

---

## 14. 分期落地计划

| 阶段 | 周期 | 交付 | 退出门 |
|------|------|------|--------|
| **M0 实测与台架** | 2 周 | 果园袋几何统计 ≥ 200 袋；台架挂袋架；三末端 w_capture 与误差标定；IO 反证测试；刀具独立反馈接线 | 数据报告 + 标定文件 |
| **M1 骨架** | 2 周 | 新包树、精简 IDL、lifecycle+bond、命令门、刀具状态机、EndEffector 插件（3 个空实现 + mock IO）、BT 主树跑 mock | launch_testing 全绿；mock PREGRASP 链 |
| **M2 感知（v2.0，现有模型）** | 2 周 | 现有 `best.pt` + MobileSAM 导出 TensorRT、SAM 编码复用与正负点提示；2D 宽度剖面颈/底 + 3D 剖面重写（带协方差）；深度前端改造（置信度、静止中值、曝光）；3D 跟踪 KF；数据飞轮落盘 | 果园离线数据：`peach_bag` 检测召回 ≥ 90%；袋底 3D ≤ 8 mm；袋颈轴向（融合）≤ 10 mm |
| **M3 建模与预算** | 2 周 | 多视融合、袋轴/袋颈估计、RSS 预算、GetDecision 服务、近距重测 | 台架：σ 达标率 ≥ 90% |
| **M4 运动与避障** | 3 周（早期估算） | 当前 MoveIt 后端整链验证；分层场景/阶段豁免；接触监测适配与倒放验证；新 IK/MTC 仅在基准触发后引入 | mock FULL 与故障注入先过；原目标仿真 FULL ≥80%、台架 PREGRASP ≤3 mm 均需实际证据 |
| **M5 三末端接入** | 3 周 | 三个插件实装（adaptive 的导纳/Servo 真机通路）；剪断第二判据 | 台架每把 50 次：套入 ≥ 90%，剪断 ≥ 90% |
| **M6 果园试运行** | 4 周 | 分项统计、失败模式归因、参数迭代 | §1 首期目标 |
| **M6.5 感知升级 v2.1（关键点）** | 3 周（与 M6 并行起步） | 用 M2–M6 飞轮积累的伪标签（≥ 3000 张）+ 人工抽检，训练 YOLO11-seg + 三关键点并替换 perception_infer；接口不变 | 同一离线评测集上：袋颈轴向误差比 v2.0 降 ≥ 40%，检测召回不降；果园 A/B 剪断成功率不降 |
| **M7 提速与上底盘准备** | 持续 | 物流优化、视点复用、双果并行规划、底盘接口（REP-105） | 每果 ≤ 15 s |

**迁移策略：** 方案原文写「新旧栈不并存运行」。现行落地是**同仓并存、不同 launch**：生产入口仍是 `peach_bringup/harvest_system.launch.py`（旧 `peach_*`）；v2 在 `src/peach2/`，入口 `peach2_bringup/peach2_system.launch.py`，不进 harvest_system / 现行 lifecycle。旧栈作对照（离线回放同一 bag）。可复用资产：URDF/meshes、手眼标定、立体前端 C++ 主体、TSDF/ICP 实现、`peach_sim` 场景、`harvest_fsm` 测试用例（改写成 BT 测试）、账本格式。

### 现行进度（2026-09-30 晚）

**这一轮要收口的软件任务已完成（M1 退出门）。方案 §1 产品验收（果园套袋+剪断）未完成。**

| 阶段 | 状态 | 说明 |
|------|------|------|
| **M0 实测与台架** | 未过退出门 | `peach2_calibration/results/*.yaml` 仍是 `design_reference`（照抄 profile，不是台架测量）。`cut_allowed` 因此恒 false。果园袋几何统计、IO 反证、刀具独立反馈未做 |
| **M1 骨架** | **退出门已过** | 十二包在 `src/peach2/`；lifecycle+bond、命令门、EndEffector 三插件 + mock IO、BT 主树。launch_testing 全绿；mock PREGRASP 链通 |
| **M2 感知 v2.0** | 代码有、退出门未过 | 仍走现有检测/分割，未做 TensorRT 热路径；果园离线召回/袋底/袋颈门未评 |
| **M3 建模与预算** | 代码有、退出门未过 | RSS / GetDecision 已接线；台架 σ 达标率未测。`require_known_swing` 默认 true |
| **M4 运动与避障** | 代码有、退出门未过 | MoveIt 后端 + 邻袋胶囊 + 残差 LIN。仿真 FULL ≥80%、台架 PREGRASP ≤3 mm 未测。系统测尚无 mock FULL（套入/剪切） |
| **M5 三末端接入** | 未过 | 插件空实现 + mock IO；adaptive 导纳/Servo 真机通路、剪断第二判据未接 |
| **M6 / M6.5 / M7** | 未过 | 果园试运行、YOLO11-seg+三关键点（v2.1）、底盘接口 |

**M1 当日验收（mock，禁止真机 SetIO）：**

| 项 | 结果 |
|----|------|
| `peach2_manipulation` gtest | 111 tests，0 fail（静态计数＝运行时 XUnit，2026-09-30 工件）；31 skip＝cppcheck 吻合；2026-10-08 修复轮（§18.16 P4）增至 **114** 全绿 |
| `peach2_system_tests` launch_testing | 26 tests（10+7+9），0 error / 0 fail（09-30 XUnit）；域 95–97；2026-10-08 另增 frozen_camera_bag pytest 2 例 |

> 注（2026-10-08 源码核对，R18·S5/S6）：本表原记「191 / 33」两个总数无法从任何工件（静态计数、XUnit、testing-log 全文检索）复现——最接近拼法 26＋当日 lint＝32 仍差 1；已按可复现口径改写。「全绿」结论本身有 XUnit 支持。同源数字还出现在 `peach2_system_tests/README.md` 与 `docs/REFACTORING.md` L725，属源码侧文件，待所有者会话同轮更正。
| `RunBatch` `execution=false` | goal REJECT，`execution_disabled`；plan-only 走 `HarvestTarget` / `CheckReachability` |
| `HarvestTarget` plan-only | `SKIPPED/NONE/planned`，`plan_only=true`，`reached=NONE` |
| mock PREGRASP 执行 | `SetEnables(execution=true)` 后 TCP 到 `pregrasp_tcp`（min_dist=0.0000），`reached=PREGRASP`；无目标批次 `no_targets` |
| 残差 LIN | `collapse_at_goal=false`；默认 `at_goal_tolerance_rad=0.002`（0.005 rad ≈4 mm，会把 3 mm 残差当已到位） |
| 四处原 xfail | 已改为硬断言（bondpy 路径、RunBatch 拒执行关、拍照位空运动、PREGRASP 不抬到 RETREATED） |

系统测命令与四项关闭说明见 `src/peach2/peach2_system_tests/README.md`。下一步有意义的两件：M0 台架标定，或把 mock FULL（仍禁止真机 SetIO）补进系统测。

---

## 15. 实施时的模型分工

在 Cursor 里按任务性质选模型，平衡额度与质量：

| 任务 | 推荐模型 | 理由 |
|------|----------|------|
| 架构决策、跨包接口设计、安全审查、疑难并发 bug | Claude Opus（主会话） | 需要全局一致性与安全判断 |
| C++ 大块实现（MTC 任务工厂、命令门、EndEffector 插件、ros2_control 相关） | Claude Sonnet / GPT Sol | 代码质量高，成本适中 |
| Python 算法实现（跟踪 KF、袋轴估计、预算、融合） | Claude Sonnet | 数值细节与测试 |
| 批量样板（IDL、package.xml、CMake、launch、yaml、BT XML 节点注册） | Composer fast | 快、便宜、模式固定 |
| 大范围代码检索、漂移扫描、参数一致性核对 | Composer fast / Gemini Flash（explore 子代理） | 读多写少 |
| 论文与开源调研、数据集搜索 | Gemini Flash / Grok（带网络检索） | 长上下文、检索快 |
| 单测生成与补齐 | Composer fast → Sonnet 复核 | 先量后质 |
| 真机前最终安全复核 | Opus + security-review 子代理 | 带刀与动臂 |

---

## 16. 开放问题（必须实测才能定）

1. **袋颈几何：** 扎口在果柄上还是结果枝上？可剪空间多长？决定三末端的适用比例和剪切目标点定义。
2. **刃口捕获半宽：** 三把末端各自对柄径、倾角、袋材的真实捕获带（台架 ±20 mm 扫描）。
3. **工具 DI 方向：** 驱动 `tool_io_states` 是否包含输出位（P0-3），决定反馈改线方案。
4. **真机控制接口：** 透传控制器的流式/抢占/停止语义能否支持所需控制？短段 FJT 只是待验证候选，不是默认替代；不兼容则该能力禁用，另行确定硬件/控制方案。
5. **腕部力觉：** 是否加装六维 F/T（强烈建议，adaptive 贴果与全程接触监测都依赖它）。
6. **深度前端室外表现：** stereo 与 Percipio 在正午/逆光/阴天下的覆盖率与误差（按 §4.1 方法实测后定默认前端）。
7. **摆幅分布：** 典型风况下袋底摆幅与周期，决定择时触发和跳过阈值。
8. **v2.0 袋颈精度是否够剪：** 用现有检测 + MobileSAM + 几何剖面，袋颈轴向误差在近距重测后能否稳定 ≤ 5 mm；若不能，相关工具保持剪切禁用，比较更好的观测、机械捕获带与关键点模型。adaptive 也必须独立通过定位、接触、控制和剪断验收，不能因为有被动柔顺就默认承担剪切。
9. **接果方式：** 剪后果是靠刀夹持、靠圆盘托住，还是自由落入接果袋——影响撤出与放果序列。

---

## 17. 参考资料

**论文与标准（节选，完整清单见调研子报告）**

- Bac et al., *Harvesting Robots for High-value Crops: State-of-the-art Review*, JFR 2014, doi:10.1002/rob.21525
- Arad et al., *Development of a sweet pepper harvesting robot* (SWEEPER), JFR 2020, doi:10.1002/rob.21937
- Lehnert et al., *Autonomous Sweet Pepper Harvesting for Protected Cropping Systems*, RA-L 2017, arXiv:1706.02023
- Xiong et al., *An autonomous strawberry-harvesting robot*, JFR 2020, doi:10.1002/rob.21889
- *Evaluation of Depth Cameras for Use in Fruit Localization and Sizing*, Agronomy 2021, mdpi 11/9/1780
- Ravi et al., *SAM 2*, arXiv:2408.00714；*SDM-D*，arXiv:2411.16196
- Wen et al., *FoundationStereo*, CVPR 2025, arXiv:2501.09898
- Zaenker et al., *Viewpoint Planning for Fruit Size and Position Estimation*, IROS 2021, arXiv:2011.00275
- Sundaralingam et al., *cuRobo*, arXiv:2310.17274；Millane et al., *nvblox*, arXiv:2311.00626
- *YOLOv8-LBP*（剪点/果柄/果底三关键点），Front. Plant Sci. 2025, doi:10.3389/fpls.2025.1656381
- *Accurate Cutting-point Estimation for Robotic Lychee Harvesting*, arXiv:2404.00364
- *HAVS* 混合视觉伺服苹果采摘，Agriculture 2026, mdpi 16/5/620
- *Force Aware Branch Manipulation*, arXiv:2503.07497
- ISO 10218-1/-2:2025；ISO 18497-1…4:2024；ISO 13850；IEC 60204-1；ISO 3691-4

**开源（选型用；链接 2026-10-08 核验可达）**

- [BehaviorTree.CPP v4](https://www.behaviortree.dev/)（MIT，Jazzy apt）、[BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)（Apache-2.0）
- MoveIt 2 Jazzy：[MTC 教程](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)、[Pilz](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html)、OMPL、STOMP、Servo；pick_ik / trac_ik（Jazzy apt）
- ros2_controllers [`admittance_controller`](https://control.ros.org/jazzy/doc/ros2_controllers/admittance_controller/doc/userdoc.html)（Jazzy，本机 apt 4.42.1 已装，2026-10-08 核）
- OSU apple-harvest（Humble，使能门与力启发式对照；未核到稳定公开仓库 URL，名称级引用）
- ros-industrial [`industrial_reconstruction`](https://github.com/ros-industrial/industrial_reconstruction)（Open3D TSDF，Apache-2.0）
- [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)（LGPL-3.0，手眼标定）
- [`mgonzs13/yolo_ros`](https://github.com/mgonzs13/yolo_ros)（GPL-3.0，只作参考，不入产品仓）

**本仓审查子报告（原文）：** 视觉链、臂与轨迹、调度与契约、三末端与安全、论文调研、开源选型六份，见本次会话记录。

---

<a id="implementation-baseline"></a>

## 18. 实施基线、契约与交付验收

**本章状态：2026-10-08 完善的目标设计与实施约束，尚未据此修改业务代码或完成实物验收。** 原方案中的预期性能、硬件候选、算法可选项以本章的基线/触发条件收敛；当前实现差距见补充评审 R14。用户要求是充分完善本方案，不表示已授权真机动作或部署切换。

### 18.1 决策基线与备选方案

| 决策 | 首期基线 | 备选与启用条件 | 明确不做 |
|---|---|---|---|
| 演化路线 D01 | v2 做后续主线；v1 维持可回放对照并修运行关键缺陷 | 完整退出门通过后切入口 | 不同时连接同一真机执行器跑两套控制权威 |
| 任务 D02 | BT 管批次/恢复；执行层管单果阶段 | 固定小流程可继续纯状态机 | 不以框架名替代取消和幂等测试 |
| 视觉 D03 | 现有 YOLO detect + MobileSAM，修输入一致性与几何 | 新关键点/分割模型须同数据验证瓶颈与收益 | 不用几何伪标签循环证明几何正确 |
| 建模 D04 | 按视点聚合的地标融合；明确未知量 | 稠密点云/TSDF 在证明改善地标或碰撞表达时接入 | 不为网格观感增加主链负担 |
| 预算 D05 | 保留当前实现作对照；先校准统计口径，再改变放行条件 | 含相关项的传播/经验分位经独立验证后采用 | 不通过减小边界或换公式获得“绿灯” |
| 运动 D06 | 当前 MoveIt 后端 + 单果阶段执行器，先补 FULL | MTC 在联合阶段搜索/场景管理收益明确时迁移 | 不强制同时引入新 IK、STOMP、MTC、Servo |
| 工具 D07 | 三插件共享生命周期，实物能力逐把声明 | adaptive 控制链经台架和控制器验证后开放 | 不把 IMU 姿态当接触力或剪断证据 |
| 场景 D08 | 固定环境 + 有效时刻快照 + 目标几何；未知按保守策略 | 软接触策略逐类/逐工具标定后启用 | 不因类别缺失或空间挖空而默认可通过 |
| 部署 D09 | 先单机验证，开发机与 NX 各有基准 | RK3588 分流仅在资源瓶颈证据充分时做 | 不把开发机 fps/TOPS 外推为车载实时性能 |

这些决策定义最小闭环，而非取消未来扩展。改变任一基线，必须记录触发问题、候选比较、实测结果与回退方式，不能仅更新架构图。

### 18.2 坐标、单位与工艺几何契约

**坐标约定：** `base_link` 下表达目标；光学坐标遵循实际相机 frame；轴 `a` 单位化并定义为袋底指向袋颈。IDL 距离用米、协方差用 m²；内部角计算用 rad，名称带 `_deg` 的字段明确在边界转换。uint16 深度单位从实际前端配置读取并验证，不将 stereo 的 0.25 mm 与 Percipio 的 1 mm 混用。

**工艺点定义：**

| 量 | 定义 | 来源/未知行为 |
|---|---|---|
| bottom | 规定袋底截面参考点，不随任意可见质心漂移 | 视觉几何；不可见/退化标 invalid |
| neck | 袋体收缩区域参考点 | 视觉地标，不自动等同剪切点 |
| tie | 扎口参考点 | 可未知，不用颈点复制值伪装独立测量 |
| cut target | 实际需要切断的工艺位置及可接受区域 | M0 建立 neck/tie 到 cut target 的关系；未知时禁剪 |
| fruit clearance | 刃面到果实的最小容许距离 | 实测/可信证据；代理值须受适用条件和保守边界约束 |
| blade_in_tcp | TCP 到刃面坐标系的刚体变换 | 逐工具测量，换工具后核对版本 |
| insertion envelope | 从预抓取到剪切位的工具扫掠体 | 工具实体 + 目标外形/误差，不只一个 d95 |

**变换定义：** 记 `T_A_B` 为把 B 系点映射到 A 系，目标刃面位姿为 `T_base_blade*`，则 `T_base_tcp* = T_base_blade* · inverse(T_tcp_blade)`。不得同时在 URDF、工具档案和动作代码三处手动加减 L_blade。

当前简化约定若 `p_tcp_blade=(0,0,-L_blade)` 且 TCP +Z 对齐 a，则 `p_base_tcp*=p_cut+L_blade·a`。从 `bottom−standoff·a` 出发，沿轴行程为相应投影；只有 cut target=neck 等简化条件成立时才化为 `length+L_blade+standoff`。这不是对真实 CAD 符号的免检证明。

**几何验收：** 在三种不同姿态核对 TCP/刃面变换；覆盖工具更换、负偏移/正偏移、非轴对称滚转和非法四元数。方向或单位不明时拒绝产生接触许可，不靠放大容差通过。

### 18.3 数据、身份与时效契约

表中“拟补”是语义需求；实施时优先复用标准 Header、现有字段和 action UUID，不新建平行契约包。当前 `GetDecision` 有 `target_id/tool_id/min_model_revision`，当前模型和许可并未具备下面全部溯源字段。

| 契约 | 生产者/所有者 | 必须保留的语义 | 消费侧行为 |
|---|---|---|---|
| Observation | perception | 图像 stamp/frame、scene_epoch、target_id、有效标记、协方差来源、camera_pose | 拒绝错系、过期、非有限值和错误 epoch；不把缺失协方差当高精度 |
| TargetModel | target_model | 几何 revision、最后证据时间、视点与质量、未知量 | 查询缓存不改变最后证据时间；重启/换 epoch 不与旧模型混淆 |
| Decision | target_model 唯一计算 | tool、model_revision、分阶段许可、有效边界、拒因；拟补标定与复测溯源 | manipulation 独立检查时效、工具/版本和运行门 |
| SceneSnapshot | scene | 快照时刻/版本、覆盖范围、未知区域；拟补版本溯源 | 与规划使用的场景一致；关键变化后重验 |
| Action | task 发起，manipulation 执行 | request、target、tool、ROS goal UUID、阶段与终态 | 同时最多一个运动所有者；重复请求不重复物理动作 |
| Ledger | task 的作业结果权威，observability 可镜像 | 尝试、阶段、结果、恢复状态、版本与证据引用 | 至多一次确认终局；重启后在途状态记 UNKNOWN/需恢复，不伪造成功 |

**时效分离（拟实施）：**

- `model_retention`：模型可用于历史查询/重新观测的保存期，不授权接触。
- `contact_observation_age`：CONTACT/TOOL 使用的最后有效观测年龄，按目标动态性与动作时长标定。
- `remeasure_age`：复测证据有效期，绑定目标、模型变化、工具标定和当前尝试。
- 有效截止时间取相关证据截止的最早值；查询/心跳不能把过期证据变新。
- ROS stamp 用于图像/TF/证据比较；本地等待与 watchdog 用单调时间。时钟倒跳、未来 stamp、仿真暂停需要专门测试，不能一律当成“年龄为负所以新鲜”。
- 目标位移、轴角/包络变化、工具/标定改变、epoch 改变或重启，应按规定撤销相关复测许可并重新验证。允许的小变化阈值从几何余量反推，不能仅比较 revision 数字。

**TOCTOU 边界：** 查询许可到实际写命令之间仍有间隔。执行层在提交前再次检查运行门和截止，执行中监视撤权与超时；无法取得新证据时停止进入下一接触阶段。仅增加一次 RPC 不构成连续目标闭环。

### 18.4 误差模型、通行余量与许可的修订设计

将每项误差登记为：随机标准不确定度、已校正偏差的剩余边界、动态位移边界、未建模形状边界。每项附单位、标定集、工具、距离/姿态适用域和有效版本。

对输出几何 `y=f(x)`，随机项的一阶传播采用 `Σ_y = J Σ_x Jᵀ`，保留可估计的交叉相关项；只有相关项可以忽略时才退化为 RSS。参考 [NIST 不确定度合成](https://physics.nist.gov/cuu/Uncertainty/combination.html)。该传播是局部近似，非线性/退化情况还需采样或经验覆盖验证。

**目标许可表达（拟替换当前单纯 RSS）：**

- 径向可用净空按完整套入路径的最小值计算：工具内包络减目标外包络，再扣墙距、随机覆盖边界、系统/形状剩余边界与动作期间动态位移边界。
- 轴向可用窗口由真实剪切捕获区域定义，扣除刃面/目标随机覆盖边界、系统偏差及动态位移，不以名义 8 mm 固定适配所有工具。
- 横向圆形余量是便于比较的保守简化；非轴对称工具必须验证姿态相关扫掠体。参数 d95 是样本形状统计量，不自动等于全行程最大障碍半径。
- 未知相关性或缺少标定时，选择可解释的保守边界/拒绝相应阶段，不能默认为零。
- `approach_allowed` 仅表示几何上允许尝试预抓取；完整运动仍须通过场景碰撞、控制器、运行授权和数据时效。它不能命名解释为“现场安全”。

**验证方法：** 固定标定集估计参数，在独立验证集测覆盖率；按工具、距离、袋型、倾角、遮挡和风况分层。检查错误落在预测区间内的比例、区间宽度和错误放行次数。拒绝“区间无限放大所以覆盖率高”的单指标优化；同时报告可处理比例与误拒绝率。

**果顶/剪点证据门：** 可验证的保守代理可以保留，但需明确适用域；未知袋内几何且无法证明刀果距离时禁剪。剪点与袋颈之间的偏差应进入标定/误差模型，而不是通过修改 neck 定义隐藏。

### 18.5 单果阶段与失败处理契约

| 阶段 | 进入条件 | 成功证据 | 失败后默认处理 |
|---|---|---|---|
| VALIDATE | Active、独占运动权、目标/工具/场景有效 | 输入版本与运行门通过 | 无运动终结，记录拒因 |
| PREPARE_TOOL | FULL 且允许该工具动作 | 独立张开反馈/工具状态有效 | 工具 FAULT；不开始套入 |
| TRANSIT/APPROACH | 可达与路径验证、execution 门 | 实际执行结果、TCP 证据 | 停当前段，按已到位置判恢复 |
| VERIFY | 已到预抓取，数据足够新 | TCP 残差与目标近距证据分别通过 | 有界修正/补测；不能把 TCP 修正当目标修正 |
| INSERT | grasp 门、当前 sleeve 许可、正确场景豁免 | 实际完整行程或已验收接触判据 | 不剪；停并评估卡滞/撤退条件 |
| CUT | tool 门、当前 cut 许可、复测/刃面有效 | 剪切动作完成 | 不自动重复下刀；进入确认/故障处理 |
| CONFIRM | 工具反馈可用 | 独立闭合证据 + 已验证剪断第二判据 | 记未确认，不能记采摘成功 |
| RETREAT | 当前状态和携带物已知、退路验证 | 退出危险包络 | 停并进入恢复；不得盲退 |
| RELEASE | 到合法放果位置、工具释放条件成立 | 释放证据 | 保留“已剪但未放”状态，不伪造完整成功 |
| FINALIZE | 所有动作终态已确定 | 唯一账本终局及证据引用 | 写盘失败报错，不重复物理动作补账 |

PREGRASP_ONLY 的成功仅表示达到预抓取目标；按策略停驻或撤出，结果必须带 reached/恢复需求。完整成功要求剪断、撤出和释放均满足本次工艺定义；若接果方式尚未确定，不能冻结 FULL 的最终成功口径。

**重试分类：** 无接触的瞬态感知失败可在同一目标总预算内有限重试；模型失配/结构性不可行直接跳过；已经接触、刀状态未知、疑似夹果、取消或保护停止不得自动重试原动作。每个 FailureCode 除码外，还结合 reached、工具状态与 recovery_required 选择策略，不能仅由一个数值码决定是否继续运动。

### 18.6 并发、取消、生命周期与进程退出

目标实现约束：

1. 单个 manipulation 同时只接受一个实际运动所有者；HarvestTarget、MoveTo、Servo/IMU 和恢复动作共享仲裁，不以不同 action 名字允许并发。
2. 生命周期失活先关闭新入口和输出门，再取消/唤醒任务、等待受管理线程结束，最后释放 ROS/MoveIt/工具资源。
3. 禁止捕获宿主 `this` 的线程超时后 detach 并允许宿主销毁；禁止混用原生 pthread join 与仍由 std::thread 管理的所有权。
4. 取消请求有独立、可运行的回调路径；不能依赖被长规划/服务等待占住的互斥回调组。所有外部等待有截止、取消检查和明确失败码。
5. C++17 下使用现有标准线程/条件变量/future 与 ROS executor 能力，不为有界等待再造通用线程框架。若第三方调用不可取消，隔离并阻止新周期，不能靠放弃线程继续执行。
6. bond 用于托管进程/节点失联；流式命令超时用于命令断流；两者均不是柜功能安全。必须测试“进程活着但业务线程卡死”，不能仅测 kill 后 bond 失联。
7. shutdown 测试区分正常退出、取消后退出和故障后强制清理；强制清理可用于环境收尾，但不得计作优雅退出通过。

**应用停止预算分解：** 检测延迟 + 仲裁延迟 + 取消传输 + 控制器接受 + 实际停止时间分别测量。软件 ACK 不等于机械臂已经停稳；是否停稳使用现有状态/测量证据，不修改驱动且不把该测量宣称为安全认证。

### 18.7 场景与工具状态的一致性

- scene 负责环境对象，manipulation 负责目标接触域和阶段豁免；每个对象/ACM 条目有明确所有者。
- 规划前冻结所用场景版本与目标几何；执行前检查关键变化。若当前 IDL 尚无版本，需要最小扩展或内部快照标识，不能仅依赖“最后收到”。
- 目标邻域挖空必须与替代胶囊/实体同一事务生效；失败时保留保守场景，不允许先清空后等补对象。
- ACM 只对指定工具部件与当前目标、指定阶段生效；成功、取消、异常、失活均恢复。测试不能只覆盖正常返回。
- 叶/细枝豁免需要独立证明分类和接触容忍；无该证明时保守处理。外形缺失、点云过期、视野外未观测区域须显式影响可行性。
- 采后携带物、刀开闭状态改变碰撞包络；退路验证使用当前状态，而非沿用空工具假设。
- TOOL_SAFE/abort_safe 不是“随时自动开刀”：开刀可能掉果，闭刀可能夹枝。默认停并报告未知，具体降能动作由工具实测及现场工艺定义，仍遵守硬件状态与授权边界。

### 18.8 算法实验、数据集与真值方案

**数据分组：** 真正独立的采集会话/树/日期为划分单元；同一连续视频的相邻帧不能分到训练和测试两侧。固定 holdout 后再调阈值。室内台架、真实果园、合成场景分别报告，合成噪声不当作室外深度实证。

**真值：** 袋底/颈/剪点采用可追溯测量或人工多视标注，记录工具精度与标注者差异；被袋遮挡的内部果顶不能只由同一算法估计作真值。标注不可观测时记未知，单独统计，不用插值充当真实标签。

**最小消融矩阵：**

| 实验 | 保持不变 | 对比变量 | 判断依据 |
|---|---|---|---|
| A1 感知管线 | 同权重、同图像/TF | v1/v2 同步、掩膜、质量处理 | 地标误差与延迟，非仅检测框数量 |
| A2 身份 | 同检测/深度输入 | 旧关联 vs KF+协方差关联 | ID switch/重现/重复处理 |
| A3 几何 | 同掩膜点集 | 简单主轴/旧拟合/切片圆拟合 | 各袋型轴/颈偏差及失败识别 |
| A4 融合 | 同独立视点与标定 | 单视、地标融合、旧 TSDF/refit | 真值误差、覆盖率、耗时和失败率 |
| A5 预算 | 同实测误差与工具 | 线性边界、现 RSS、相关性修订 | 误许可与可处理率；不得真机探索危险放行 |
| A6 闭环 | 同目标工况 | 仅 TCP 校正 vs 加新鲜目标复测 | 目标运动后的套入/剪点误差 |

每项先离线再台架。伪标签用于增加训练候选，但验证集必须人工/独立测量审查；成功采摘不能自动证明所有中间地标正确，失败样本同样保留。

### 18.9 指标、统计与成功口径

统一记录分母：全部识别/人工盘点目标、可达目标、几何许可目标、实际尝试目标。分项成功率注明条件分母；整批成功率不得用“成功数/未跳过数”冒充全部目标采收率。

- 定位报告带方向的偏差、绝对误差 p50/p95、极端失败与未知率；不能只报告平均 RMSE。
- 周期从目标开始处理到终局，分别列观察、规划、执行、工具、等待和恢复；另报整批墙钟，重试不能从周期统计中消失。
- 损伤预先定义袋撕裂/果伤/枝损的判定与观察时点；仅凭即时照片不能证明无后续果伤。
- 成功率和误许可率报告样本数及置信区间；观察到零失败不等于失败概率为零。台架每把 50 次作为阶段试验规模，不能据此宣称高置信可靠性认证。
- 原 §1 的检测 ≥95% 是产品目标，M2 ≥90% 是中间退出门，必须分别标注。§13 的 30 次预抓取与 50 次工具试验是最低阶段证据，不覆盖所有室外条件。
- 比较两版本用相同场景分层，顺序交替/随机化以降低光照、风和温度漂移；差异不明确时报告不确定，不凭单次最好结果选版本。

### 18.10 测试、故障注入与交付证据

| 编号 | 测试触发 | 必须观察的结果 | 工件 |
|---|---|---|---|
| T01 | 错单位/frame、非法协方差、未来/旧 stamp | 拒绝对应数据且有原因，无接触放行 | 纯核与节点集成结果 |
| T02 | 换 epoch、节点重启、旧模型重放 | 旧身份/许可不跨会话生效 | 消息与账本对照 |
| T03 | 预抓取后目标位移/轴变、复测过期 | 撤销旧接触条件；需重测或跳过 | 模型/决策/action 时序 |
| T04 | 执行中撤权/心跳断流 | 请求停止、无后续剪切；结果与恢复态一致 | 取消与关节状态时序 |
| T05 | 规划/服务阻塞、节点活但业务卡死 | 超时闭门、取消可响应、不启动第二运动 | 回调/线程与 watchdog 日志 |
| T06 | tool 反馈断线/卡高/命令回读 | 不错误确认剪断；FAULT/UNKNOWN 可见 | mock 注入及授权台架反证 |
| T07 | 场景更新失败、ACM 异常退出 | 不遗留过宽豁免；不清空后继续执行 | PlanningScene 前后差分 |
| T08 | 取消/异常发生于每个阶段 | reached/工具状态/恢复需求与实情一致 | 参数化系统测与 ledger |
| T09 | 空闲/在途重复启停 | 无崩溃、无线程访问已销毁对象、无残留 | 退出码、进程与日志 |
| T10 | 磁盘满/写盘失败/diagnostics 丢失 | 显式降级；不重复物理动作补账 | 记录完整性报告 |
| T11 | 三工具 mock FULL | 套入→剪切→确认→退→放顺序正确且故障可中止 | launch_testing 及阶段断言 |
| T12 | 真实台架/果园 | 几何、完整工艺与损伤目标满足适用域 | 校准文件、视频、bag、统计 |

先纯核/gtests，再隔离 ROS_DOMAIN_ID 的 launch_testing，再授权台架。每次运行前后检查并清理本次启动的进程，保留失败证据；不清理他人不相关活进程。只读文档更新不需启动机器人，本次也不执行上述试验。

验收包最少包含：版本/配置清单、测试命令、环境与依赖、原始结果、跳过原因、失败样本、统计脚本/口径、清场结果和剩余限制。未运行、跳过和通过分别计数，不能把存在测试文件当成测试通过。

### 18.11 部署、性能与可观测性

既有 §3.4 硬件图保留为部署方案。NX 的操作系统/CUDA/TensorRT/相机 SDK/ROS ABI 组合必须实际构建运行验证；`ros:jazzy` 容器名称不是自动兼容 Jetson GPU 的证明。测试使用的设备、功耗档、温度与存储应写入基准。

**性能预算拆分：** 图像采集/传输、等待同步、TF、检测、SAM 编码/解码、几何、融合、决策、规划、执行、bag I/O 分段记录 p50/p95；同时记录端到端图像年龄、丢帧和 CPU/GPU/RAM/磁盘。先测再冻结预算；无数据不填写虚构“实时”指标。

**录制压力：** 诊断/事件/账本优先完整，图像点云按预算记录；达到磁盘门时停止新作业或进入明确降级，不因 bag 阻塞感知回调。回收大工件保留摘要与审计，不静默删失败样本。

**观测责任：** 每次跳过/拒绝应能关联 request、目标、工具、模型/场景/标定版本、阶段与证据时间。统一 `/diagnostics` 与事件口径；Web 展示不成为第二状态权威或直接 SetIO 通道。工作线程耗时和取消延迟应能定位“进程在但不服务”的状态。

**依赖治理：** ROS apt 与 venv 各自归属清楚，numpy 固定 1.26.4；模型权重、TensorRT engine 与 CUDA/架构绑定记录。资源优化不得更改时间戳、坐标或许可语义。多机分流前先验证时钟偏差、断网与重连行为。

### 18.12 实施工作包与依赖

| 工作包 | 内容/涉及模块 | 前置 | 完成交付 |
|---|---|---|---|
| W0 基线冻结 | 配置、数据、权重、标定、测试现状 | 无 | 可复放版本清单、失败列表 |
| W1 执行可靠性 | v1 收尾；v2 仲裁/取消/生命周期 | W0 | T04/T05/T08/T09；不动只读驱动 |
| W2 契约与时效 | interfaces、target_model、task、manipulation | W0；物理定义初稿 | 字段迁移、旧消息拒收、T01–T03 |
| W3 数据与几何 | perception/core/model、标注与真值 | W0 | A1–A4 分层报告、失配检测 |
| W4 标定与预算 | calibration/core/decision、三工具 | W2/W3 + 授权 M0 测量 | 误差来源、覆盖率、适用域、A5 |
| W5 完整软件链 | task/manipulation/scene/end_effector | W1/W2；mock 参考数据 | T06/T07/T10/T11，不冒充实物验收 |
| W6 台架闭环 | 真实工具、控制映射、独立反馈 | W1–W5 中相关硬门通过 | T12 台架、A6、损伤与恢复证据 |
| W7 果园与切换 | 分层场景试验、部署基准 | W6 + 部署兼容性 | 完整指标、回退演练、入口切换记录 |

这是依赖关系，不是未经资源评估的工期承诺。原 §14 周数保留为早期估算，实际排期应在 W0 明确可用硬件、数据和操作者窗口后更新。W3 离线数据工作与 W1 可以独立推进，但接口变更按 W2 单一版本收口，不让多条实现自行发明字段。

### 18.13 新旧栈迁移、切换与回退

1. 先同 bag 离线对照；必要的消息转换仅用于测试数据，不在实物命令链建立双向兼容桥。
2. v1/v2 同仓可以，不同时占有同一驱动/相机/运动端口；切换前停止并确认旧实例退出，检查 TF 无重复。
3. mock FULL 与故障注入通过后，逐工具台架验收；未过的工具保持不可用，不因另一工具通过而整体放行。
4. 切换包包含已冻结参数、模型、标定、依赖、完整测试及可恢复的旧版本。回退以停止当前作业、丢弃在途 goal 为前提，禁止把旧动作在另一版本恢复执行。
5. 出现误许可、错误剪断确认、未能撤权、线程崩溃或越界场景豁免，应停止扩大验证范围并回到相应工作包；没有明确证据不得靠提高重试次数掩盖。

切换决策依据实际成功率、损伤、误许可、周期、人工恢复和稳定性。框架更漂亮、测试数量更多或开发者主观偏好都不是退出门。

### 18.14 开放问题的关闭条件

| 问题 | 所需证据 | 未关闭时约束 |
|---|---|---|
| 袋颈是否对应剪切区 | 实袋结构测量、剪切位置扫描 | 不授权真实剪切 |
| 三工具捕获窗口/通行包络 | 各袋径、倾角、姿态、材质台架矩阵 | 仅限定已验证适用域 |
| DI 是否独立、第二判据是否有效 | 断开执行器、卡高/卡低、未剪断对照 | 不把命令回读算成功 |
| adaptive 控制接口与力觉 | 控制器语义、延迟/停机和实物接触测量 | 保持导纳不可用；不预设 FJT 替代 |
| 近距成像是否有效 | 各工作距离完整视野/有效深度与误差 | 不因“更近”自动放宽质量门 |
| 果顶代理是否保守 | 可观测真值与袋型分层误差 | 证据不足则禁剪 |
| 风摆预测是否可信 | 不同风况窗口/残差/动作时延验证 | 未知不视为零运动 |
| 采后如何持果/放果 | 实物夹持/托接/释放实验 | FULL 成功口径不冻结 |
| NX 与依赖组合可运行 | 目标机安装、负载、热与 I/O 实测 | 不承诺开发机同等性能 |

这些是物理或部署数据缺口，不能用模型推断填成已完成结论。每项关闭后在本章补日期、证据链接、适用范围和残余风险，并同步受影响运行快照。

### 18.15 文档与变更完成标准

本方案完善的完成标准是：评估可回查、现状与目标分开、决策有取舍、接口有语义、算法有有效条件、失败有处理、工作包有退出证据、物理未知有关闭路径。**不要求也不允许为了文档“完整”而编造实测参数。**

本轮只完善本文件；没有更改 IDL、配置、运动程序、工具参数或驱动。后续实施触及行为时，同轮更新三份活文档及相关过程记录；本节待办不得在代码未验证时勾选完成。

### 18.16 2026-10-08 软件正确性实施轮

**目标：** 按用户后续“完善 v2 项目代码和算法”指令落实时效与融合状态正确性；本节是实施记录，§18.15 的“只完善文档”仅指此前方案编写轮。

**架构与范围：** 在现有 `peach2_target_model` 纯核/节点与 `peach2_manipulation` 命令门内修复，不改驱动、不增加并行授权体系，不以未标定常数放宽剪切。Jazzy/C++17、numpy 1.26.4；先复现再修改，受影响包完整测试后更新三份快照。沿用本文件 §18.3/§18.6 的设计，在当前会话逐项执行，不另开方案文件。

**审查关注：** 重复/倒序/非有限观测、旧融合迟到、重复查询续期、摆幅证据过期、复测状态跨模型/会话复用、时钟回跳和命令门输入非有限值。

- [x] P1：`decision.py` + `test_decision.py`：用固定最后观测时间、两次查询及 future/NaN/Inf 时间重现错误；截止绑定证据而非查询，非法时间拒绝；跑该文件再跑整包。（2026-10-08 完成：`valid_until` 锚定 `last_obs_s`；非有限/未来证据 fail-closed 拒 MODEL_EXPIRED、valid_until 保持有限；`test_decision.py` 38 用例含新增 2 项全绿，整包 `colcon test peach2_target_model` 80 tests 0 fail）
- [x] P2：`store.py` + `test_store.py`：用倒序/重复帧、延迟 FusionOutput、窗口后 unknown 摆幅重现污染；拒绝重复/倒序提交，窗口以最新观测计；覆盖正常多视和 epoch 行为。（2026-10-08 完成：add_frame 拒重复/倒序/非有限 stamp；commit 拒迟到融合；视点锚固定为开视帧位姿（buffer 滚动不漂移）；swing 窗口锚最新观测；`test_store.py` 30 用例含新增 7 项全绿）
- [x] P3：模型复测状态与节点：新几何修订撤销 PASSED、保留 MISMATCH；复测提交核对 store/epoch/模型基准，避免异步结果跨会话写入；观察等待使用单调时限，图像判据仍用 ROS stamp。（2026-10-08 完成：commit 修订即撤 PASSED（MISMATCH 粘滞保持，测试锁定）；`_remeasure` 快照 epoch+revision 写回前校验，失配终局 MODEL_STALE=21 不写状态；observe/remeasure deadline 改 `time.monotonic()`，`frames_since` 仍用 ROS stamp）
- [x] P4：`command_gate.cpp` + 对应 gtest：检查非有限时间、未来接收时间与 deadline 边界；执行门 fail closed，保持既有阶段语义。（2026-10-08 完成：enables/robot_ready/TOOL_SAFE 三路非有限时间 fail-closed；robot_ready 补未来 1.0 s 护栏（对称 enables）；新增 3 个边界 gtest（非有限、未来、`>` 等值边界），`test_command_gate` 14 用例全绿）
- [ ] P5：目标包 pytest/colcon test、C++ gtest 与包测试；记录失败和 skip，检查无本轮残留进程；更新 README、architecture/io/testing 和 testing-log 的实现事实与限制。（2026-10-08 部分完成：四包 `colcon test` 全绿——peach2_core 80 / peach2_target_model 80 / peach2_manipulation gtest 114（111+3 新增）/ peach_arm 构建与测试过；本轮无新增残留进程（并行会话的 orchard_nav 栈属其在途工作，已报告未动）；testing-log 已追加；architecture/io/testing 三活文档正被并行轮修改，避让不碰，归所有者会话同轮收口后勾选）

本轮不宣称完成 M0、真实剪断、连续视觉伺服、统计覆盖率标定或整套迁移；这些仍依赖 §18.12 后续工作包与实物证据。

---

<a id="review-20261008"></a>

## 补充评审：2026-10-08 v1/v2 软件框架、算法与验收

本节汇总 2026-10-08 三轮项目评估及详细讨论，集中记录结论、依据、设计取舍、算法假设、风险与后续完善事项，避免结论只留在对话中。**这是静态评估与后续工作建议，不是产品验收报告。** 原方案正文保留；下文指出的“方案能力”与“当前实现”差别，应在后续实施时逐项回填，不应把规划图当作已经落地的系统。

### R0. 范围、证据与阅读口径

- 范围：生产入口旧 `peach_*`（本文称 v1）、隔离重写 `src/peach2/`（本文称 v2），包括框架、感知、身份、多视建模、许可、场景、运动、末端、测试与观测性。
- 本轮已做：读取关键源码、配置、三份活文档、既有测试产物及验收记录；运行旧栈接口清单静态检查，结果为 **57 个现行名称、5 个预留名称通过**。
- 本轮未做：重新编译、重跑整栈/性能测试、机器人运动、SetIO、台架测量和果园试验。不能据此声称精度、速度、采摘率或安全性获得实测提升。
- 初次评估收尾时，所检查的 ROS 进程模式无匹配；这只说明该次检查时刻的结果，不是持续机器状态保证。
- 工作区已有未提交修改，评估针对当时工作树，不对应一个已冻结的发布版本。后续验收应记录 commit、工作树差异、配置、模型权重与标定版本。
- 证据分四类：**源码已核实**、**历史运行证据**、**设计判断/推断**、**待实测**。后文不将后三类混写成已通过事实。
- 原方案 §2 的历史问题表包含“子审查”“待真机实证”；本节不代表对该表全部条目重新完成逐行复核。
- 本节记录如何完善；部署现状仍以 `architecture.md`、`io.md`、`testing.md` 与源码为准，不将本方案扩展成第四份运行快照。

### R1. 总体判断与演化方向

**总体判断：v1 已有较完整的机器人应用工程基础，但仍属于研发验证系统；v2 的职责边界、许可表达和可测试性更适合后续主线，算法实际效果尚未证明全面优于 v1。**

v1 的主要负担是现场迭代产生的状态分散、重复路径与例外，已经影响正确性推理和收尾可靠性。v2 的主要负担是关键几何与统计假设、动态目标处理及实物工艺闭环尚未得到充分验证。

建议：后续能力建设以 v2 为主，v1 保留真实数据、回放资产、现场行为和失败案例作为对照；旧栈仍在使用期间应先处理收尾及撤权等关键缺陷。**本建议不等于立即切换生产入口，也不要求立刻全面拆包、追加 CI/Gazebo 或更换所有算法。**

保留价值较高的共同思想：停走式感知、精确 stamp TF、纯核零 ROS、逐目标动态预算、默认预抓取干跑、失败可定位、接触后恢复语义、会话账本与 bag 证据。框架名、包数、测试数量及公式复杂度都不能替代完整采摘证据。

### R2. 产品需要的五类事实及物理定义

套袋桃采摘至少需要五类不同事实：

1. **目标身份**：当前观测是否属于之前那一颗，是否已处理。
2. **套入口与通行包络**：从何处进入，整个推进区间是否允许工具通过。
3. **袋轴/推进方向**：表面主轴、袋底到袋颈连线和可套入方向可能不同。
4. **剪切目标**：果柄、扎口或短枝的实际位置，不应未经标定就等同于几何袋颈。
5. **周围障碍**：工具、相机、手腕、臂身及携带物经过的空间。

必须分别定义袋底、袋颈、扎口、刃面和真正剪切对象的坐标与真值获取方法。表面拟合准确不代表袋颈准确；袋颈准确不代表刃面落在可剪区；TCP 到位不代表刀口到位；终点可达不代表进出路径可行。接口命名清楚只是第一步，视觉地标与真实工艺位置之间的对应关系需要台架和实袋验证。

### R3. 框架对比：职责、进程与状态所有权

| 维度 | v1 当前特征 | v2 当前特征 | 评价与约束 |
|---|---|---|---|
| 分层 | 感知/重建/调度在 brain 同进程，臂独立 | perception/target_model/scene/manipulation/task 分开 | v2 更便于独立测试；增加通信、时效和生命周期协调成本 |
| 批次编排 | 表驱动 FSM + reducer + EventHold + 命令循环 | BT.CPP 子树 + 会话/黑板 + 动作节点 | 改善来自状态归属清楚，不是 BT 天然优于 FSM |
| 观察 | fast/conservative 分布在调度与臂两侧 | ObserveTarget 与任务树接线；当前主动补视能力有限 | 避免再次维护两套观察权威；NBV 不应当作已完成 |
| 许可 | clearance 令牌与快照回退两路 | GetDecision + approach/sleeve/cut + 执行前复查 | v2 更易解释；仍需统一版本、时间与复测失效语义 |
| 末端 | 工具档案与流程分支 | pluginlib + 工具几何 + 阶段协议 | 可替换边界更自然；插件加载成功不证明实物能力 |
| 公共逻辑 | 纯核散布在现有业务包 | peach2_core 集中几何/统计纯核 | 防止 core 吸收会话、线程、参数系统或 ROS 客户端 |
| 运行可靠性 | 有较多现场与故障证据，仍有退出线程问题 | M1/mock PREGRASP 有通过记录 | 成熟度需按能力分别评价，不能只按新旧命名判断 |

v1 同进程减少了通信和序列化，但一个阻塞/崩溃可能影响多项能力。v2 分进程隔离推理与执行更清楚，但应监测跨进程延迟和数据年龄。Python 适合模型生态和算法迭代，C++ 适合 MoveIt 与执行控制；语言本身不提供硬实时或功能安全保证。

跨包应继续只通过清晰契约或确有必要的共享纯库依赖，不增加平行 Manager/Adapter 框架。一次目标结果、claim、恢复状态和账本各自需要唯一所有者，不能由 BT 黑板、动作节点和会话对象各维护一份互相同步的真相。

### R4. 调度算法：FSM、BT、取消与选果

**FSM 并非 v1 的落后之处。** 固定“巡检→选果→观察→执行→记录”流程使用表驱动 FSM，具有状态可枚举、转移可单测、日志易回放的优点。v1 的主要问题是外围叠加 reducer、暂停命令暂存和观察分支，导致一个事件的后果分散。

v2 `harvest_batch.xml` 使用 `ReactiveSequence/CheckSafety`，将观察、许可检查、执行、重试与 ACK 表达为子树，适合深度差补视、摆动等待、不可达跳过、接触失败恢复等分支。但必须逐项核对：

- `halt()` 是否取消下游 action 并最终停止输出，而不是只改变树节点状态。
- 重试是否会重复 claim、重复记账、重复剪切；每次重试是否消耗同一目标总时限。
- “失败且没动过”与“失败但已移动/接触”是否有不同恢复语义。
- 暂停、取消、撤权、故障恢复是否被混为一种中断。
- 超时/重试装饰器与叶节点内部重试是否叠加成不可预测的总次数或时长。
- 最后命令门必须留在执行侧；BT CheckSafety 不能代替执行侧强制点。

选果方面，v1 主要使用资格门、深度/可达窗、优先级和框面积；v2 `selection.cpp` 显式处理锁定集、质量、深度、可达性，再按点名次序、可达等级、高度带、距离和面积排序。这是可解释的启发式，不是全局采收路线优化。现阶段合理，但不能由此宣称总周期最优。后续若优化，应测真实移动代价、遮挡变化、成功概率与失败恢复成本，不能仅按框大或距离近推断收益。

### R5. 单帧感知：已改进的部分与共同上限

两代当前主要仍使用 **YOLO detect + MobileSAM**。v2 不能按已经完成关键点网络、完整 TensorRT 热路径或新检测精度来评价；配置入口或后续方案不是已验收能力。

v2 可确认的工程改进包括：容量 1 的 LatestSlot、回调外推理线程、图像时刻 TF（失败丢帧）、深度质量与协方差、2D/3D 地标交叉检查、epoch 变化后的旧帧拒收，以及按原因诊断。这些主要改善延迟和输入一致性。

取舍：最新帧策略减少积压，但可能丢失短暂无遮挡的好帧；不能替代主动观察的视角管理。2D 与 3D 地标往往共享图像、掩膜和部分深度来源，它们一致是自检，不是独立传感器互证。同一个错误掩膜可能同时污染两条计算。

共同难例包括过曝白袋、相邻袋粘连、枝叶遮挡袋底/颈、检测框截断、掩膜混入邻袋和枝条、有效深度不足。代理深度置信度主要反映局部几何连续性，不能自动等同相机测量可信概率。

待完善：按光照、距离、遮挡比例和袋型分层统计漏检/误检、掩膜污染、有效深度率、地标误差和端到端延迟。升级识别网络之前先确定瓶颈在检测、分割、深度还是几何，避免模型升级掩盖接线/标定错误。

### R6. 身份跟踪与锁定集合

v2 当前是常速度 KF、创新协方差 `P_pred + R` 的马氏门、类别约束和匈牙利匹配，按图像 stamp 推进；相较仅靠距离关联，噪声与短时运动表达更完整。袋底锚点可减少袋颈遮挡导致估计长度变化时的虚假中心移动。

限制：邻袋接近/交叉摆动会混配；遮挡后重新出现不必然恢复原 ID；协方差低估可能排除正确观测，高估则可能接纳邻袋。**平滑轨迹也可能属于错误目标。** 常速度模型适合局部预测，不等于真实风摆动力学。

验收应包含 ID switches、遮挡恢复原 ID 比例、重复处理率、已采袋与邻袋误关联、锁定目标消失后的调度行为。锁定集合有利于稳定批次分母和账本，但需要明确新出现目标、长期消失目标和重新巡检的 epoch 语义，不能只以“集合已锁定”替代新鲜观测。

### R7. 袋几何：旋转体先验与真实通行方向

v1 使用柱/球等几何拟合；产品实际执行范围是套袋，裸果显示能力不等于裸果采摘验收。

v2 `landmarks3d.py` 使用切片圆拟合、稳健重加权、中心线估计、半径剖面和重力约束。切片圆心针对“仅看见前半表面，质心向相机偏移”的问题，具有明确动机，不是简单 PCA 换名。

主要模型失配：真实纸袋存在扁截面、折角、封底、果实局部鼓起、余纸和枝条压迫；最佳圆拟合轴不必然等于最佳套入轴。表面主轴、底颈连线、工具可进入方向应分别理解。

约 45° 重力锥先验能抑制错误方向，也可能把真实倾斜袋子修正为看似合理的轴。应记录约束前后角度、退化/回退次数及误差变化；先验不能静默掩盖失配。

待验证：不同袋型和可见弧段下的轴线偏差；袋径估计是否覆盖工具实际经过的最大外形；袋底/袋颈部分不可见时是否正确降级；真实扎口/果柄与估计颈点的偏移分布。不能只用规则圆柱合成数据证明真实纸袋可套入。

### R8. 多视建模：稠密表面与任务地标的取舍

v1 主路径是 ICP/TSDF 后几何 refit，保留较丰富表面信息，但依赖配准、相机外参、掩膜一致性与目标近似刚性。袋子风摆、变形或掩膜跳变可能造成重影和错误融合；稠密网格并不天然更准确。

v2 当前主链是袋底/袋颈/轴按视点聚合后 Huber 融合，**不是方案图中的完整 TSDF 主链**。对于对轴、套入和刃面定位，这是合理的轻量任务表达，但会丢失部分局部鼓包、折角等形状细节。

v2 按相机位姿将同一视点连续帧聚合，保留代表性的单帧协方差，再做视点间融合，避免原地多拍被当成多次独立证据，是实质改进。

仍需注意：跨视点共享手眼误差、深度偏差、几何先验，不能按独立观测无限缩小不确定度；多视一致可以是共同偏差。默认 `min_views=1` 表示“融合接口支持多视”，并不意味着每次许可必经多视验证。

建议分别维护任务几何与避障几何的要求：前者可以精简成地标/轴，后者不能因此忽略真实外形。验收比较收敛时间、地标误差、残差与真值误差的关系，不只比较网格观感或内部拟合残差。

### R9. 不确定度与许可预算

v1 线性叠加误差，易解释但可能过度保守。已核实默认精化预算示例：颈轴向下限 3 mm + 刃面 2 mm + 机器人 2 mm + 目标运动 3 mm，至少 10 mm，超过 8 mm 刀口捕获半宽，默认组合轴向余量至多 −2 mm。这说明该默认组合不可通过；不应外推到所有配置或绕过重建路径。

v2 使用 RSS，拆分 approach/sleeve/cut，并用标定状态与颈复测约束剪切，表达更清楚。但 **RSS 只有在误差定义和相关性假设成立时才有统计意义**；不能因为公式使余量变正就认定工艺可行。

每个误差项必须回答：

1. 随机噪声还是系统偏差？
2. 标准差、95% 分位、峰值还是保守上界？
3. 单轴误差、二维径向误差还是三维距离？
4. 是否与其他项共享图像、外参、估计模型或标定来源？
5. 是否随姿态、距离、速度、动作时间和工具变化？

改进方向（设计建议，尚未实施）：可消除偏差先标定；随机项用协方差及相关性传播；未建模误差留明确边界；风摆峰值与系统偏差不未经验证地全部按独立高斯项处理。

当前融合横向最大标准差乘 1.96，不直接构成二维径向 95% 覆盖保证。Huber/MAD 改善异常值鲁棒性，也不能自动证明小样本置信区间正确。应做真实覆盖率与误许可率验证。

果顶未知时，当前使用 `length − d95` 作为剪切点到果顶距离代理。它假设果贴袋底、果径近似袋径，**不是袋内果实观测**；余纸、偏心果和异形袋会破坏假设。需显式记录代理使用率、适用条件及真袋误差，必要时在证据不足时限制剪切。

### R10. 动态目标、有效期和近距复测

v2 当前 `validity_s=120`，查询时 `valid_until=now+validity_s`，同时以最后观测年龄拒绝过期模型。这不是无限续签，但把“历史模型保存期限”和“接触位置证据期限”靠得过近。

建议区分三种期限：模型可保存多久、接触前观测最多多旧、复测通过可复用多久。阈值应从风摆/动作持续时间实测反推，而非单纯放宽旧栈过期门。

近距复测使用新图像 stamp、距离门和多帧颈点比较，优于只检查旧模型；但仍要定义复测结果与 model revision、目标位置变化、工具及当前周期的绑定。**之前复测通过不等于后续仍成立。** 当前复测状态如何随新模型变化失效，应列入专项审查和故障注入。

v2 摆幅估计采用短窗口正弦与漂移联合拟合，适合近似周期摆动；阵风、非周期扰动、短窗口和遮挡跳点可能产生误导结果。应评价拟合残差、周期可信度、未知比例及对许可的影响；输出一个周期不代表周期有充分证据。

### R11. 运动、闭环与退出路径

v1 有 MTC/Pilz 接近及现场调试积累，但许可双路、线程和共享执行状态增加复杂性。v2 当前是 **HarvestCycle + MoveGroupInterface**，已有滚转采样、分段规划、预抓取残差修正、许可复查及逆路验证，不应称为已经落地的 MTC 任务工厂。未来是否迁 MTC，应取决于阶段组合、场景差分与搜索需求，不以框架名作为目标。

必须分开验证：终点 IK、路径碰撞、轨迹可执行性、执行期间目标是否仍有效。当前 TCP 与规划目标的残差修正主要处理机器人执行误差，**不等于视觉伺服**；袋子位移需要新图像与目标重测。

近距颈复测属于离散观测闭环，不能直接描述为连续视觉跟踪。滚转采样是可解释候选搜索，但离散分辨率、关节连续性、相机遮挡与工具姿态约束均需评价。

逆向退出已走路径有合理性，但采后果实被携带、枝条回弹、工具形态或场景发生变化，回程必须重新验证。来时可通行不保证回去可通行。失败后的撤退也受当前授权、控制器和现场状态约束，不能把“自动撤退”当成始终安全的默认补救。

### R12. 场景避障与三末端物理差异

v1 场景快照适配停走式腕载相机，但原有场景障碍主要保护相机，不能外推为全臂现场避障。v2 分层场景、目标胶囊和阶段豁免方向更合理，仍需验证点云/TF/关节同时间、掩膜缺失时策略、目标邻域挖空边界与 ACM 恢复。

目标几何被从场景中移除并交给胶囊表达时，应验证原障碍没有被过度挖空；枝叶语义不可靠时，也不能把“未识别为硬障碍”当成“可安全接触”。快照冻结降低噪声和计算成本，但会遗漏作业中的动态变化。规划场景始终只是应用护栏。

v2 末端插件将刀口偏移、roll、prepare/cut/confirm/release 集中，值得保留；共享接口不代表共享物理条件：

- shear/bite/adaptive 的刃面偏移、允许袋型、可吞入长度和接近姿态应分别标定。
- 非轴对称 bite 工具不能只用圆形内径判定通行；导轨、刀片和入口方向需要姿态相关包络。
- 当前枝方向输入不足时，不能按已具备完整枝方向滚转约束评价。
- adaptive 导纳/力反馈与真机通路未完成产品验收，LIN 回退不等同导纳能力。
- DO 命令成功、刀闭合、负载变化、果实脱离是不同事实。剪断确认必须独立于命令，并验证第二判据。
- 三份标定仍为 `design_reference`；不得通过仅改状态字段替代实测。

### R13. 本轮关键缺陷与历史验证边界

| 优先级 | 发现 | 证据类型与边界 | 后续处理 |
|---|---|---|---|
| 最高 | v1 `pthread_timedjoin_np` 使用 `{2,0}`；该 API 需要绝对截止时间，非相对 2 秒 | 源码与本机 man page 核实 | 统一线程所有权和停止协议，禁止以错误超时处理线程 |
| 最高 | 原生 pthread join 成功不清除 std::thread joinable 状态；捕获 this 的线程还有 detach 路径 | 本机标准库头与源码核实；未动态复现全部触发链 | 验证析构/赋值、在途动作取消、重复启停；不能只加异常捕获 |
| 高 | E1 历史产物记录 peach_arm 退出 −6 | 历史运行证据，本轮未复跑；不能断言唯一根因 | 修复后在途/空闲两类 shutdown 回归 |
| 高 | imu_follow 开启运动时直接向 Servo 发布，仅使用自身开关 | 源码核实；整栈默认 false | 统一仲裁与撤权，验证与其他运动源互斥 |
| 高 | E1 执行使能关闭用例被 skip；退出断言接受 −9 | 源码核实；不证明在途撤权或优雅收尾 | 区分进程已清除与优雅停止，补真实撤权路径 |
| 高 | 真实剪切标定、独立反馈、IO 反证、第二判据未闭环 | 当前配置与里程碑记录 | M0 台架与工具专项验收 |
| 中 | 既有 stereo 测试产物含 lint 失败 | 历史产物，不等于当前重新运行结果 | 固定版本后重跑相关检查，不宣称全绿 |
| 中 | 架构文档有旧 autostart 非目标表述，方案图包含尚未落地能力 | 文档与当前约束/源码对照 | 修改行为时同步三份活文档，区分计划和现状 |

**bond 状态更新：** 不能继续沿用“缺 bondpy 所以未生效”的旧判断。2026-10-08 项目指令已明确 apt 包在装，Python import 已修为 `from bondpy.bondpy import Bond`。旧栈 `bond_timeout` 仍默认 0，开启前需 mock 冒烟；v2 launch 默认 4.0。接线、默认值与实际故障检测验收是三个不同事实。

历史记录还曾指出 venv/ROS diagnostics 表示与运行环境一致性问题；本轮未复测，不能认定已解决或仍必然存在。后续验收应固定解释器、ROS 消息来源与 numpy 1.26.4，并验证诊断直播与 bag 两条通道。

### R14. 方案能力、已实现能力与验收状态

| 能力 | 本轮看到的实现/记录 | 不应提前宣称 |
|---|---|---|
| v2 任务骨架 | BT、命令门、插件、mock PREGRASP，M1 有退出门通过记录 | 完整果园采摘已验收 |
| 检测分割 | 复用 YOLO detect + MobileSAM，允许配置 engine | 新关键点模型或 TensorRT 全链性能提升已完成 |
| 建模 | 地标按视点聚合和 Huber 融合 | 稠密 TSDF 主链已迁入 v2 |
| 主动观察 | ObserveTarget 等待收敛/复测，视点运动另由任务侧承担 | NBV 主动视角优化已实现验收 |
| 运动 | 自定义 HarvestCycle + MoveIt 后端 | 完整 MTC 工厂与连续视觉伺服 |
| 末端 | 三插件的软件协议、几何与 mock 测试 | 三种实物工具均已可稳定剪断 |
| 误差预算 | RSS、分阶段许可、标定状态门 | 95% 真实覆盖率及误许可率已证明 |
| 产品指标 | 原方案列出了目标指标 | 目标数值就是实测成绩 |

按现有里程碑，M0 未过；M1 已有通过记录；M2–M5 多为代码存在但退出门未过。后续记录必须注明测试输入是否为 mock、标定是否实测、末端是否真实接入，不能将 PREGRASP 成功计为完整采摘成功。

### R15. 同数据对比与验收矩阵

比较 v1/v2 必须固定数据、末端、标定、成功口径及资源条件，保留失败样本。以下是建议测量项目，不是已完成测试或新增硬性数值门槛；具体阈值结合原方案 §1/§13 与 M0 结果确认。

| 层 | 指标 | 必须区分的条件 |
|---|---|---|
| 感知 | 召回/误检、掩膜污染、有效深度率、地标误差、端到端延迟 | 光照、距离、遮挡、袋型、前端、真实/合成 |
| 身份 | ID 切换、重现恢复、重复处理、邻袋误关联 | 静止/摆动、密集/稀疏、短/长遮挡 |
| 模型 | 袋底/颈/轴误差、收敛时间、视点数、不确定度覆盖率 | 单视/多视、外参偏差、真实袋形失配 |
| 许可 | 误许可率、误拒绝率、失败原因正确性、代理几何使用率 | 三末端、标定状态、风摆、过期观测 |
| 规划执行 | 规划成功、预抓取误差、进出路径成功、取消/撤权响应 | PREGRASP/FULL、空闲/在途、目标更新/障碍变化 |
| 工艺 | 套入、剪断、撤出、放果、损伤、单果周期、人工恢复次数 | 可达目标与全部目标分别统计；三末端分别统计 |
| 运行 | 重复启停、进程故障、传感断流、诊断与账本完整性 | 正常退出/取消/强制清理分别统计 |

报告应明确样本数量、失败定义、分母、分位数和不确定性。只统计系统愿意处理的简单目标会夸大成功率；需要同时报告全部目标、可达目标和许可目标。不同硬件/果园条件的论文指标仅作参考，不能直接当作本系统性能背书。

优先故障注入：使能心跳丢失、中途撤权、目标 revision 变化、观测过期、TF 缺失、深度断流、邻袋 ID 混配、模型复测失配、控制器拒绝、工具反馈超时、节点失活/崩溃和 shutdown 在途动作。每项需检查动作终态、运动输出、恢复状态、账本唯一性及进程收尾。

### R16. 建议执行顺序与取舍结论

1. **先修正在用旧栈的关键可靠性问题：** 线程所有权/收尾、取消与撤权、重复启停；不以接受 −9 作为优雅停止验收。
2. **冻结统一对比基线：** 真实 bag、标注/真值、模型/标定版本、配置和运行环境；真实测量与合成场景分开标记。
3. **校准算法：** 袋底/颈/轴、95% 覆盖率、跟踪身份、RSS 各项来源与相关性；首先证明几何地标对应真实工艺位置。
4. **补 v2 mock FULL 与故障注入：** 不只覆盖成功 happy path，覆盖重试、取消、撤权、断流和恢复。
5. **完成 M0 与实物末端验收：** 独立反馈、IO 反证、刃面/TCP、捕获窗口、三工具适用率及剪断第二判据；真机动作/SetIO 仍需授权。
6. **按结果决定入口切换：** 比较误许可率、采摘成功、损伤、周期与恢复成本；通过后再扩大场景与性能优化。

暂不优先：为“更先进”而增加规划器、替换检测网络、扩大包数、引入完整稠密重建或路线优化。只有基准证明瓶颈在对应环节时再做。机械容差和刀具几何能解决的问题，不应全部转嫁给视觉毫米级精度。

### R17. 后续完善跟踪表

以下项目保持待办；不得因本节写入而视为实现完成。完成时补充证据路径、运行版本和剩余限制。

- [ ] 旧栈线程收尾及在途 shutdown 修复与回归。
- [ ] 单一运动仲裁、IMU/Servo 撤权与其他运动源互斥验证。
- [ ] 撤权测试解除 skip；正常退出与强制清理口径分离。
- [ ] v1/v2 同数据集、同真值、同配置条件的基准。
- [ ] 袋底、袋颈、扎口和真实剪切对象的定义及偏移实测。
- [ ] 折角/扁袋/部分遮挡/大倾角的几何模型失配测试。
- [ ] KF 身份保持、重现和锁定集合边界测试。
- [ ] 跨视点相关误差与 95% 覆盖率校准。
- [ ] RSS 参数来源、统计口径及相关性说明。
- [ ] 果顶代理与非轴对称末端通行包络验证。
- [ ] 模型保存期、接触观测期、复测结果期分别定义并测试。
- [ ] 复测通过状态与 revision/目标变化/周期绑定审查。
- [ ] 场景挖空、ACM 恢复、携果回程和动态障碍验证。
- [ ] v2 mock FULL 与失败/恢复/取消全链系统测。
- [ ] 三末端 M0 标定、独立 DI、IO 反证与剪断第二判据。
- [ ] bond 实际故障检测与诊断运行环境一致性复核。
- [ ] 依据验收结果同步原方案阶段状态和三份活文档。
- [ ] v1 SIGINT 深修：对齐 nav2 SimpleActionServer **无 detach** 模式（std::async future＋协作停止标志＋有界轮询；超时强制终止 goal 仍不 detach），或线程捕获 `shared_from_this()` 续命；配合 Jazzy `Context::add_pre_shutdown_callback`（rcl_shutdown 前、信号线程、不得抛异常）提前收线程。2026-10-08 已修三处定点（timedjoin 绝对时刻 P0、survey 终局与 ACM 分离线程 try/catch），detach 后 UAF 根治仍开放。
- [ ] v1 P2 组：`execution_started` 置位过早致 FAILED/SKIPPED_UNREACHABLE 分级漂移并误置恢复锁；`_wait_result` 超时取消后不等动作退场（下一 goal 被 REJECT 误记 full_failed）；EventHold 暂存命令被不同事件覆盖；MoveTo/Survey 不占 `running_` 的参数重写窄竞态；硬编码 group/tcp/world/帧名不走 config。
- [ ] v2 P2 组：moveit backend `active_goal_` 赋值窗口（goal 发出→handle 就绪最长 2 s）与 `mgi_mutex_` 阻塞锁让 stop 排队（公开 stop 用 try_to_lock 拿不到即静默跳过）；`stop_grace` 超时后仅 ERROR 不 latch recovery；`selection.cpp` height NaN 破坏 std::sort 严格弱序（UB）；v1 `peach_common/lifecycle.py` bond 无 `on_broken` 回调（bondpy 支持 `on_broken=` 参数，Timer 线程触发需线程安全）且 `start()` 不在守卫内；v1 视觉 slop 同步失败无计数、「从未收帧」诊断显示 healthy。
- [ ] 三活文档（architecture/io/testing）与 `peach2_system_tests/README.md` 的 33/191 旧数字及 §3.3 相关表述，由所有者会话在并行轮收口时同轮更正（`REFACTORING.md` 同源数字已随该文件 2026-10-10 删除消失）。

### R18. 源码与外部依据索引

相对路径均指本工作区；代码后续变化时应重核结论，行号不作为永久契约。

- v1 调度：`src/peach/peach_harvester/peach_harvester/supervisor/harvest_fsm.py` 及同目录调度实现；源码地图 `.agents/skills/peach/harvest-flow.md`。
- v1 感知/重建：`src/peach/peach_harvester/peach_harvester/vision/`；预算 `common/tool_budget.py`；轴向下限 `target_reconstruction/refine.py`；配置 `src/peach/peach_harvester/config/target_reconstruction.yaml`。
- v1 收尾：`src/peach/peach_arm/src/manipulation_skills_node.cpp::closeMotionOutputAndCancel`、`cycle.cpp::onActionAccepted`；本机 `/usr/include/c++/13/bits/std_thread.h` 和 `pthread_tryjoin_np(3)` 核对线程语义。
- v1 旁路运动：`src/imu_follow/imu_follow/follow_node.py::_publish_twist`；测试 `src/peach/peach_system_tests/test/test_e1_supervisor_chain.py`。
- v2 感知：`src/peach2/peach2_perception/README.md` 与 `peach2_perception/` 节点/纯核实现。
- v2 算法：`src/peach2/peach2_core/peach2_core/{tracker,landmarks3d,fusion,budget,swing}.py`。
- v2 模型/许可：`src/peach2/peach2_target_model/peach2_target_model/{store,decision,target_model_node}.py`、`config/target_model.yaml`。
- v2 调度：`src/peach2/peach2_task/trees/harvest_batch.xml`、`src/peach2/peach2_task/src/core/selection.cpp`。
- v2 场景/运动：`src/peach2/peach2_scene/peach2_scene/scene_core.py`；`src/peach2/peach2_manipulation/src/{command_gate,harvest_cycle,moveit_motion_backend}.cpp`。
- v2 工具/标定：`src/peach2/peach2_end_effector/`、`src/peach2/peach2_calibration/results/*.yaml`。
- 历史证据：`docs/testing-log.md` 2026-09-29/30 记录；本方案 §14 里程碑；本地 `build/` 测试 XML 仅为历史运行工件，不提交且不代表当前复测。
- [BehaviorTree.CPP 异步节点](https://www.behaviortree.dev/docs/guides/asynchronous_nodes/)：异步返回与 halt 责任；[Sequence 语义](https://behaviortree.dev/docs/nodes-library/SequenceNode/)：不同序列控制节点的重检查语义。网页版本可能演进，实际编码须对照所安装版本。
- [MoveIt Task Constructor 教程](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)：stage 与 PlanningScene 修改的设计参考，不证明本仓 v2 已实现 MTC。
- 2026-10-08 官方资料核验明细（补齐上轮两条失败访问；分级、引文、逻辑链与未核项如下，本条目替换上轮的失败访问注记）。

**方法与证据分级。** 核验手段三种：官方页抓取（WebFetch 转述）、HTTP 状态码（curl）、本地安装件直读（apt 包版本与 `/opt/ros/jazzy` 头文件）。级别：**A** = 官方页逐字引文 / 注册表或本地安装件直读；**B** = 官方页成功抓取的转述；**C** = 官方页不可达时多方独立来源交叉；**D** = 未核（列关闭条件）。上轮两条失败访问（Jazzy Testing、Nav2 lifecycle）本轮均已成功抓取并直读。

**一、确定项（A/B 级）**

| # | 对象（方案出处） | 官方源 · 手段 | 关键事实 / 引文 | 级 | 逻辑与影响 |
|---|---|---|---|---|---|
| V1 | launch_testing 结构（§13） | [Testing Main](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html) / [Integration](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Integration.html) · 抓取 | 四要素：`generate_test_description` / `ReadyToTest` / 运行中 TestCase / `@post_shutdown_test` + `assertExitCodes`；`add_launch_test` 入 CTest；`run_test_isolated.py`（ament_cmake_ros）为每个测试配独立 `ROS_DOMAIN_ID` | B | 教程明文"different tests shouldn't communicate"→ §13 的 isolated domain 非本仓发明；本仓 `add_ros_isolated_launch_test` 即该机制的一行包装 |
| V2 | `bond_timeout` 4.0 s＝Nav2 默认（§3.3 / §12 表 / R13） | [Nav2 Jazzy 配置页](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/)（rolling 版同值）· 抓取；本地 nav2_lifecycle_manager 1.3.13 头文件 · 直读 | Jazzy 与 rolling 两版页面均：默认 **4.0 s**、0 禁用、建议 >0.3 s；`bond_respawn_max_duration` 默认 10.0 s；本地头文件 `bond_respawn_max_duration_{10s}` 一致 | A（三源一致） | 「4.0＝Nav2 默认」成立。逻辑链：v2 launch 取 4.0＝对齐官方默认；v1 旧栈取 0＝**禁用** bond、死检由 HeartbeatWatchdog 承担——两代是两个不同事实，不可互推（R13「接线 / 默认值 / 验收三分离」继续有效） |
| V3 | 进程死→整栈降级（§3.3） | 同 V2 | 管理器对每个托管节点建 bond，失联即 "transition down all nodes" | B | 「缺心跳＝故障」的官方同构；仍非功能安全（分层见 §10，与 V12 互证） |
| V4 | mock＝命令镜像（v1 方案 / AGENTS） | [mock_components userdoc](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) · 抓取 | "provide ideal behavior by mirroring commands to their states"；用途＝无硬件测管线；`calculate_dynamics` 等默认 false | B | v1「mock＝GenericSystem + 标准 JTC」的依据；mock 不证动力学 / 碰撞 / 急停——与「mock 不证明安全」同源 |
| V5 | C++17（全仓 MUST） | [Jazzy Code Style 页](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html) · 抓取 | 逐字："Jazzy targets C++17" | A | 出处页是 **Code Style**，Developer Guide 只链接它——引用时引对页（本轮已按此修正 v1 plans 附录 A） |
| V6 | 布局 / 测试口径（v1 plans 编制依据） | Jazzy Developer Guide · 抓取 | "All packages should have some level of system, integration, and/or unit tests"；单测用 mock；行覆盖目标 >95%；include/src 标准树 | B | 95% 是**核心库**目标；本仓应用栈按 REP-2004 QL3～QL2 不照搬——与 AGENTS 的区分一致 |
| V7 | configure/activate 资源语义（§3.3 托管顺序） | [node_lifecycle 设计文](https://design.ros2.org/articles/node_lifecycle.html) · 抓取 | onConfigure＝生命周期级资源（话题/订阅/持久内存）；onActivate＝运行期资源（如硬件访问）；inactive 节点不获执行时间 | B | 支撑「configure 建资源、activate 才进实时路径」与拆栈反向顺序 |
| V8 | BT 异步 / halt（§8） | [BT.CPP 异步节点页](https://www.behaviortree.dev/docs/guides/asynchronous_nodes/) · 抓取 | 引擎单线程："If any tick() method is blocking, the entire flow of execution will be blocked"；异步动作尽快返 RUNNING；halt() 由开发者实现；默认不鼓励自起线程，真异步推荐 ROS action | B | §8「BT 叶子非阻塞 + 长动作走 action」的直接依据；网页版与安装版差异风险见 U4 |
| V9 | Sequence 族差异（R18 原条目复验） | [SequenceNode 页](https://behaviortree.dev/docs/nodes-library/SequenceNode/) · 抓取 | Sequence：FAILURE 重启、RUNNING 同子节点续；ReactiveSequence：RUNNING 也重启（每 tick 重检前置并 halt 在途子节点）；SequenceWithMemory：记住已完成兄弟 | B | 选哪种序列＝恢复语义的分叉点；主树编码前对照此表 |
| V10 | MTC stage 分类（§6.2、R18 原条目复验） | [MTC 教程](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)（链接见上）· 抓取 | generators 双向 / propagators 单向 / connectors 桥接；`ModifyPlanningScene`（allowCollisions / attachObject）；serial 容器端到端才算解 | B | 支撑 §6.2 把 MTC 降为「候选演进」的成本与能力边界判断 |
| V11 | P-stop 恢复语义（§10.2、v1 §10、AGENTS） | [UR robot_state_helper](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/robot_state_helper.html) · 抓取 | 逐字："A robot's protective stop or emergency stop is only pausing the running program"；安全路径＝stop_program 后重发，非 resume | A | 「故障后禁止 resume 原轨迹」的直接出处；三份文档口径一致 |
| V12 | Collision Monitor 非安全额定（§10 对照） | [Collision Monitor 页](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)（Jazzy 另有 docs.ros.org 镜像）· 抓取 | 逐字："this does not provide hard real-time safety certifications"（CPU 级实现、任意传感器）；功能＝虚拟区域形状触发降速 / 停 | A | 「应用护栏≠功能安全」分层的官方措辞；本仓 ContactMonitor 同理不得称 e-stop |
| V13 | 参数单源 GPL（§12） | [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) README · 抓取 | YAML→struct + `ParamListener`；声明 / 取值 / 校验 / 动态更新自动生成；自动生成参数文档 | B | §12 C++ 包用 GPL 的依据；Python 侧按决策 0024 yaml 直读，两轨并存与 AGENTS 偏离表一致 |
| V14 | admittance_controller Jazzy 可用（§17） | 本地 apt · 直读 | `ros-jazzy-admittance-controller` 4.42.1 已安装 | A | "Jazzy 已装"由选型判断变装机事实；是否真用导纳仍属 §16 开放问题，不因可用而启用 |
| V15 | JetPack 6.x 宿主 Ubuntu 22.04 → 须容器（§3.4.5） | [NVIDIA JetPack 页](https://developer.nvidia.com/embedded/jetpack) · 抓取 + 多源交叉 | JetPack 6.0＝Jetson Linux 36.3＝内核 5.15 + **Ubuntu 22.04 rootfs**，支持全部 Orin 模块含 Orin NX；JetPack 7＝内核 6.8 + Ubuntu 24.04（模块级支持未明，见 U5） | B/C | 逻辑链：Jazzy 官方＝24.04 ≠ 22.04 rootfs → 宿主原生装不上 Jazzy apt → 容器 / 源编是**必要路径**而非可选优化；§18「容器名≠GPU 兼容」告诫仍前置 |
| V16 | `ros:jazzy` arm64 镜像存在（§3.4.5） | Docker Hub registry API · 直读 | `jazzy` 及 ros-core / base / perception 各变体 architectures 均含 **arm64**（amd64+arm64 双架构） | A | 容器路线的镜像前提成立；CUDA/TensorRT 直通与 GPU 加速仍按 §18 部署前置验证实测，不由镜像存在性推出 |
| V17 | ISO/IEC 版次年份（§10 / §17 / v1·v2 设计书附录） | iso.org 目录页 · 直读或交叉 | ISO 13850:2015 第 3 版（目录页直读；2026-09 起 90.92 修订中）；ISO 10218-1/-2:2025（2025-02 第 3 版，替换 2011）；ISO 18497-1…4:2024（四分册首版，共同替换 18497:2018）；IEC 60204-1 现行 6.1 版（2016+A1:2021 CSV） | A（13850）/ C（其余，见 U3/U8） | 四份文档年份引用全部成立；**IEC 60204-1 无 2024/2026 新版**，遇「2024 版」说法须回查（第 7 版预计 2026–27 时间窗） |

**二、口径勘定（结论成立，引用方式需限定）**

- **Pilz「不避障、碰了整条拒」**：MoveIt 官方 how-to 页**无此逐字表述**（该页只述命令集、限速缩放与 sequence 失败全拒）。语义成立的证据链：Pilz＝确定性轨迹生成（PTP 梯形速度、路径固定、无绕障自由度）→ 碰撞检查只能过 / 拒（ROS wiki Pilz 教程："If there would be any collisions, the robot won't move"）→ 已知缺陷另有两项：attached body 碰撞检查不充分（moveit GitHub issue）、ACM 更新不生效（StackExchange 案例）。**引用规则**：本仓文档按语义引用并注来源，不写成「MoveIt 官方原文」。
- **Nav2 文档站 URL 迁移**：`docs.nav2.org/tutorials/...` 旧路径 404；现行结构＝`docs.nav2.org/{distro}/configuration_and_development/configuration_guide/core_servers/...`。本轮新增链接一律指 Jazzy 版页面；AGENTS 内指向旧路径的链接下次触及时同轮更新（不属本轮四文档范围）。

**三、不确定与未核项（D 级，含关闭条件）**

| # | 项 | 现状与风险 | 关闭条件 |
|---|---|---|---|
| U1 | Jazzy Testing CLI 子页（colcon test 运行 / 报告细节） | 未抓取；本方案引用点已被 Main / Integration 页覆盖 | 需引用 colcon test 细节时补抓 |
| U2 | REP-103/144/149/2004 全文 | 仅编号引用（编号+主题稳定，风险低） | 引用具体条款前读全文 |
| U3 | ISO/IEC 标准正文 | 付费墙；10218 / 18497 / 60204-1 年份靠目录页与多源交叉（C 级） | 购买文本或以官方摘要为限 |
| U4 | BT.CPP 网页文档 vs 本机安装版差异 | 网页为当前版；apt 安装版行为以安装版为准 | 编码前对照所装版本头文件 / CHANGELOG |
| U5 | JetPack 7（内核 6.8 / Ubuntu 24.04）对 **Orin NX 模块级**支持 | NVIDIA 页面只说「Orin 平台」未列模块清单；若支持，§3.4.5 容器约束可能从「必要」降为「可选」 | 查 JetPack 7 支持矩阵按模块确认 + NX 原生 Jazzy 实测 |
| ~~U6~~ **已关闭（2026-10-08）** | 图源对应已实锤：`peach_v2_hw_arch.drawio`（旧单页）已删除，替换为 [images/peach_v2_arch.drawio](images/peach_v2_arch.drawio)（双页：页 1 硬件 / 页 2 软件）；两张 PNG 由 **drawio 31.7 CLI 从该源直接导出**（`drawio -x -f png -s 2 --page-index 1/2`），时间戳与图源一致；§3.4.1 维护注记已更新导出命令 | 无遗留 |
| U7 | OSU apple-harvest 仓库 URL | 检索未找到稳定公开仓库；名称级引用保留 | 需引代码细节时向作者确认获取途径 |
| U8 | iso.org/standard/73933.html（ISO 10218-1 目录页） | 本轮仍超时；2025 版年份靠三方交叉（C 级） | iso.org 恢复后直读目录页 |

**四、对四份文档的影响。** 核验未发现事实错误 → 正文论断**零改动**；全部改动在引用层：v2 目录 §8 对齐、§17 与 plans v2 附录 A 补已核 URL、v1 §16 补链与 bond 行注记、plans v2 升版 V2.2 同步 §3.4 / §18 口径、plans v1 附录 A 年份勘误、图片断链修复（U6 相关）。上轮失败访问注记由本明细替换；后续实现前如需引用 U1–U8 对应内容，先按关闭条件补核再写。

**五、源码核对明细（2026-10-08 工作树）。** 方法：三路只读探索（路径+行号证据）＋ 承重文件亲读复核 ＋ 本仓静态计数与 `build/` 历史 XUnit 交叉。级别沿用 A–D（A＝亲读源码/直接工件，B＝行级证据转述，C＝间接工件，D＝未核）。注意：工作树正被并行实施轮修改（peach2_bringup / system_tests / task、`test_decision.py` / `test_store.py`），本表为 2026-10-08 午后时点快照，行号不作为永久契约。

| # | 论断（出处） | 证据 | 级 | 结论与逻辑 |
|---|---|---|---|---|
| S1 | 十二包清单（§3.2 / §14） | `ls src/peach2/` 恰好 12 包，无多无少 | A | 成立 |
| S2 | 命令门条件（§10.2 / plans v2 §7.4） | `command_gate.cpp`：Active（L83）、cancel（L100）、robot_status 年龄 0.3 s（L62，hpp/yaml 同 0.3）、enables 链（L48-50）、心跳 3.0 s（L42）、阶段许可 switch（L111-135）；`motion_possible` 仅新轨迹前查（L75；hpp:71 注释明示）；关门善后链＝abort_probe→cancel_active_goal+stop（moveit_motion_backend L242-247）→abort_safe（harvest_cycle L404-411） | B | 逻辑成立；**两处表述偏差已改**：v2 §10.2 原写 `¬protective_stop`（实现无独立字段，并入 `in_error`，L71）与 motion_possible 列为持续条件（实现仅新轨迹前） |
| S3 | RSS 公式与标定门（§5.1 / §5.2） | `budget.py` L69-74：`m_r = 0.5·d_inner − 0.5·d95 − c_wall − rss(s_lat, L·sinθ95, e_tcp, e_handeye, e_runout, A)`、`m_a = w_capture − rss(s_ax, e_blade, e_robot, A)` 与公式逐项一致；L77-89 `status != CALIBRATED → cut_ok=false / calibration_pending`；GetDecision 为同步 service（`target_model_node.py` L135/315，docstring 注明锁外计算） | B | 成立 |
| S4 | M0 卡点（§14 M0 行） | `peach2_calibration/results/{shear_v1,bite_shear_v1,adaptive_shear_v1}.yaml` 三档案全部 `status: design_reference`，头注释明示 cut_allowed 恒 false 直至 M0 台架改 calibrated | B | 成立——「M0 未过」判断有据 |
| S5 | gtest 规模（§14 M1 验收表） | 静态计数（亲核）：manipulation 111 / task 114 / end_effector 56＝全 peach2 281；09-30 XUnit manipulation 111 tests 0 failures；「31 skip＝cppcheck」与工件精确吻合 | A | **「191」无法复现**：静态、XUnit、testing-log 全文均无 191；已改表为 111。testing-log 亦无该轮运行记录（证据只在 build/ 工件）。同源数字在 `peach2_system_tests/README.md` 与 `REFACTORING.md` L725，属源码侧，待所有者会话同轮更正 |
| S6 | launch_testing 规模（§14 M1 验收表） | 静态（亲核）：26＝10（bringup_mock）+7（plan_only_chain）+9（pregrasp_mock_exec）；09-30 XUnit 三文件 26 tests 0 fail 0 error；10-08 新增 `test_frozen_camera_bag` pytest 2 例（XUnit 11:14，先挂后绿） | A | **「33」无法复现**（最接近拼法 26＋当日 lint＝32，仍差 1）；已改表为 26；「全绿」结论本身有 XUnit 支持。域 95–97 归 peach2（v1 系统测用 89–94，归属无写错） |
| S7 | 八进程划分（§3.3 表） | `peach2_system.launch.py` 实际：相机可选 include→aubo bringup→**perception 单 LifecycleNode**（无 container/infer 拆分）→target_model→scene（可选）→manipulation→task→lifecycle_manager；observability 不随系统 launch 启动（包内自带 launch）；托管名单/顺序 5 节点与 bond_timeout 4.0 默认开、preflight 拒残留均属实 | B | **设计表≠实现**：container/infer 拆分与 observability 进程属目标架构；已在 §3.3 表后加「实现现状」注记 |
| S8 | O1 接口字段（plans v2 §8） | `peach2_interfaces` 逐字段在册：`FailureCode.msg:21 NECK_REMEASURE_PENDING=28`、`TargetObservation` camera_pose/swing_known、`TargetObservationArray.locked_target_ids`、`TargetModel.branch_direction*`、`HarvestResult.plan_only/recovery_required`、`ToolState.feedback` 三态、`srv/BeginScene` | B | 成立；细节：plan_only 是 HarvestResult 字段，action 侧为 `MODE_PREGRASP_ONLY/MODE_FULL` 常量 |
| S9 | 已落地优化抽查（plans v2 §8） | O3：`motion_backend.cpp` L15-51 `finalize_plan` 塌缩、残差修正显式 `collapse_at_goal=false`（harvest_cycle L578-580）；O4：`trajectory_reverse.cpp` L23-29 点序倒排+速度取负+**加速度原样**；O10：`decision.py` L168-169 摆幅未知拒绝（代码缺省 False，`target_model.yaml:43` true 生效）；O13：节点实为 `NeckRemeasurePending` 条件＋`ObserveTarget neck_remeasure` | B | 全部成立；O13 名称与 O10 缺省值两处表述已按实现精化（plans v2 §8/§7） |
| S10 | BT 非阻塞与复测等待（§8 / R14） | `harvest_batch.xml` 头注 50 Hz tickOnce；`async_leaves.hpp` `AsyncActionLeaf : StatefulActionNode`（onStart 发／onRunning 轮询／onHalted 取消）；`_execute_observe` 条件变量等 revision+converged、`_remeasure` 等近距帧中位判 PASSED/MISMATCH | B | 成立 |
| S11 | v1 现行方案论断（v1 现行 §14 等） | `tool_budget.py` L49-66 径向/轴向**线性相加**（非 RSS），代入缺省值轴向余量＝−2 mm 恒负与文档一致（L97-115 自认结构性缺口）；`harvest_fsm.py` 零 rclpy；`stages/cycle/grasp_task.cpp` 在；v1 launch `bond_timeout` 默认 0.0（L228-230）；`follow_node.py::\`_publish_twist\`` 在；批流从不发 PREVIEW（executor_node 仅调试占位；cycle.cpp L324 仅 PREVIEW 模式写绑定） | B | v1 两份文档 7/7 论断成立，**零改动** |
| S12 | §18.16 P1/P2 进行状态 | 工作树 `test_decision.py`（+2：重复查询不续期；future/NaN/inf 拒许可）与 `test_store.py`（+5：倒序/重复帧、延迟 fusion、锚点漂移、旧 swing 过期、复测撤销 PASSED）与 P1/P2 描述逐条吻合 | B | 实施轮确在进行中；「未勾选」状态如实，未被本表改变 |

**六、源码核对影响汇总。** 12 项核对：**10 项成立、2 项数字失实已改**（S5 gtest 191→111、S6 launch_testing 33→26）、1 项设计-实现差距已加注（S7 八进程）、4 处命名/条件表述已精化（S2 两处、S9 两处）；v1 两份文档零改动（论断全成立）。src/ 侧同源数字（`peach2_system_tests/README.md`、`REFACTORING.md` L725）不在本轮文档范围，建议所有者会话同轮更正。本表为快照：实施轮合入后（P1–P5、frozen_camera_bag 等）应重核 S5/S6/S12。

**七、修复轮实施记录与权威依据（2026-10-08 午后）。** 按 P0/P1 全修范围实施 §18.16 P1–P4、`peach2_core` 三项算法加固与 v1 四处定点修；四包 `colcon test` 全绿（peach2_core pytest 80 / peach2_target_model pytest 80 / peach2_manipulation gtest **114**=111+3 新增 / peach_arm 构建与测试通过）。每个修复点对齐的权威源：

- [pthread_timedjoin_np(3)](https://man7.org/linux/man-pages/man3/pthread_tryjoin_np.3.html)：abstime 为 **CLOCK_REALTIME 绝对时刻**（官方示例 `clock_gettime`+偏移；glibc 内部按 MONOTONIC 换算、不受墙钟跳变影响）——修 v1「`{2,0}` 相对值致 2 s 有界回收恒为 0 s」的 P0。
- [cppreference std::thread](https://en.cppreference.com/w/cpp/thread/thread.html)："if it terminates by throwing an exception, **std::terminate is called**"——v1 executeSurvey 终局与 on_activate ACM 分离线程补 try/catch 的标准依据。
- rclcpp_action `client.hpp`（本机 Jazzy L413-416 原文）："Dropping the shared pointer to the goal handle **will not cancel** the goal… you must explicitly call async_cancel_goal"；[nav2 bt_action_node](https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp) 发送超时即 `async_cancel_all_goals()`（有界等待）——`moveit_motion_backend` 僵尸 goal 修复同款（本客户端单目标在途，cancel-all 即精确）。
- [Autoware control_command_gate](https://github.com/autowarefoundation/autoware_universe/blob/main/control/autoware_control_command_gate/src/command/subscription.cpp)：订阅入口逐字段 `std::isfinite` **整条拒绝**非有限命令（CHANGELOG #13239：clamp 拦不住 NaN，isfinite 必须在最外层）；源超时切内置停止源——`command_gate` 三路 fail-closed 与 `decision.py` 非有限时间拒绝的同构模式。
- numpy `np.linalg.cholesky` / `scipy.linalg.cho_factor`：非正定直接 LinAlgError——tracker 门控距离与增益求解改 Cholesky（`np.linalg.solve` 只查奇异不查正定，不定但可逆的 S 会解出**负** Mahalanobis 距离、反而通过任意门限）；配 Joseph form 保持 PSD。
- nav2 [SimpleActionServer](https://github.com/ros-navigation/navigation2/blob/1.1.15/nav2_util/include/nav2_util/simple_action_server.hpp)（**全程无 detach**：std::async future＋协作停止标志＋有界轮询，超时强制终止 goal 仍不 detach）与 Jazzy `Context::add_pre_shutdown_callback`（`context.hpp` L276-279：rcl_shutdown **之前**调用、信号线程异步执行、不得抛异常）——v1 SIGINT 深修的目标模式，已列入 R17。

加固与勘定：fusion 视点尺度下限 1 mm（精度权重上限 1e6，防单视点劫持 Huber 均值）；`_regularized` 特征值裁剪保证下游全正定；landmarks3d d95 空选集回退全云（更大 d95 只缩余量，fail-safe 方向）；`fuse_job` 的 min_views 改传 params（去硬编码，部署值 1 无行为差）。边界 gtest 教训：等值边界须用二进制可精确表示值（0.5/4.0 s）——`10.0+0.3` 的双精度差是 0.3000000000000007，会把「等值仍开门」的断言写歪。

---

## 源码架构图重绘记录（2026-10-09）

本次按工作区源码静态核对后绘制四张独立架构图。硬件图描述源码中可确认的机械结构、驱动通信和安全边界，不充当现场电气接线图；软件图描述主要职责、算法、接口及启动范围，不充当完整运行时 ROS graph。所有图均输出 PNG（插文档）、SVG（缩放查看）、PDF（打印）与 DOT（编辑）格式，重绘命令：`python3 scripts/render_peach_architecture.py`，依赖系统 Graphviz 与 Noto Sans CJK SC 字体。

- [v1 硬件架构图](images/peach_v1_hardware.svg)：固定座、六轴机械臂、腕载相机、三种可换末端、柜体 IO、安全回路与条件 IMU 支路。
- [v1 软件架构图](images/peach_v1_software.svg)：brain 同进程三节点、显式 FSM、ICP/TSDF 重建、peach_arm 运动、场景快照、条件 Servo 支路及 8090 观测。
- [v2 硬件架构图](images/peach_v2_hardware.svg)：复用同一硬件；独立反馈、力传感与 IMU 的接入状态以虚线说明，不假定新增器件已经落地。
- [v2 软件架构图](images/peach_v2_software.svg)：BT 任务、单帧感知、多视点 Huber 模型融合、许可预算、分层场景、单一命令门、进程内末端插件与只读驱动。

源码依据（图中模块按以下实现核对，后续重绘应重查）：

- v1 整栈：`src/peach/peach_bringup/launch/harvest_system.launch.py`；brain：`src/peach/peach_harvester/peach_harvester/brain.py`；调度：`src/peach/peach_harvester/peach_harvester/supervisor/executor_node.py`、`harvest_fsm.py`。
- v1 算法：`src/peach/peach_harvester/peach_harvester/vision/scene_perception/inference.py`、`vision/target_reconstruction/target_reconstruction_node.py`；执行：`src/peach/peach_arm/src/manipulation_skills_node.cpp`。
- v2 整栈：`src/peach2/peach2_bringup/launch/peach2_system.launch.py`；任务树：`src/peach2/peach2_task/trees/harvest_batch.xml`。
- v2 感知与模型：`src/peach2/peach2_perception/peach2_perception/perception_node.py`、`detector.py`、`segmenter.py`；`src/peach2/peach2_target_model/peach2_target_model/{store,decision,target_model_node}.py`；`src/peach2/peach2_core/peach2_core/fusion.py`。
- v2 场景与执行：`src/peach2/peach2_scene/peach2_scene/{scene_node,scene_core}.py`；`src/peach2/peach2_manipulation/src/{manipulation_node,harvest_cycle,command_gate}.cpp`、`moveit_motion_backend.hpp`。
- v2 末端：`src/peach2/peach2_end_effector/plugins.xml`、`src/{aubo_io_backend,cutter_end_effector,tool_state_machine}.cpp`；独立观测入口：`src/peach2/peach2_observability/launch/observability.launch.py`，端口依据其 `config`。
- 共用硬件：`src/aubo_description/urdf/aubo_e5.urdf.xacro`、`components/camera.xacro`、三种 `tcp_*` xacro；`src/aubo_e5_bringup/launch/bringup.launch.py`。相机网格注释为 PS800-E1，不能据此推定所有部署使用完全相同相机。

关键勘定：v1 `_include` 没有 scoped GroupAction，v2 使用 scoped GroupAction；v1 serial_imu 由 imu_enabled 控制，imu_follow 由 adaptive 工具条件控制；v2 整栈未 include 两者；v2 observability 有独立启动文件但整栈未 include；v2 scene 与 manipulation 分别写环境障碍和 `peach_bag_*`；力传感后端目前返回 false。三工具标定与真机产品验收边界沿用源码/已有记录，图的生成不新增通过结论。

验证：四张图已成功导出四种格式并逐张目视检查中文、图例、节点范围与安全链路；未运行 ROS、未运动或 SetIO。旧图保留，避免破坏既有引用。

### 架构图表达修订：高层视图与接口视图分开（2026-10-09）

用户澄清交付需要的是架构图。前一轮将节点、动作、话题和算法步骤同时展开，形成了过密的连接图。现已重绘并覆盖四张主图及其对应导出文件，上一小节的源码依据仍适用，当前表达以本小节为准。

本次参考官方架构资料：

- [Autoware Architecture overview](https://docs.autoware.org/main/design/autoware-architecture-v1/)：以职责分层降低模块耦合，高层架构与 Node diagram 单独呈现。
- [MoveIt System Architecture / Concepts](https://moveit.ai/documentation/concepts/)：高层图组织核心能力与用户、机器人接口，具体内部细节在下级概念文档展开。该页面含历史 API 名称，本次只参考图的组织方式，不作为 Jazzy API 实现依据。
- [ros2_control Jazzy Architecture](https://control.ros.org/jazzy/doc/getting_started/getting_started.html#architecture)：框架架构与内部 UML 类图分开，控制器管理、资源管理与硬件抽象具有明确职责边界。

由这些参考归纳并采用的绘图原则（本项目的表达选择，不宣称官方规定唯一图法）：一张图固定一个抽象层级；模块先按职责归属，再用包名定位实现；同一层并列组件保持相同粒度；跨层关系仅保留关键主链；横向支撑与独立安全域另列；运行时节点通信、完整话题清单、阶段流程不混入主架构图。

当前软件主图：任务编排 → 感知与世界模型 → 作业执行 → 机器人基础设施 → 硬件抽象与驱动，附独立安全域与跨层的任务执行、观测输入关系。层次表示责任归属，不表示所有请求必须逐层转发。v1/v2 使用相同布局，以比较纯 FSM 与 BT、ICP/TSDF 与多视地标融合、臂内工具逻辑与末端插件边界。

当前硬件主图：计算与操作、通信与设备接口、控制与采集、机械与执行四个职责域，单独表达功能安全边界。两版本机械与驱动基本复用，差异注释反映 IMU/Servo 接入范围，不臆造 v2 新硬件。该图不表示具体引脚、电源、总线拓扑或安全认证等级；这些需电气图与现场依据。

输出更新：四张图均提供 SVG / PNG / PDF / DOT，另增加同名 `.drawio` 可编辑文件。`render_peach_architecture.py` 以固定布局生成一致的版本视图；修改模块或边界前应重查上小节列出的源码。四张渲染图已逐张目视检查，SVG、PNG 和 drawio 文件结构检查通过。Graphviz HTML 标签初次导出遇到空标题语法错误，已修正并全部重新导出成功。

### 架构图视觉优化（2026-10-09）

按用户要求美化四张主图：采用统一浅色背景、职责色带、独立模块卡片和圆角边界；区分层标题、模块职责与源码定位三档文字；增加侧栏留白，改善主链和跨层连线阅读；安全域保持红色。v1/v2 使用一致模块位置与配色，内容依据与系统边界沿用上一轮源码核对。PNG 导出提升到 180 dpi，SVG/PDF 保留矢量能力，DOT/drawio 同步重生成。逐图检查渲染，文件结构与差异空白检查通过；drawio 已检查 XML 结构，未在 diagrams.net 中打开验证。

### NX 与 RK3588 硬件部署角色补回（2026-10-09）

用户指出硬件图遗漏 NX 与 RK3588。原因是绘制职责分域时把具体计算设备合并成“上位机”，丢失了既有 §3.4 的部署边界；本次已在 v1/v2 硬件主图恢复 **Jetson Orin NX 16GB** 现场主控及 **RK3588 预留分流板**，并单列开发/测试机。

证据：本方案 §3.4、`docs/peach_v1_现行终版方案.md` 硬件部署段、`_archive/code_cleanup_2026-10-12/plans/peach_v1_套袋桃采摘机器人项目设计书.md` §5，以及 `_archive/code_cleanup_2026-10-12/plans/peach_v2_套袋桃采摘机器人项目设计书.md` 硬件段均明确 NX 默认承载整机栈，RK3588 是压力过大时的前端采集/预处理预案，预留未实装。源码的整栈 launch 确认应用能力、设备接口和命令门，但没有证明 NX 或 RK3588 已完成部署验收，因此图注明“部署目标 / 验收待完成”，不把预案画成当前已运行事实。

图示恢复默认相机直接接入 NX 的说明；RK3588 可选承担采集、时间同步、深度与 ROI 预处理，通过 GbE 向 NX 提供处理后的 RGB-D。RK3588 不直接控制机械臂或工具 IO，v2 命令门与运动决策保留在 NX。所有硬件图格式（SVG/PNG/PDF/DOT/drawio）同步导出并检查。

### 官方与论文系统图参考后的拓扑优化（2026-10-09）

本轮继续查阅官方与原始论文，按图的语义重新组织主图，而非只改色彩：

- NIST《World Model Data Specifications as Metrics for Sensory Processing》Figure 1：[原文](https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=822563)。感知处理、世界模型与行为生成分开，传感与执行跨越内部/外部边界。借鉴职责与闭环表达，不把 Peach 宣称为完整 4D/RCS 实现。
- Zhang 等《Algorithm Design and Integration for a Robotic Apple Harvesting System》：[原文](https://arxiv.org/abs/2203.00582)。Figure 1 是物理系统，Figure 2 是采摘周期算法流程图；本轮借鉴感知、规划、控制的功能划分，不将论文流程图等同于软件架构图。
- Parsa 等《Modular autonomous strawberry picking robotic system》，Journal of Field Robotics，Figure 3：[原文](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22229)。正文与图注明确硬件/软件系统框图、计算设备的分工及 Ethernet 连接；借鉴部署边界与子系统连接表达。
- NVIDIA《Introducing Jetson Xavier NX》Figure 2：[官方页面](https://developer.nvidia.com/blog/jetson-xavier-nx-the-worlds-smallest-ai-supercomputer/)。仅参考“功能块与接口”画法；Xavier 与项目 Orin NX 不是同一型号，不迁移该页面的芯片规格到本图。
- Autoware / MoveIt / ros2_control 官方参考沿用上节，其高层架构、节点图、内部类图保持区别。

证据范围：通过网页/论文文本核查图注与相关说明，苹果论文 PDF 的 web 截图请求返回了页面引用；额外本地下载因超时未形成可用 PDF，未用不完整文件生成参考图，也未声称所有参考原图均已成功在本地查看。图形表达为原创绘制，未直接复用论文图片或他人设备照片。

当前硬件图改为以 **NX 为中心的实际系统拓扑**：相机默认直连 NX；RK3588 在可选视觉链路上；NX 与柜体双向通信；柜体分别连接关节驱动和工具 IO；机械安装与通信区别表达；安全回路独立。纠正此前“通信域→采集域→机械域”串行分层可能造成的拓扑误读。

当前软件图改为 **输入—感知—目标模型—任务决策** 的功能横向主线，加环境模型、运动技能/命令门、末端控制、MoveIt、驱动与设备，单独画状态反馈、作业结果反馈、工具 IO 直达驱动及 v2 的许可查询。环境碰撞对象指向 MoveIt，而非假定场景由技能节点转发。公共支撑只用代表性关联，不扩展所有节点级依赖。两版本保留同一模块位置与色彩，源码差异通过模块内容体现。

所有 SVG/PNG/PDF/DOT/drawio 已同步更新；drawio 补充折线转点与双向箭头信息。渲染目视检查与文件结构检查通过，尚未在 diagrams.net 中验证编辑渲染。未启动 ROS 或硬件程序。
