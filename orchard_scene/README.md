# orchard_scene — 室外套袋桃园仿真场景

独立目录、**不进 colcon 构建、不改 `src/` 任何源码**。提供两档同一布局的
Gazebo Harmonic 世界（gz-sim 8.15.0 / sdformat14，随 ROS Jazzy vendor 安装）：
几何基元版零外部依赖，渲染网格版视觉换程序 OBJ（碰撞保持粗包络不变）。
每只套袋果的目标几何与 `scripts/sim_field_targets.py` / `field_pregrasp_cases.yaml`
的 `targets_20260909` 字段语义一致，可直接做感知/规划侧的目标注入源。

## 目录

```
orchard_scene/
├── README.md                       # 本文件
├── env.sh                          # 一键 export GZ_CONFIG_PATH + GZ_SIM_RESOURCE_PATH
├── tools/
│   ├── meshgen.py                  # OBJ/MTL 程序网格库（纯 stdlib）
│   └── generate_orchard_world.py   # 场景生成器：一跑出全部产物
├── models/orchard_meshes/          # HD 网格资产（model.config + meshes + materials）
├── worlds/
│   ├── bagged_peach_orchard.sdf    # 几何版：纯 SDF 基元，无外部依赖
│   └── bagged_peach_orchard_hd.sdf # 渲染版：视觉换 model:// OBJ；含 overview_cam 传感器
├── data/orchard_targets.json       # 每袋世界系目标几何（两版共用，同 seed 同布局）
└── preview/
    ├── orchard_layout.png          # 布局四联图（3D/俯视/侧视/axis_z 统计）
    ├── orchard_hd_render.png       # HD 网格软件渲染
    └── gazebo_hd_frame.png         # Gazebo 真实渲染帧（1280x720，无头抓取）
```

## 快速开始

```bash
source /opt/ros/jazzy/setup.bash
source orchard_scene/env.sh
cd orchard_scene
gz sim -r worlds/bagged_peach_orchard.sdf       # 几何版（无需网格资源）
gz sim -r worlds/bagged_peach_orchard_hd.sdf    # 渲染版（GUI 或 --headless-rendering）
```

无头渲染抓帧（HD 世界的 `overview_cam` 传感器，已验证可用）：

```bash
export GZ_PARTITION=orchard_capture             # 与机器上其他 gz 会话隔离
gz sim -s -r --headless-rendering worlds/bagged_peach_orchard_hd.sdf &
sleep 30 && gz topic -e -t /overview_cam/image -n 1 > /tmp/frame.pb
# 文本 protobuf：width/height/data(八进制转义) → numpy → PNG
```

## 再生成

```bash
python3 orchard_scene/tools/generate_orchard_world.py                # 默认 seed=20260923
python3 orchard_scene/tools/generate_orchard_world.py --rows 3 --trees 6 --seed 7
```

纯 stdlib + matplotlib（仅预览图，缺库自动跳过）。产物确定性：同 seed 同布局，
两档世界与 targets JSON 逐位一致。

## 场景设计

- 布局：4 行 × 5 株（可调），行距 4.0 m、株距 2.6 m、行端干道 3.0 m；地面 40×30 m
- 树形：三主枝自然开心形（干高 0.55–0.75 m，主枝开张 38–50°，每主枝 2 侧枝 + 结果枝）
- 叶冠：每树 7 个球形叶团（双叶色），碰撞球 r×0.92；主干/主枝圆柱带碰撞；细枝仅视觉
- 套袋果：每树 10–14 只，作业高度带 0.80–1.90 m，同树袋间距 ≥0.20 m；
  袋 = 牛皮纸色椭球（r=0.052 m）+ 扎口；axis_z 主流 0.72–1.0，~5% 近水平压测袋
  （对齐现场 1021_1）；每袋独立 link `bag_r{行}_t{株}_b{序}`，与 JSON id 一一对应
- 环境：方向光太阳 + 天空云 + 阴影；全部 static；HD 世界挂
  `gz-sim-sensors-system`（ogre2）供 `overview_cam` 出图

## targets JSON（orchard_targets_v1）

字段语义对齐 `src/peach_arm/test/fixtures/field_pregrasp_cases.yaml`
的 `targets_20260909`（注意：该文件是 **base_link 系**，本文件是 **world 系**；
world→base_link 由臂的安装位姿决定，装载时自行变换）：

| 字段 | 含义 |
|------|------|
| `entry_xyz` | 袋底 = 工具入口 [m] |
| `axis` | 袋底→袋颈 单位向量；主流 axis_z≥0.70 |
| `bag_bottom` / `bag_neck` | 袋底 / 袋颈（= entry_xyz + axis×travel_m）[m] |
| `travel_m` | 底到颈 0.104 m（= 2×纸身半径） |
| `axis_z` / `height_z` | 冗余标量，便于按包络筛选 |

当前 seed=20260923 实测：229 袋 / 20 树；axis_z∈[0.302, 0.995]，
≥0.70 占比 96.5%；近水平压测袋 8 只；高度带 0.80–1.68 m。
一致性断言已过：两档 SDF 的 bag link 集合与 JSON 完全一致、pose z 相符。

## 验证记录

- `gz sdf -k`（sdformat14）：两个世界 `Valid.`，零告警
- 无头渲染：`gz sim -s -r --headless-rendering` 抓得 `overview_cam` 1280×720 真实帧
  （见 `preview/gazebo_hd_frame.png`：四行树、土带、树影、冠缘纸袋均正确）
- 生成用时 ~2.4 s；几何版 445 KB / HD 421 KB / JSON 90 KB

## 边界声明

- 场景碰撞是**粗包络**（叶团球、干圆柱、袋球），仅供规划/仿真语境，
  **不是**安全边界；真机安全仍以柜/示教器急停与授权纪律为准（AGENTS.md 第 2 章）
- 仿真与 mock 同理不证明采摘方向正确；方向验收仍以真机轮次为最终权威
- 本目录不进 colcon 构建、未接入任何 launch/构建；要被 ros_gz spawn 臂模型时，
  另加 `GZ_SIM_RESOURCE_PATH` 指向 `src/`（只读引用，不改驱动包）
