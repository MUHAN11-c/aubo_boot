# 架子式桃子采摘机器人 URDF 再导出移植指南

> 用途：SolidWorks 模型改版后重新导出 URDF，按本指南逐步接入本工作区，
> 避免重复踩坑。全程约 30 分钟。首次移植的完整过程与原理见
> `src/peach_gantry_description/README.md`（移植记录），报错原理见
> `src/peach_moveit_config/README.md` 附录 A/D。
> 本文是"操作清单"，解释为什么时会引用上述文档，不展开原理。

## 0. 前置

- 环境：Ubuntu 24.04 + ROS 2 Jazzy + colcon 工作区（本仓库根目录）。
- 两条铁律（后续步骤都依赖）：
  1. SolidWorks URDF Exporter 的产物有 4 个固定缺陷——mesh 相对路径、
     `<material>` 无 `name`、continuous 关节无 `<limit>`、部分关节
     `effort`/`velocity` 为 0——必须逐项修；
  2. 任何修改后用真机前的最小验证集（第 5 节）必须全绿。
- 启动任何东西之前先查残留进程：

  ```bash
  pgrep -af 'ros2 launch|component_container|ros2 run'
  ```

## 1. SolidWorks 重新导出

1. 用 URDF Exporter 重新导出（保持原有 link/joint 命名，命名变了后面
   MoveIt 配置要跟着改）。
2. 导出前在 Exporter 里顺手做两件事，能省后面的修补：
   - 有机械限位的关节（如 joint1 ±180°）类型选 **revolute** 并填
     lower/upper；确认无走线限制的旋转关节才用 continuous；
   - 每个关节的速度/力矩限值尽量填实测/估算值（导出物里 0 也要改，见 2.4）。
3. 检查导出物：`xxx.urdf` + `meshes/*.STL`（视觉与碰撞共用同一网格）。

## 2. 更新描述包 peach_gantry_description

以下命令都在工作区根目录执行，假设导出目录为 `$EXPORT`（例：
`/home/mu/Pictures/架子式桃子采摘机器人总装5_urdf/架子式桃子采摘机器人总装5`）。

### 2.1 拷贝资源

```bash
cp "$EXPORT/meshes/"*.STL src/peach_gantry_description/meshes/
# 删除不再存在的旧 STL（对比文件名清单后手动删）
cp "$EXPORT/"*.urdf src/peach_gantry_description/urdf/peach_gantry.urdf
```

URDF 文件名固定用 ASCII 的 `peach_gantry.urdf`（launch/配置都引用它）；
文件内 `robot name="..."` 保留中文无妨。

### 2.2 修导出物固定缺陷（按序执行）

```bash
cd src/peach_gantry_description/urdf

# ① mesh 相对路径 → package:// 引用（visual+collision 全部）
sed -i 's|filename="meshes/|filename="package://peach_gantry_description/meshes/|g' peach_gantry.urdf

# ② <material> 补 name 属性（urdf_parser 强制要求，缺了 check_urdf 报错）
sed -i 's|<material>$|<material name="">|' peach_gantry.urdf
```

③ 虚拟根 `base_footprint`：检查 `<robot>` 标签后是否有
`<link name="base_footprint"/>` + `base_footprint_joint`（fixed）。新导出物
没有，需把现有 URDF 开头的这段原样加回去（消除 KDL 根惯量警告，
TF 根符合 REP-105）：

```xml
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
```

④ 所有 continuous 关节补 `<limit effort="100" velocity="10"/>`——
**不补会导致 JTC 加载时段错误、ros2_control_node 假死**（原理见
peach_moveit_config README 附录 A-3）。revolute/prismatic 关节确认
`<limit>` 的 lower/upper 正确。

### 2.3 恢复项目化改动（对照旧 URDF 逐项搬）

- `joint1`：确认为 `type="revolute"`、`<limit lower="-3.1416"
  upper="3.1416" .../>`（实物走线限位）；
- 六臂关节速度占位限值：joint1/joint3：1.5 rad/s、joint2/joint4：0.25 m/s、
  joint5/joint6：3.0 rad/s、effort 100（带「仿真占位限值」注释）；
- 末端 `tool0`：把这段原样加回 `</robot>` 前（TCP 变了就改 origin）：

  ```xml
  <link name="tool0"/>
  <joint name="tool0_joint" type="fixed">
    <parent link="link6"/>
    <child link="tool0"/>
    <origin xyz="0.0479 0.230066 0" rpy="0 0 0"/>
  </joint>
  ```

### 2.4 校验

```bash
check_urdf src/peach_gantry_description/urdf/peach_gantry.urdf
# 运动树应为 base_footprint → base_link →（轮×4 + link1→…→link6→tool0）
colcon build --packages-select peach_gantry_description
colcon test --packages-select peach_gantry_description && colcon test-result --verbose
ros2 launch peach_gantry_description display.launch.py   # 眼睛过一遍模型
```

## 3. 更新 MoveIt 配置 peach_moveit_config

按改动幅度选一条路。

### 路线 A：仅几何/限位变化（link、joint 命名没变）——手改

1. `config/joint_limits.yaml`：核对六关节 `max_velocity`/`max_acceleration`
   与新 URDF 一致（仍是占位值就两边同步），**全部保持浮点写法**；
2. `config/initial_positions.yaml`：初值必须落在新 URDF 位置限位内
   （joint5 之类下限非 0 的特别注意）；
3. SRDF：组链 `base_link → tool0`、virtual_joint 指 `base_footprint`
   不变；`disable_collisions` 列表在几何变化后可保留（偏宽松无妨）；
4. TCP 未变则无需其他改动。

### 路线 B：结构变化（增删 link/joint、改名）——重跑 Setup Assistant

```bash
ros2 launch peach_moveit_config setup_assistant.launch.py
# 或新开：ros2 run moveit_setup_assistant moveit_setup_assistant
```

在 Setup Assistant 里重新生成后，**必须按这份补丁清单逐项重打**
（生成产物每次都带这些缺陷）：

| # | 位置 | 补丁 | 原因 |
|---|---|---|---|
| 1 | `config/joint_limits.yaml` | 所有限值改浮点写法；按 URDF 对齐数值并启用 `has_velocity/acceleration_limits: true`、补加速度 | 整数崩溃（附录 A-1）；TOTG/Pilz 前提（A-6） |
| 2 | SRDF | 组 `peach_arm` 改链式 `<chain base_link="base_link" tip_link="tool0"/>` | 位姿规划末端为 tool0 |
| 3 | SRDF | `virtual_joint` 的 `child_link` 改 `base_footprint` | 虚拟关节被跳过（A-2） |
| 4 | SRDF | `zero` 组态各关节值设在限位内（joint5=0.5） | 与 mock 初值一致 |
| 5 | `config/initial_positions.yaml` | 同上，初值落在限位内 | START_STATE_INVALID（A-4） |
| 6 | `config/moveit.rviz` | 加 TF display（坐标轴+名称，Marker Scale 0.3） | 核对坐标系 |
| 7 | `config/*.ros2_control.xacro` | 六关节接口声明（position 命令 + position/velocity 状态）不变；新关节要补声明 | 与 JTC 参数契约 |
| 8 | `config/ros2_controllers.yaml` | joints 列表与实际一致 | — |

控制 URDF/xacro 顶层文件里的 `initial_positions_file` 用默认值即可
（xacro 相对该文件所在目录解析，不要传绝对路径拼接）。

## 4. 重建

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash   # MoveIt 源码 overlay
colcon build --packages-select peach_gantry_description peach_moveit_config
source install/setup.bash
```

## 5. 最小验证集（全绿才算完成）

```bash
# ① demo 启动：控制器激活 + move_group 就绪，无崩溃无 virtual joint 警告
ros2 launch peach_moveit_config demo.launch.py
# 日志关键字：Configured and activated peach_arm_controller /
#            You can start planning now!

# ② TCP 静态变换正确
ros2 run tf2_ros tf2_echo link6 tool0

# ③ 关节目标规划+执行（替换 [] 内为限位内目标）
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup \
  "{request: {group_name: 'peach_arm', num_planning_attempts: 1, \
  allowed_planning_time: 5.0, max_velocity_scaling_factor: 0.5, \
  max_acceleration_scaling_factor: 0.5, goal_constraints: \
  [{joint_constraints: [...]}]}, planning_options: {plan_only: false}}"
# 期望：error_code val: 1

# ④ RViz 拖球 Plan & Execute 若干代表位姿（含伸展极限）
# ⑤ 结束：Ctrl+C 后 pgrep 确认无残留（必要时按 PID 补杀）
```

任何一步报错 → 先查 `peach_moveit_config/README.md` 附录 D 报错速查表，
都是首次移植踩过的坑，基本全覆盖。

## 6. 收尾

- 更新 `peach_gantry_description/README.md` 移植记录（追加一节：日期、
  导出版本、改动点、验证结果）；
- TCP/限位有变化时同步 `peach_moveit_config/README.md` 附录 B；
- `git status` 确认改动范围只在两个 peach 包内，build/install/log 不入库。
