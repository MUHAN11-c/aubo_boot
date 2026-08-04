# peach_gantry_description

## 简介

架子式桃子采摘机器人（总装 4）的 URDF 描述包，模型由 SolidWorks URDF Exporter
导出，包含四轮移动底盘与升降/伸缩式采摘臂共 11 个连杆、10 个关节
（4 个 continuous 轮关节 + joint1~joint6：旋转/升降/俯仰/伸缩/腕摆/腕转）。
从 SolidWorks 导出物移植到本工作区的完整过程见文末「移植过程记录」。

## 使用方法

一键展示模型（robot_state_publisher + joint_state_publisher_gui + RViz2，
自带 `rviz/display.rviz` 配置，固定帧 `base_footprint`）：

```bash
ros2 launch peach_gantry_description display.launch.py
```

URDF 位于 `urdf/peach_gantry.urdf`，网格经 `package://peach_gantry_description/meshes/`
引用，也可单独传给 `robot_state_publisher` 使用（launch 参数 `model` 可换模型文件）。
TF 树根为无惯量的 `base_footprint`（REP-105），经固定关节连接 `base_link`。

## 执行逻辑

纯资源包，无可执行节点。关节链：`base_link` → 四轮（left1/left2/right1/right2，
continuous）+ `joint1`（旋转，revolute ±π，实物限位）→ `joint2`（升降，prismatic 0~0.3 m）→
`joint3`（俯仰，revolute -0.8~0.5 rad）→ `joint4`（伸缩，prismatic -0.5~0.15 m）→
`joint5`（腕摆，revolute 0.5~3.6 rad）→ `joint6`（腕转，revolute ±1 rad）。
末端另有固定关节挂虚拟 TCP `tool0`（相对 link6 零姿态偏移
x=47.9 / y=230.066 / z=0 mm），是 MoveIt 位姿规划与 IK 的 tip
（SRDF 组 `peach_arm` 定义为链 `base_link → tool0`）；换装夹爪/吸盘或
重测后修改 `tool0_joint` 的 origin。
注意 SolidWorks 导出的部分关节 `effort`/`velocity` 限值为占位值，接入真机前需补全。

## 软件框架

```
peach_gantry_description/
├── urdf/peach_gantry.urdf   # 主模型（mesh 路径已改为 package:// 引用；
│                            #   根为无惯量 base_footprint，固定关节接 base_link）
├── meshes/                  # 11 个 STL（视觉与碰撞共用同一网格）
├── launch/display.launch.py # 模型展示：rsp + joint_state_publisher_gui + RViz2
├── rviz/display.rviz        # 配套 RViz 配置（固定帧 base_footprint）
├── CMakeLists.txt           # 安装 meshes/urdf/launch/rviz，接入 ament_lint_auto
├── LICENSE                  # BSD-3-Clause（拷贝自 aubo_description）
└── package.xml
```

## 移植过程记录

按时间顺序记录从 SolidWorks 导出物到本工作区 ROS 2 包的全部改动（2026-08-04）。

### 0. 移植来源

- 原始目录：`/home/mu/Pictures/架子式桃子采摘机器人总装4_urdf/架子式桃子采摘机器人总装4/`
- 原始内容：`架子式桃子采摘机器人总装4.urdf`（382 行）+ `meshes/` 下 11 个 STL
  （base_link / left1_link / left2_link / right1_link / right2_link / link1~link6）。
- 导出物为 SolidWorks URDF Exporter 的典型产物：mesh 用相对路径、`<material>`
  无 `name` 属性、视觉与碰撞共用同一网格、部分关节 `effort`/`velocity` 限值为 0。

### 1. 建包与资源拷贝

- 新建 `src/peach_gantry_description/`（ament_cmake 纯资源包，结构对齐
  `aubo_description`：相同的 CMakeLists 骨架、`ament_lint_auto` lint 接入、
  BSD-3-Clause `LICENSE` 直接拷贝自 `aubo_description`）。
- 11 个 STL 原样拷入 `meshes/`，文件名保持导出原貌（大写 `.STL` 后缀）。
- URDF 重命名为 ASCII 文件名 `urdf/peach_gantry.urdf`（避免中文文件名在工具链
  中的潜在编码问题）；URDF 内部的 `robot name="架子式桃子采摘机器人总装4"`
  属性保留中文不变。

### 2. URDF 修改一：mesh 路径改为 package:// 引用

- 原导出物为相对路径 `filename="meshes/xxx.STL"`，只有进程 CWD 恰好在 URDF
  目录时才能解析，装到 `install/` 后必然失效。
- 用 sed 将全部 22 处（11 个 visual + 11 个 collision）替换为
  `filename="package://peach_gantry_description/meshes/xxx.STL"`。

### 3. URDF 修改二：`<material>` 补 `name` 属性

- `check_urdf` 报错 `Visual material must contain a name attribute`：
  urdf_parser 强制要求 `<material>` 带 `name`，SolidWorks 导出物缺失。
- 11 个 `<material>` 统一改为 `<material name="">`（空名内联材质，颜色值不变）。

### 4. URDF 修改三：新增 base_footprint 虚拟根

- robot_state_publisher 启动时 kdl_parser 警告
  `The root link base_link has an inertia specified in the URDF, but KDL does
  not support a root link with an inertia`（base_link 带 197 kg 车架惯量，
  KDL 会直接丢弃根惯量）。
- 修复：在 URDF 顶部新增无惯量的 `<link name="base_footprint"/>` 和
  `base_footprint_joint`（type=fixed，零位姿）连接 base_footprint → base_link。
- 效果：KDL 警告消除；TF 树根变为 base_footprint，符合 REP-105 的底盘父帧
  约定，便于以后接定位/导航。

### 5. 新增 display launch 与 RViz 配置

- `launch/display.launch.py`：robot_state_publisher（`robot_description` 经
  `Command(['xacro ', model])` 代入）+ joint_state_publisher_gui + rviz2；
  声明 `model` 参数，默认指向包内 URDF，可替换模型文件。
- `rviz/display.rviz`：固定帧 `base_footprint`，预置 Grid / TF / RobotModel
  （RobotModel 走 `/robot_description` 话题），Orbit 视角对准机体。
- CMakeLists 安装目录由 `meshes urdf` 扩为 `meshes urdf launch rviz`；
  package.xml 增加 exec_depend：`joint_state_publisher_gui`、
  `robot_state_publisher`、`rviz2`。
- launch 文件按项目统一风格补 BSD copyright 头（ament_copyright lint 曾报
  `could not find copyright notice`，补上后通过）。

### 6. 验证记录

- `check_urdf`：解析成功，运动树为 base_footprint → base_link →
  （link1→…→link6 采摘臂链 + 4 个轮 link），共 11 连杆 10 关节。
- `colcon build --packages-select peach_gantry_description` 成功；
  `colcon test` 123 项 lint 测试 0 错误 0 失败。
- launch 冒烟：`timeout 15 ros2 launch peach_gantry_description display.launch.py`，
  三个节点全部正常启动，jsp_gui 成功获取 robot_description，rsp 启动无 KDL
  警告；退出后 `pgrep` 确认无残留进程。

### 7. 下游影响：peach_moveit_config 的两处适配（2026-08-04）

`peach_moveit_config`（MoveIt Setup Assistant 生成）接入本包模型后启动 demo
时暴露两个问题，均已修复：

- **move_group 崩溃**：`config/joint_limits.yaml` 中 `max_velocity`/`max_acceleration`
  写成整数（`0`、`10`），MoveIt 参数类型要求 double，报
  `InvalidParameterTypeException ... expected [double] got [integer]`。
  修复：全部改为浮点写法（`0.0`、`10.0`）。
- **virtual joint 警告**：SRDF 中 `virtual_joint` 的 `child_link` 还是
  修改前的 `base_link`，与 URDF 根 `base_footprint` 不匹配（警告
  `Skipping virtual joint ... does not match the URDF frame 'base_footprint'`，
  world→base_link 静态 TF 也随之失效）。修复：`child_link` 改为
  `base_footprint`，静态 TF 自动变为 world→base_footprint。

修后冒烟：`ros2 launch peach_moveit_config demo.launch.py` 25 秒，
move_group 正常加载（KDL 运动学插件、规划管线、planning scene monitor 全部
就绪），无崩溃、无 virtual joint 警告；`No 3D sensor plugin(s) defined for
octomap updates` 为无传感器时的标准提示，不影响使用。

### 8. JTC 加载段错误与起始位姿越界（2026-08-04，第二轮适配）

demo 能启动后又暴露两个更深的问题，均已修复：

- **peach_arm_controller（JTC）加载时段错误**：spawner 三次
  `load_controller` 超时、`ros2_control_node` 打残缺堆栈后假死。gdb 定位：
  `SIGSEGV` 于 `JointTrajectoryController::on_init()+4429`，指令
  `movsd 0x18(%rdx),%xmm0` 对空指针解引用。根因：JTC on_init 会解析 URDF
  逐关节读取限值，而 **continuous 类型的 joint1 没有 `<limit>` 元素**
  （SolidWorks 导出物对 continuous 关节一律省略；URDF 规范允许省略，
  但 JTC 不做空指针检查）。修复：给 URDF 全部 5 个 continuous 关节
  （joint1 + 4 个轮关节）补 `<limit effort="100" velocity="10"/>`
  （continuous 的 limit 只需 effort/velocity；数值为占位，待电机实测）。
- **起始位姿越界导致规划必败**：`config/initial_positions.yaml` 全 0，
  而 joint5 的 URDF 限位是 [0.5, 3.6]，mock 硬件以 0 启动，MoveIt
  `CheckStartStateBounds` 报 `Joint 'joint5' ... is outside bounds`。
  修复：joint5 初值改 0.5（与 SRDF `zero` 组态一致）。

验证（修后）：隔离三件套（rsp + ros2_control_node + spawner）控制器
`Configured and activated`；`/peach_arm_controller/follow_joint_trajectory`
goal 3 秒轨迹 `SUCCEEDED`（Goal successfully reached）；完整 demo launch
move_group 就绪，`/joint_states` 中 joint5 初值 0.5 落在限位内。

### 9. 仿真占位限值补全，仿真规划执行闭环（2026-08-04，第三轮适配）

目标：让仿真（mock + MoveIt）能完整规划并执行。此前 joint2~5 的 URDF 限值
为 `effort="0" velocity="0"`、`joint_limits.yaml` 全部 `has_*_limits: false`，
时间参数化（TOTG）与 Pilz 都没有可用约束。改动：

- **URDF**（本包）：六个臂关节补仿真占位限值——joint1/joint3：1.5 rad/s；
  joint2/joint4（直线）：0.25 m/s；joint5/joint6：3.0 rad/s；effort 统一 100。
  均带「仿真占位限值」注释，待电机/电缸实测后替换。
- **`joint_limits.yaml`**（peach_moveit_config）：六关节全部
  `has_velocity_limits: true` / `has_acceleration_limits: true`，速度与 URDF
  对齐，加速度约为速度 2 倍（直线 0.5 m/s²，joint1/3：3.0，joint5/6：6.0）。

验证（修后，demo launch 内发 `/move_action`）：

- OMPL（RRTConnect）规划+执行到 `[0.5,0.1,0.2,0.05,1.0,0.5]`：`val: 1`
  （SUCCESS），`/joint_states` 实测到达（误差 ≤0.01）；
- 回零第二 goal：`SUCCEEDED`，执行管理器状态可重复使用；
- Pilz PTP 规划+执行：`Goal reached, success!`（旋转/直线混合关节可用）。

补充（同日）：按实物限位将 joint1 由 `continuous` 改为 `revolute ±π`
（continuous 类型会忽略 `lower`/`upper`，限位必须改类型才生效；JTC 必需的
`<limit>` 元素随之自然存在，轮关节仍保持 continuous + effort/velocity 限值）。
验证：限位内目标 joint1=2.0 规划执行成功；越界目标 joint1=4.0 被 OMPL 钳制
到边界，执行后 `/joint_states` 中 joint1 停在 3.1416，全程不超过 ±π。
- 已知无害提示：TOTG 警告
  `combination of revolute and prismatic joints ... path_tolerance will not
  function correctly`（信息性，混合链固有）；planning scene 警告
  `Missing left1_joint ...`（轮关节无状态接口，仿真下恒为 0）。

### 10. 虚拟 TCP tool0 与实测偏移（2026-08-04）

- **tool0 坐标系**：URDF 末端新增无 mesh 的 `<link name="tool0"/>`，经固定关节
  `tool0_joint` 挂在 `link6` 下，作为 MoveIt 位姿规划/IK 的末端（tip）。
  SRDF 组 `peach_arm` 相应改为链式定义
  `<chain base_link="base_link" tip_link="tool0"/>`。
- **TCP 偏移**：`tool0_joint` origin 初为零偏移占位，后按给定实测值设定为
  `xyz="0.0479 0.230066 0"`（相对 link6 即 joint6 坐标系，零姿态），
  即 x=47.9 mm / y=230.066 mm / z=0 mm（历经零偏移 → y/z 写反两版后
  定稿；RViz 开 TF 显示可直观看各坐标系朝向核对）。
- 验证：demo launch 中 `tf2_echo link6 tool0` 平移 `[0.048, 0.230, 0.000]`、
  姿态单位四元数（按定稿值复测）；此前以零偏移/首版偏移做过的位姿目标
  规划+执行均 `SUCCEEDED`（TCP 只改固定变换，不影响规划链路）；
  `peach_moveit_config/config/moveit.rviz` 已加 TF 显示（坐标轴+名称）。

### 11. 工作空间可达性地图：评估后移除（2026-08-04）

曾评估两种方案并全部移除（效果不达预期，暂不考虑此功能）：

- 自研 `tools/peach_reachability_map.py`（网格点 + `/compute_ik` 采样），
  效果不佳，当日删除；
- GitHub 现成包 DLR-MO/reachability_map（关节采样 + FK + Bonxai 体素），
  克隆于 `src/reachability_map` 并打过三处补丁（根坐标系 frame、prismatic
  按 voxel 采样、`--ignore-collisions` 开关）。实测：带自碰撞检查拒绝
  53.8% 位形（碰撞网格与视觉共用偏胖，图被掏成空心壳）；纯运动学版本
  r 0.15..1.23 m、z 1.20..2.45 m（z 下限 1.2 m 为运动学真实）。因效果不好
  已整体移除：vendor 包、rviz 的 MarkerArray 显示均已删除。

如未来重启该功能，可参考上述结论：z 下限是运动学极限；自碰撞失真需先做
碰撞网格简化（凸包化）才有意义。

### 12. 已知遗留事项

- 全部关节限值目前为**仿真占位值**（见第 9 节：旋转 1.5/3.0 rad/s、直线
  0.25 m/s、effort 100；轮关节 100/10），仅保证仿真规划执行闭环；接真机前
  必须按电机/电缸实测参数替换 URDF 与 `joint_limits.yaml` 两处。
- 碰撞网格与视觉共用同一 STL，未做凸包/简化，进 MoveIt 前建议生成简化碰撞体。
- 质量/惯量参数为 SolidWorks 材料属性计算值，未与实物称重校验。
- 4 个轮关节为 continuous 类型，用于差速/四轮驱动控制时需另配运动学控制器，
  本包不含驱动配置。
