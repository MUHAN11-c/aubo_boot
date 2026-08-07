# peach_moveit_config

## 简介
新机械臂，暂时不参与编程控制
架子式桃子采摘机器人（总装 4）的 MoveIt 2 配置包。由 MoveIt Setup Assistant
生成骨架后，经多轮排错与项目化修改达到仿真规划/执行闭环（详见附录 A 修改
全记录）。模型资源在 `peach_gantry_description`（URDF + meshes，移植过程见
该包 README）；重新导出 URDF 后的整套接入流程见
`docs/peach_urdf_migration.md`。

规划组 `peach_arm`：链 `base_link → tool0`（joint1~joint6 + 固定 TCP）。
底盘四轮不在规划组内，本包不含底盘控制。

## 使用方法

仿真一键启动（mock 硬件 + ros2_control + move_group + RViz）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash   # 本机 MoveIt 为源码 overlay
source install/setup.bash
ros2 launch peach_moveit_config demo.launch.py
```

RViz 中拖 tool0 交互球 → Plan & Execute。预置显示：Grid、TF（坐标轴+名称）、
MotionPlanning；Fixed Frame 为 `base_footprint`。

其他入口（demo 的组合件，按需单用）：

- `rsp.launch.py` — 只发 robot_description（xacro 展开 URDF + ros2_control 宏）
- `move_group.launch.py` — 只起 move_group（真机接入时复用）
- `moveit_rviz.launch.py` — 只起 RViz（moveit.rviz）
- `spawn_controllers.launch.py` — spawner × 2（joint_state_broadcaster +
  peach_arm_controller）
- `static_virtual_joint_tfs.launch.py` — 由 SRDF virtual_joint 生成
  world→base_footprint 静态 TF
- `setup_assistant.launch.py` — 重开 Setup Assistant 编辑本配置

## 执行逻辑

数据流（仿真与真机同构，仅硬件插件不同）：

```
RViz / 用户节点
  → /move_action（moveit_msgs/action/MoveGroup，位姿或关节目标）
  → move_group：规划管线（ompl / pilz / chomp / stomp）
      请求适配器：CheckStartStateBounds → CheckStartStateCollision → …
      响应适配器：AddTimeOptimalParameterization（TOTG 时间参数化，依赖
      joint_limits.yaml 的速度/加速度限值）→ ValidateSolution
  → simple_controller_manager 按 moveit_controllers.yaml 把轨迹发给
    /peach_arm_controller/follow_joint_trajectory（control_msgs/action/
    FollowJointTrajectory）
  → joint_trajectory_controller（JTC，100 Hz）按控制周期插值，
    经 ros2_control 位置 command interface 下发
  → 硬件插件：仿真为 mock_components/GenericSystem（命令即回显状态）；
    真机需自写 SystemInterface 插件（见附录 C）
  → joint_state_broadcaster 发 /joint_states（100 Hz）
```

关键事实：

- **JTC 与硬件的契约只有接口名**：`ros2_control.xacro` 声明六关节
  position 命令接口 + position/velocity 状态接口，与
  `ros2_controllers.yaml` 中 JTC 的 `command_interfaces`/`state_interfaces`
  一一对应。换真机插件时接口名不变，JTC 与 MoveIt 全部不用动。
- **起始状态合法性**：mock 硬件以 `initial_positions.yaml` 上电，
  必须落在 URDF 位置限位内，否则规划被 CheckStartStateBounds 拒（附录 A-4）。
- **限值链路**：URDF `<limit>`（位置/速度/力矩）是模型真值；
  `joint_limits.yaml` 覆盖/补充给规划用（可再收紧），两者必须都是浮点写法
  （附录 A-1）；TOTG 与 Pilz 强制要求每关节速度+加速度限值（附录 A-6）。
- **位姿规划末端**：SRDF 组定义为链 `base_link → tool0`，IK/位姿目标/交互球
  均作用于 tool0（TCP 偏移在 URDF `tool0_joint`，改 TCP 只动 URDF 一处）。

## 软件框架

```
peach_moveit_config/
├── .setup_assistant                 # Setup Assistant 会话（作者/源 URDF 路径/时间戳）
├── config/
│   ├── 架子式桃子采摘机器人总装4.urdf.xacro      # 顶层 xacro：include 描述包 URDF
│   │                                             #   + ros2_control 宏（FakeSystem）
│   ├── 架子式桃子采摘机器人总装4.ros2_control.xacro # 六关节接口声明 + mock 插件
│   ├── 架子式桃子采摘机器人总装4.srdf            # 组/组态/虚拟关节/碰撞豁免（ACM）
│   ├── initial_positions.yaml       # mock 上电初值（joint5=0.5，须在限位内）
│   ├── joint_limits.yaml            # 规划用限值（浮点！）+ 全局速度/加速度缩放 0.1
│   ├── kinematics.yaml              # KDL 运动学插件（peach_arm）
│   ├── moveit_controllers.yaml      # move_group → JTC 的控制器映射
│   ├── ros2_controllers.yaml        # controller_manager + JTC + jsb 参数
│   ├── pilz_cartesian_limits.yaml   # Pilz 笛卡尔限值
│   └── moveit.rviz                  # RViz 配置（Grid + TF + MotionPlanning）
├── launch/                          # demo.launch.py（总入口）+ 各单件 launch
├── CMakeLists.txt / package.xml     # 纯安装包（未接入 lint）
└── README.md
```

## 附录 A：修改全记录（Setup Assistant 产物 → 仿真闭环）

按时间顺序。每条 = 现象 → 根因 → 改法。URDF 侧改动详见
`peach_gantry_description/README.md` 移植记录，此处聚焦 MoveIt 配置侧。

### A-1 move_group 启动即崩溃（参数类型）

- 现象：`terminate called ... InvalidParameterTypeException:
  parameter 'robot_description_planning.joint_limits.joint6.max_velocity'
  has invalid type: expected [double] got [integer]`，move_group 进程退出。
- 根因：Setup Assistant 生成的 `joint_limits.yaml` 把限值写成整数
  （`max_velocity: 10`、`0`），而 MoveIt 把
  `robot_description_planning.joint_limits.*` 声明为 double 参数，YAML 整数
  加载为 integer 类型即炸。
- 改法：`config/joint_limits.yaml` 全部限值改浮点写法（`10.0`、`0.0`）。
  教训：该文件任何数值永远带小数点。

### A-2 virtual joint 被跳过（world TF 丢失）

- 现象：`Skipping virtual joint 'virtual_joint' because its child frame
  'base_link' does not match the URDF frame 'base_footprint'`，
  随后 `Assuming fixed joint`；world→base_link 静态 TF 缺失。
- 根因：生成 SRDF 时 URDF 根还是 base_link；后来描述包加了虚拟根
  base_footprint（消除 KDL 根惯量警告），SRDF 未同步。
- 改法：SRDF `virtual_joint` 的 `child_link` 改 `base_footprint`。
  静态 TF 由 `static_virtual_joint_tfs.launch.py` 按 SRDF 自动生成，
  改 SRDF 一处即可。

### A-3 JTC 加载段错误，ros2_control_node 假死（最难排的一个）

- 现象：`spawner peach_arm_controller` 三次 `load_controller` 超时后退出；
  `ros2_control_node` 打印残缺堆栈（只有 #13~#15 帧）后对所有服务无响应，
  SIGTERM 杀不掉（需 SIGKILL）。
- 定位：ptrace_scope=1 无法 attach，改用 `gdb -batch -ex run ... --args
  ros2_control_node ...` 以子进程方式跑，SIGSEGV 时自动停下：
  崩在 `JointTrajectoryController::on_init()+4429`，指令
  `movsd 0x18(%rdx),%xmm0` 对空指针解引用；残缺堆栈是 backward_ros 信号
  处理器在 loader 线程里打印到一半卡死所致。
- 根因：JTC on_init 解析 URDF 逐关节读 `<limit>`，而 continuous 类型的
  joint1（及 4 个轮关节）在 SolidWorks 导出物里没有 `<limit>` 元素——
  URDF 规范允许省略，JTC 不做空指针检查。
- 改法：给 URDF 全部 continuous 关节补 `<limit effort=".." velocity=".."/>`
  （continuous 的 limit 只需 effort/velocity，lower/upper 无效）。
  轮关节未被 JTC 控制，但一并补上防后患。
- 教训：ros2_control_node 卡死 → 先 gdb 子进程复现拿全堆栈；
  `pkill -f` 的 pattern 会匹配自身命令行，清理进程要用 `[x]` 括号写法
  或与运行命令分开执行。

### A-4 规划必败：起始状态越界

- 现象：RViz 任意 Plan & Execute 都
  `Joint 'joint5' from the starting state is outside bounds by: [0] should
  be in the range [0.5], [3.6]` → `START_STATE_INVALID`。
- 根因：`initial_positions.yaml` 全 0，mock 硬件以 0 上电，而 joint5 的
  URDF 限位是 [0.5, 3.6]（SolidWorks 导出的机械限位）。
- 改法：`initial_positions.yaml` 的 joint5 改 0.5（与 SRDF `zero` 组态一致）。
  原则：mock 初值必须落在 URDF 位置限位内。

### A-5 joint1 由 continuous 改为 revolute ±π

- 背景：实物 joint1 有走线，只允许 ±180°。URDF 中 continuous 类型会忽略
  `<limit>` 的 lower/upper，限位必须连类型一起改。
- 改法：URDF `joint1` → `type="revolute"`，
  `<limit lower="-3.1416" upper="3.1416" effort="100" velocity="1.5"/>`。
- 验证：限位内目标 joint1=2.0 规划执行成功；越界目标 joint1=4.0 被 OMPL
  钳制到边界，执行后实测停在 3.1416——MoveIt 对越界目标是"钳制执行"
  而非报错，程序发目标时需自行先校验。

### A-6 仿真占位限值补全（时间参数化/Pilz 前置条件）

- 背景：joint2~5 的 URDF 限值是 `effort="0" velocity="0"`，
  `joint_limits.yaml` 全部 `has_*_limits: false`——能规划，但 TOTG 无约束、
  Pilz 不可用、轨迹时长失真。
- 改法（两处同步，均为仿真占位值，接真机前换实测）：
  URDF——joint1/joint3：1.5 rad/s，joint2/joint4（直线）：0.25 m/s，
  joint5/joint6：3.0 rad/s，effort 100；
  `joint_limits.yaml`——六关节 `has_velocity_limits: true` +
  `has_acceleration_limits: true`，速度同 URDF，加速度约为 2 倍
  （直线 0.5 m/s²，joint1/3：3.0，joint5/6：6.0）。
- 验证：OMPL（RRTConnect）规划+执行 `val: 1`，joint_states 实测到位
  （误差 ≤0.01）；连续两个 goal 均成功；Pilz PTP 规划+执行成功。
- 无害提示：TOTG 警告 `combination of revolute and prismatic joints ...
  path_tolerance will not function correctly`（混合链固有）；planning scene
  警告 `Missing left1_joint ...`（轮关节无状态接口，仿真恒 0）。

### A-7 虚拟 TCP tool0（位姿规划末端）

- 需求：以夹爪 TCP 而非 link6 原点做位姿规划。
- 改法：URDF 加无 mesh 的 `tool0` link + `tool0_joint` 固定关节挂 link6；
  SRDF 组由"link/joint 枚举"改链式
  `<chain base_link="base_link" tip_link="tool0"/>`；
  RViz 配置加 TF 显示（坐标轴+名称）便于核对坐标系朝向。
- 偏移定稿 `xyz="0.0479 0.230066 0"`（相对 link6/joint6 坐标系，零姿态），
  历经零偏移占位 → y/z 写反 → 定稿三版；改 TCP 只动 URDF `tool0_joint`
  的 origin 一处。
- 验证：`tf2_echo link6 tool0` 平移 `[0.048, 0.230, 0.000]`；
  以 tool0 当前位姿 +5 cm 发位姿目标（位置球域 + 姿态约束），
  规划+执行成功，实测落在 1 cm 球域内。

### A-8 可达性地图（已移除）

评估过自研脚本与 DLR-MO/reachability_map，效果不达预期已整体移除，
结论备查见 `peach_gantry_description/README.md` 第 11 节。

## 附录 B：仿真参数配置说明

- **`ros2_controllers.yaml`**：controller_manager `update_rate: 100`；
  `peach_arm_controller`（JTC）：joints=[joint1..6]、
  command_interfaces=[position]、state_interfaces=[position, velocity]、
  `allow_nonzero_velocity_at_trajectory_end: true`；
  `joint_state_broadcaster` 全接口广播 → /joint_states 100 Hz。
- **`joint_limits.yaml`**：`default_velocity/acceleration_scaling_factor:
  0.1`（全局慢速，RViz 滑条与请求缩放在此基础上叠加）；每关节
  `has_velocity_limits/max_velocity/has_acceleration_limits/max_acceleration`
  ——TOTG 与 Pilz 的硬前提；**数值必须是浮点写法**（附录 A-1）。
  该文件值可与 URDF 不同（规划侧可再收紧），真机接入时按实测改这里+URDF。
- **`kinematics.yaml`**：`peach_arm` 用
  `kdl_kinematics_plugin/KDLKinematicsPlugin`（支持 prismatic 关节的数值
  IK）。混合链（旋转+直线）下个别位姿 IK 不收敛属正常，可换 TRAC-IK。
- **`moveit_controllers.yaml`**：把 `peach_arm_controller` 声明为
  `follow_joint_trajectory` action 手柄并挂到 `peach_arm` 组——
  move_group 执行轨迹靠它找到 action server；真机沿用同名控制器则不用改。
- **`initial_positions.yaml`**：mock 上电初值，必须在 URDF 限位内（A-4）。
- **`pilz_cartesian_limits.yaml`**：Pilz 的笛卡尔速度/加速度/减速度上限，
  Setup Assistant 默认值即可。

## 附录 C：真机接入指南

### C-1 控制数据格式（完整定义与本机实例）

总览：

| 环节 | 格式 | 说明 |
|---|---|---|
| 用户/RViz → move_group | `moveit_msgs/action/MoveGroup` | 位姿或关节目标，规划+时间参数化在 move_group 内完成 |
| MoveIt 输出 | `control_msgs/action/FollowJointTrajectory` goal | 见 C-1.2；positions 单位 rad（旋转）/ m（直线），已按限值时间参数化 |
| JTC ↔ 硬件 | double 数组（ros2_control interface） | 每控制周期（100 Hz）一个位置设定点，见 C-1.3 |
| 硬件插件 ↔ 驱动器 | 取决于实机（CAN/Modbus/EtherCAT/脉冲） | 插件负责 rad/m ↔ 驱动器单位（脉冲、度、mm）换算与限幅，见 C-1.4 |

MoveIt 侧不需要任何改动：只要 action 名
`/peach_arm_controller/follow_joint_trajectory` 与接口名不变。

#### C-1.1 组/关节顺序（一切数组的共同约定）

权威关节顺序（JTC 参数 `joints` 的顺序，所有数组都按它排）：

```
[joint1, joint2, joint3, joint4, joint5, joint6]
 旋转rad  直线m   旋转rad  直线m   旋转rad  旋转rad
```

#### C-1.2 FollowJointTrajectory action（MoveIt → JTC）

类型定义（`control_msgs/action/FollowJointTrajectory`）：

```
# ===== Goal =====
trajectory_msgs/JointTrajectory trajectory
trajectory_msgs/JointTolerance[] path_tolerance    # 路径容差（本配置未用）
trajectory_msgs/JointTolerance[] goal_tolerance    # 终点容差
builtin_interfaces/Duration goal_time_tolerance
# ===== Result =====
int32 error_code        # 1=SUCCESSFUL；-1=INVALID_GOAL；-2=INVALID_JOINTS；
                        # -3=OLD_HEADER_TIMESTAMP；-4=PATH_TOLERANCE_VIOLATED；
                        # -5=GOAL_TOLERANCE_VIOLATED
string error_string
# ===== Feedback（每控制周期发出）=====
std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual
trajectory_msgs/JointTrajectoryPoint error
```

核心载荷 `trajectory_msgs/JointTrajectory`：

```
std_msgs/Header header
string[] joint_names                              # 见 C-1.1 顺序
trajectory_msgs/JointTrajectoryPoint[] points     # 时间递增的路点
# JointTrajectoryPoint:
float64[] positions                # 6 维；rad（旋转关节）/ m（直线关节）
float64[] velocities               # rad/s、m/s（TOTG 会给全）
float64[] accelerations            # rad/s²、m/s²
float64[] effort                   # 本链路不使用，恒空
builtin_interfaces/Duration time_from_start   # 从轨迹起点起算的时刻
```

本机实例（"回零"goal 的轨迹示意，缩写为 3 点；单位见 C-1.1）：

```yaml
trajectory:
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  points:
  - positions:     [0.5, 0.1, 0.2, 0.05, 1.0, 0.5]    # 起点=当前状态
    velocities:    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 0, nanosec: 0}
  - positions:     [0.25, 0.05, 0.1, 0.025, 0.75, 0.25]
    velocities:    [0.113, 0.023, 0.045, 0.011, 0.113, 0.113]
    accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 2, nanosec: 210000000}
  - positions:     [0.0, 0.0, 0.0, 0.0, 0.5, 0.0]     # 终点
    velocities:    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 4, nanosec: 420000000}
```

注意：点密度由 TOTG 决定（通常几十~几百点，不是 3 点）；路点时刻严格递增；
首点恒等于当前状态。**直线关节 joint2/joint4 的 positions 单位是米**
（0.1 = 100 mm），与旋转关节的 rad 混排在同一数组——硬件插件换算时
必须按关节类型分支，这是最容易写错的地方。

#### C-1.3 ros2_control 接口层（JTC ↔ 硬件插件，100 Hz）

JTC 拿到轨迹后做样条插值，每个控制周期（10 ms）往 command interface 写
一个设定点。接口是"关节名/接口名"的键值对，值就是裸 double：

```
# command（JTC 写，硬件 write() 读）
joint1/position = 0.0123        # rad
joint2/position = 0.0025        # m
joint3/position = 0.0049        # rad
joint4/position = 0.0006        # m
joint5/position = 0.5187        # rad
joint6/position = 0.0123        # rad

# state（硬件 read() 写，JTC/jsb 读）
joint1/position = 0.0122        # rad（实测）
joint1/velocity = 0.1130        # rad/s（实测）
joint2/position = 0.0025        # m
joint2/velocity = 0.0231        # m/s
…（六关节同理）
```

jsb 把 state 接口原样打包成 `/joint_states`
（`sensor_msgs/msg/JointState`：`name[]` + `position[]` + `velocity[]`，
顺序同 C-1.1，100 Hz），MoveIt/Rviz 的状态回读全走这个话题。

#### C-1.4 硬件插件 → 驱动器（真机要自己写的换算）

插件 `write()` 里把 SI 单位换算成驱动器单位，示例如下（参数按实机替换）：

```cpp
// 旋转关节：rad → 编码器脉冲
//   pulse = rad / (2π) × 减速比 × 每圈脉冲数
double rad = command_interfaces_[j];                 // 例：joint1 = 0.0123 rad
int32_t pulse = static_cast<int32_t>(
    rad / (2.0 * M_PI) * REDUCTION_RATIO * PULSES_PER_REV);

// 直线关节：m → mm / 丝杠脉冲
//   mm = m × 1000；pulse = m / 导程 × 每圈脉冲数
double m = command_interfaces_[j];                   // 例：joint2 = 0.0025 m
double mm = m * 1000.0;                              // = 2.5 mm
int32_t lin_pulse = static_cast<int32_t>(m / LEAD_M * PULSES_PER_REV);
```

反向（`read()`）把驱动器读数除回 rad/m 写进 state 接口；方向符号、
零位偏置、限幅/软限位都收敛在插件这一层处理，不要改 URDF/JTC 参数去凑。
下发前建议做增量限幅（每周期最大变化量 = max_velocity × 周期），防跳变。

### C-2 标准做法：写一个 ros2_control 硬件插件（SystemInterface）

官方文档：<https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>；
范例：本仓库 `src/aubo_e5_hardware/`（AUBO E5 的真机/板级模拟双插件）、
GitHub 的 UR 驱动 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver>。

骨架（位置型六关节）：

```cpp
class PeachHardware : public hardware_interface::SystemInterface {
 public:
  // 读 <ros2_control> 的 <param>（IP、端口、减速比等），校验 joints
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params) override;

  // 每关节导出状态接口：position（必填）、velocity（可选）
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 每关节导出命令接口：position
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;  // 连驱动器、使能
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;  // 下使能、断连

  // read()：采驱动器实际位置/速度 → 写入 state interface 变量
  hardware_interface::return_type read(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;
  // write()：取 command interface 设定点 → 下发驱动器
  hardware_interface::return_type write(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;
};
```

配套改动只有两处：

1. `ros2_control.xacro` 把 `<plugin>mock_components/GenericSystem</plugin>`
   换成你的插件名（`your_pkg/PeachHardware`，附 plugin.xml 导出
   `hardware_interface/Pluginlib`），joint/interface 声明原样保留；
2. 启动：真机不要 `demo.launch.py`（内含 mock xacro）。复用
   `rsp.launch.py`（换真机 robot_description）+ 自己的 bringup
   （ros2_control_node + 两个 spawner）+ `move_group.launch.py` +
   `moveit_rviz.launch.py`。参考本仓库 `aubo_e5_bringup` 的组织方式。

注意：JTC 是"流式打点"控制器，硬件 write() 必须能跟上 100 Hz 设定点；
底层若只能低频收点，在插件内做缓冲/插值（AUBO E5 用的是 passthrough
一次性下发方案，不适用于标准 JTC，选型时先定架构）。

### C-3 备选：自写 FollowJointTrajectory action server（不走 ros2_control）

MoveIt 的 simple_controller_manager 只要求一个 action 名匹配的
FollowJointTrajectory server。驱动简单、关节少时可以自写节点：
收 goal → 按 time_from_start 调度下发 → 报 feedback/result。
缺点：绕开 ros2_control 生态（无 jsb、无控制器管理、无 mock 复用），
仅建议临时验证用。官方控制器列表与写法见
<https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html>。

### C-4 真机分阶段测试内容（务必按序）

0. **先查残留进程**：`pgrep -af 'ros2 launch|component_container|ros2 run'`，
   有就清掉（旧进程会独占设备/通道，且跑的是旧二进制）。
1. **只核对状态**：上电不运动，对比 `/joint_states` 与示教器/驱动器读数
   （含符号方向、prismatic 单位 m）；方向错在插件换算层改，不改 URDF。
2. **单关节小轨迹**：FJT action 手动构造 2~3 点、幅值 ≤0.05 rad
   （直线关节 ≤0.01 m）、时长 ≥3 s，速度/加速度缩放 0.1；逐关节过一遍，
   确认方向、限位、到位误差。
3. **取消/抢占**：执行中发新 goal（旧 goal 应 CANCELED）、执行中 cancel，
   确认硬件侧有安全减速而非急坠。
4. **整链轨迹**：六关节联动轨迹，核对墙钟/标称时长比 ≈1、终点误差。
5. **MoveIt 整机**：RViz 速度滑条压到最低，Plan & Execute 若干代表位姿
   （含伸展极限与折叠附近），观察自碰撞误报（碰撞网格偏胖会拦掉合法解，
   届时按 `peach_gantry_description` README 遗留事项做碰撞体简化）。
6. **长时间运行**：≥30 min 循环轨迹，记录温度/丢点/漂移。

安全：首次运动所有缩放压 0.1；确认急停回路有效；`auto_power_on` 类自动
使能一律关闭；限位以机械实测为准回填 URDF 与 `joint_limits.yaml` 两处。

## 附录 D：报错速查表

| 报错/现象 | 根因 | 解法 | 详见 |
|---|---|---|---|
| `InvalidParameterTypeException ... expected [double] got [integer]` | joint_limits.yaml 写了整数 | 限值全部带小数点 | A-1 |
| `Skipping virtual joint ... does not match the URDF frame` | SRDF 虚拟关节指向旧根 | child_link 改 base_footprint | A-2 |
| spawner 三次 `load_controller` 超时，节点假死 | continuous 关节缺 `<limit>`，JTC on_init 空指针 | URDF 补 effort/velocity limit | A-3 |
| `START_STATE_INVALID ... outside bounds` | mock 初值越出 URDF 限位 | initial_positions 改到限位内 | A-4 |
| 规划成功但不执行/时长失真 | 限值为 0 或 has_*_limits: false | URDF+yaml 补真实限值 | A-6 |
| 越界目标仍"执行成功" | MoveIt 对目标状态做边界钳制 | 程序发目标前自行校验 | A-5 |
| `No 3D sensor plugin(s) defined` | 无 octomap 传感器配置 | 无害，忽略 | A-6 |
| `Missing left1_joint ...` | 轮关节无状态接口 | 无害（仿真恒 0） | A-6 |
| rviz 启动报插件 namespace collision | rviz 插件重复链接（常见） | 无害，忽略 | — |
