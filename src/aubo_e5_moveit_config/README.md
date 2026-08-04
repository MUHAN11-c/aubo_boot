# aubo_e5_moveit_config — AUBO E5 的 MoveIt 2 配置与整体启动入口

## 简介

AUBO E5 的 MoveIt 2 配置包（纯 launch + config + rviz，无编译代码）：定义
规划组 `manipulator_e5`（`base_link`→`wrist3_Link` 六轴链）、双规划管线
（ompl 默认 + pilz_industrial_motion_planner）、按运行模式二选一的控制器
映射（sim/real 走 passthrough，mock 走标准 JTC）。本包只有唯一 launch
入口 `launch/moveit.launch.py`，整体拉起 `move_group` + `rviz2`；
standalone 单跑时自带 `robot_state_publisher` + `joint_state_publisher_gui`，
经 bringup 集成时这两者可关。输入是 `/joint_states` + TF（规划起态）与
MoveGroup 请求；输出是发给
`aubo_passthrough_trajectory_controller/follow_joint_trajectory`
（sim/real）或 `joint_trajectory_controller/follow_joint_trajectory`
（mock）的整段轨迹 goal。包级依赖方只有 aubo_e5_bringup（exec_depend +
include 本包 launch）；aubo_hand_eye_calibration 仅经 `/move_action`
action 接口使用 `manipulator_e5` 组，无包级依赖。

## 使用方法

构建（配置包，随全量构建装出 share）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select aubo_e5_moveit_config
source install/setup.bash
```

启动分两种场景：

```bash
# ① 经 bringup 集成（推荐；控制器映射按 hardware_mode 自动选择）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<控制器IP>

# ② 单跑（纯规划调试，不起硬件；自带 rsp + joint_state_publisher_gui）
ros2 launch aubo_e5_moveit_config moveit.launch.py
```

launch 参数（`--show-args` 可查中文说明）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `standalone_state_publishers` | true | true=单跑自带 rsp + joint_state_publisher_gui；bringup 集成时传 false（rsp 已由 bringup 提供，重复起会 TF 双发） |
| `controllers_file` | controllers.yaml | config/ 下的控制器映射文件名；mock 模式须用 controllers_mock.yaml（bringup 按 hardware_mode 自动透传，无需手改） |

关键配置默认值（权威源为 config/ 各 yaml，不在代码里另写）：

| 文件 | 要点 |
|---|---|
| `kinematics.yaml` | KDLKinematicsPlugin，search_resolution 0.005、timeout 0.05 |
| `joint_limits.yaml` | J1-J3 max_velocity 2.5964 / max_acceleration 2.5；J4-J6 3.1105 / 3.0（蓝本 aubo_boot 实测值）；jerk 与 cartesian_limits 为 Pilz 必需 |
| `ompl_planning.yaml` | 默认规划器 RRTConnect，longest_valid_segment_fraction 0.005 |
| `pilz_industrial_motion_planner_planning.yaml` | CommandPlanner，默认 PTP（可选 LIN/CIRC） |

**真机安全约定**：任何真机运动前，先把 RViz MotionPlanning 面板的
Velocity/Acceleration 滑条压到 0.1（本包 joint_limits.yaml 未定义
`default_*_scaling_factor`，滑条初值本就回退硬编码 0.1），确认行为符合
预期后再逐步放宽；程序侧调用对应请求里的
`max_velocity_scaling_factor` / `max_acceleration_scaling_factor`。

## 执行逻辑

一次规划/执行请求的数据流（`moveit.launch.py:95-138`）：

```
RViz / 客户端（/move_action goal）
  → 管线选择：默认 ompl；请求显式指定 pipeline_id 可走 pilz
  → request adapters（两管线相同）：
      ResolveConstraintFrames → ValidateWorkspaceBounds
      → CheckStartStateBounds → CheckStartStateCollision
  → 规划器：ompl=RRTConnect（projection_evaluator 取 shoulder/upperArm
      两关节投影评估）；pilz=确定性 PTP/LIN/CIRC
  → response adapters：
      ompl：AddTimeOptimalParameterization → AddRuckigTrajectorySmoothing
            → ValidateSolution → DisplayMotionPath
      pilz：ValidateSolution → DisplayMotionPath
  → ExecuteTrajectory：moveit_simple_controller_manager 按 controllers_file
      把整段轨迹作为一个 FollowJointTrajectory goal 发给目标控制器，
      硬件侧自行插补执行（passthrough 一次性下发，MoveIt 不做执行期缩放）
```

ompl 响应链里 Ruckig 必须排在 TOTG **之后**（response_adapters 按列表顺序
执行，Ruckig 要求输入已是时间参数化轨迹）；`totg.resample_dt` 由默认 0.1
收紧到 0.01，输出路点更密，供硬件侧五次重采样取更平滑的段边界。
双管线参数挂在 move_group 节点顶层（`planning_pipelines` 列表 +
`default_planning_pipeline` + `<管线名>.*` 命名空间），不能再嵌套进
`move_group` 键，否则 move_group 读不到列表会回退 legacy 单管线命名空间。
rviz2 节点拿与 move_group 同一份 `robot_description*` 参数（含
`robot_description_planning`），否则 MotionPlanning 面板滑条初值取不到。

执行余量按 passthrough 蓝本放宽（`moveit.launch.py:126-132`）：
`allowed_execution_duration_scaling=5.0`、
`allowed_goal_duration_margin=10.0`、`allowed_start_tolerance=0.15`——
轨迹一次性下发 + 硬件 RIB 水位流控，执行耗时弹性比流式 JTC 大，余量不足
会被 MoveIt 误判超时。失败路径：起态偏差超 0.15 rad 或执行超时任一发生
即判失败，不自动重发；取消经 action cancel 传到控制器侧（清双队列 +
`RobotMoveStop`，见 aubo_e5_controllers）。注意
`joint_limits.yaml` 的 `has_jerk_limits/max_jerk` 与 `cartesian_limits`
段是 Pilz 加载的硬要求，缺失会使 move_group 抛
`ParameterUninitializedException` 直接崩溃（见该文件头注释）。

## 软件框架

```
launch/moveit.launch.py   # 唯一入口：move_group + rviz2（+ 可选 rsp/jsp_gui）
config/
├── aubo_e5.srdf          # 规划组 manipulator_e5；home/camera_pose/zero 三个
│                         #   预定义位姿；disable_collisions 邻接/永不相撞表
├── kinematics.yaml       # KDL 运动学插件参数
├── joint_limits.yaml     # 关节速度/加速度/jerk + 笛卡尔极限（Pilz 用）
├── ompl_planning.yaml    # ompl 管线：planner_configs 全集 + manipulator_e5 组配置
├── pilz_industrial_motion_planner_planning.yaml
│                         # pilz 管线：CommandPlanner，默认 PTP
├── controllers.yaml      # sim/real 控制器映射 → aubo_passthrough_trajectory_controller
└── controllers_mock.yaml # mock 控制器映射 → joint_trajectory_controller（标准 JTC）
rviz/moveit.rviz          # RViz 配置（MotionPlanning 面板）
```

对外接口契约：

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| action server | `/move_action` | `moveit_msgs/MoveGroup` | 规划/执行主入口（hand-eye 标定经此使用） |
| action server | `/execute_trajectory` | `moveit_msgs/ExecuteTrajectory` | move_group 标准直接执行入口 |
| action/srv | Pilz 序列能力 | `MoveGroupSequence` | 由 capabilities 声明的 `MoveGroupSequenceAction`/`MoveGroupSequenceService` 提供 |
| action client | `/<控制器名>/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 执行下发；控制器名由 controllers_file 决定，关节顺序为权威六关节序 |
| pub | `/planning_scene`、`/monitored_planning_scene`、`/display_planned_path` 等 | 规划场景/轨迹可视化 | monitor 四开关全开（`moveit.launch.py:133-134`） |

依赖（package.xml）：`aubo_description`（URDF/xacro）、
`moveit_ros_move_group`、`moveit_simple_controller_manager`、
`moveit_kinematics`、`moveit_planners_ompl`、
`moveit_ros_visualization`、`robot_state_publisher`、
`joint_state_publisher_gui`、`rviz2`、`xacro`。注意本机 MoveIt 本体
（含 `pilz_industrial_motion_planner`，提供 PTP/LIN/CIRC 与序列能力）
**不是 apt 安装**——`/opt/ros/jazzy` 下没有 moveit 包，全部来自源码
工作区 `~/ws_moveit` 的 overlay（`~/.bashrc` 已自动 source）；换机器
部署时需先备妥该 overlay，否则 move_group 起不来。lint 走
`ament_lint_auto`（`colcon test` 强制）。
