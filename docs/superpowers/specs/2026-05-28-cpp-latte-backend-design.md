# 咖啡拉花 C++ 后端 — 完整方案

**日期**: 2026-05-28 | **版本**: v4.0
**参考**: [MoveIt2 官方教程](https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html) | [Motion Planning API](https://moveit.picknik.ai/humble/doc/examples/motion_planning_api/motion_planning_api_tutorial.html) | [MoveItVisualTools 源码](https://github.com/ros-planning/moveit_visual_tools)

---

## 1. 架构总览

```
┌──────────────────────────────────────────────────────────────┐
│                         离线层                                │
│                                                               │
│  线路 A: npz → .lat (convert_npz_to_lat.py)                  │
│  线路 B: 无 (C++ 直接调用 MoveIt2 computeCartesianPath)       │
│                                                               │
├──────────────────────────────────────────────────────────────┤
│                   在线层 (latte_node, C++)                    │
│                                                               │
│  Service: /latte/replay_trajectory (ReplayLatteTrajectory)    │
│                                                               │
│  ┌─ LatteNode (继承 rclcpp::Node) ──────────────────────────┐│
│  │                                                           ││
│  │  成员 (4 个):                                              ││
│  │    MoveGroupInterface       plan + execute                ││
│  │    MoveItVisualTools        RViz2 可视化                   ││
│  │    PlanningSceneInterface   碰撞场景                       ││
│  │    atomic<bool> busy        防重入                         ││
│  │                                                           ││
│  │  方法 (5 个):                                              ││
│  │    handleReplay()           service 回调 (入口+调度)       ││
│  │    loadLat()                线路A: .lat → waypoints        ││
│  │    generatePattern()        线路B: Eigen → waypoints       ││
│  │    retarget()               SE(3) 重定目标                 ││
│  │    execute()                ③ preview ④ safety ⑤ planExec ││
│  └───────────────────────────────────────────────────────────┘│
│                                                               │
│  ┌─ latte_math.hpp (header-only, Eigen, ~250行) ────────────┐│
│  │  CartesianTrajectory / readLat / writeLat                 ││
│  │  quatToRot / rotToQuat / quatMultiply / eulerDegToQuat   ││
│  │  retargetTrajectory / retargetWithOrientationConstraint   ││
│  │  checkWorkspaceBounds                                     ││
│  │  generateHeart / generateRosetta / generateTulip / ...    ││
│  └───────────────────────────────────────────────────────────┘│
└──────────────────────────────────────────────────────────────┘
```

## 2. 文件结构 (5 个源文件) 

```
latte_node/                               ← 新建 C++ 包
├── CMakeLists.txt
├── package.xml
├── include/latte_node/
│   ├── latte_node.hpp                   class LatteNode 声明
│   └── latte_math.hpp                   header-only 数学工具
├── src/
│   ├── latte_node.cpp                   管线实现
│   └── main.cpp                         rclcpp::init + spin
└── launch/latte.launch.py
```

### CMakeLists.txt (参考官方教程结构)

```cmake
cmake_minimum_required(VERSION 3.10)
project(latte_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ivg_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(latte_node
  src/latte_node.cpp
  src/main.cpp
)
ament_target_dependencies(latte_node
  rclcpp moveit_core moveit_ros_planning_interface
  moveit_visual_tools geometry_msgs ivg_interfaces
  tf2_ros tf2_eigen Eigen3
)

install(TARGETS latte_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
```

---

### 管线阶段总览

```
① Load/Generate   → CartesianTrajectory
② Retarget        → SE(3) 变换
③ computeCartesianPath → RobotTrajectory (所有模式都规划)
④ publishTrajectoryPath → RViz2 "Planned Path" (机器人ghost + EE轨迹线)
⑤ Safety Check    → 工作空间边界
⑥ Execute         → 仅 action 模式
```

## 3. 逻辑流程

### 3.0 节点启动

```
main.cpp:
  1. rclcpp::init()
  2. NodeOptions.automatically_declare_parameters_from_overrides(true)
  3. LatteNode 构造 — 声明参数 + 创建 Service
  4. node->init()  — 创建 MoveGroupInterface + MoveItVisualTools (延后 shared_from_this)
  5. MultiThreadedExecutor(2).add_node(node).spin()
```

### 3.1 入口: Service 收到请求

```
/latte/replay_trajectory 收到 ReplayLatteTrajectory 请求
  ↓
handleReplay(req, res):
  1. atomic<bool> busy_ 防重入 — 如正执行中则 res.success=false 返回
  2. 参数校验: speed_scale ∈ [0.01,10], mode ∈ {preview,debug,action}
  3. 进入管线
```

### 3.2 ① Load/Generate — 轨迹数据

```
分支判断: req.pattern_type == "" ?

  ── 是 → 线路 A: 录制回放 ──
    1. 拼接路径: "resource/cartesian/{req.arm}/episode_{req.episode_idx:06d}.lat"
    2. latte_math::readLat(path)
       - 读 64 字节 Header → 校验 magic "LAT\0" + version 1
       - 读 Body → 构造 CartesianTrajectory
    3. 失败 → res.success=false, "load failed"

  ── 否 → 线路 B: 参数化生成 ──
    1. latte_math::generatePattern(type, cup_radius, surface_z,
                                    mix_h, draw_h, finish_h,
                                    wiggle_amp, wiggle_freq)
       - 参数方程: heart/rosetta/tulip/swan → xyz[N,3] (Eigen)
       - compose: 融合(50帧, z=surface_z+mix_h)
                   成形(N-80帧, z=surface_z+draw_h+摆动正弦)
                   收尾(30帧, z逐渐抬起到 surface_z+finish_h)
       - antiSloshing: 梯形速度剖面 → CubicSpline 重参数化 dt
       - pitchProfile: 融合 45°→30° / 成形 30°±3° / 收尾 30°→60°
       → CartesianTrajectory (含最终朝向)

输出: CartesianTrajectory { positions[N], orientations[N], dt, frame_id }
```

### 3.3 ② Retarget — SE(3) 重定目标

```
resolveTarget(req):
  线路 A: 目标 = start_pose
            if start_pose 全零 → tf2 lookup("base_link", "tool_tcp")
            else              → 手动指定
  线路 B: 目标 = (param "lizhu_link.x",
                  param "lizhu_link.y",
                  req.cup_surface_z) @ identity_quat

R_rel = R(req.roll_deg, req.pitch_deg, req.yaw_deg)   // 用户输入的旋转矩阵

  线路 A: retargetTrajectory(cart, target, rpy_user)
    p0 = cart.positions[0]
    for each frame i:
      positions[i]    = R_rel * (positions[i] - p0) + target.position
      orientations[i] = rotToQuat(R_rel) * orientations[i]    // 完整保留录制朝向

  线路 B: retargetWithOrientationConstraint(cart, target, rpy_user)
    yaw = extractYaw(R_rel)   // 从 R_rel 提取绕 Z 旋转角
    for each frame i:
      positions[i]    = R_rel * (positions[i] - p0) + target.position
      orientations[i] = eulerDegToQuat(0, 0, yaw) * orientations[i]  // 仅yaw, 保留pitch

输出: 变换后 CartesianTrajectory (base_link 坐标系)
```

### 3.4 ③ computeCartesianPath — MoveIt2 笛卡尔规划

```
(所有 mode 都执行此步, preview/action 统一)

  1. 采样 waypoints: cart.toGeometryMsgPoses(step = req.waypoint_sample_step)
     每隔 step 帧取一个 Pose, 加上最后一帧

  2. move_group_->setMaxVelocityScalingFactor(1.0)   // 不限速
     move_group_->setMaxAccelerationScalingFactor(1.0)

  3. fraction = move_group_->computeCartesianPath(
         waypoints,           // geometry_msgs::Pose[]
         max_step  = 0.01,    // 笛卡尔插值步长 1cm
         jump_threshold = 0.0, // 禁用跳变检测
         trajectory,          // 输出: moveit_msgs::RobotTrajectory
         avoid_collisions = true)

  4. 如果 trajectory.joint_trajectory.points 为空 → 失败返回

  5. 如果 fraction < 0.95:
       RCLCPP_WARN → 以 avoid_collisions=false 重试
       fraction = computeCartesianPath(..., avoid_collisions=false)
       如果仍 < 0.95 → 失败返回

  6. scaleTrajectoryTiming:   // 按 speed_scale 缩放时间戳
       effective_dt = cart.dt / max(speed_scale, 0.01)
       for each joint_trajectory.point at index i:
         t = i * effective_dt
         point.time_from_start.sec     = int32(t)
         point.time_from_start.nanosec = uint32((t - int32(t)) * 1e9)

输出: moveit_msgs::RobotTrajectory (关节轨迹, 含速度缩放后的时间戳)
```

### 3.5 ④ publishTrajectoryPath — RViz2 可视化

```
  visual_tools_->deleteAllMarkers()
  auto robot_state = move_group_->getCurrentState()

  visual_tools_->publishTrajectoryPath(traj, robot_state, blocking=false)
    内部流程 (moveit_visual_tools 源码):
      1. loadSharedRobotState() — 加载机器人模型
      2. robot_trajectory::RobotTrajectory 构造 (msg → moveit_core 类型)
      3. 遍历每个 waypoint, FK 求 EE 位姿 → 发布 DisplayTrajectory 消息
      4. target topic: /move_group/display_planned_path

  visual_tools_->trigger()  // 批量发送

  效果: RViz2 MotionPlanning 插件的 "Planned Path" display 显示:
    - 机器人 ghost (起始状态半透明模型)
    - 绿色 EE 轨迹线
    - 路径点球标记
```

### 3.6 ⑤ Safety Check — 工作空间边界

```
  latte_math::checkWorkspaceBounds(cart, workspace)
    for each position in cart.positions:
      if x ∉ [x_min, x_max] or y ∉ [y_min, y_max] or z ∉ [z_min, z_max]:
        → 记录违规

  默认工作空间: AUBO E5 工作半径 886.5mm
    X[-0.87, 0.87] Y[-0.87, 0.87] Z[-0.85, 1.10]

  policy = workspace.safety_policy:
    "warn_and_block":
      mode="action" → res.success=false, "safety: ..."
      mode="preview"/"debug" → 仅 warn 日志
    "warn_only" → 仅 warn 日志
    "ignore"    → 跳过
```

### 3.7 分支: preview/debug 返回

```
  if mode ∈ {"preview", "debug"}:
    res.success          = true
    res.fraction         = fraction
    res.trajectory_points = traj.points.size()
    res.num_frames       = cart.positions.size()
    res.path_length      = cart.totalLength()
    res.waypoints        = sampleWaypoints(cart, step)   // 前端 3D 用
    return   ← 管线结束, 不执行
```

### 3.8 ⑥ Execute — 执行 (仅 action)

```
  result = move_group_->execute(traj)
    → MoveIt2 通过 ros2_control / JointTrajectoryController 发送给机械臂
    → 真机执行拉花动作

  if result != MoveItErrorCode::SUCCESS:
    res.success = false, res.message = "execute failed: error_code=X"
    return

  // 执行成功 → 再次发布轨迹到 RViz2 (展示最终完成的轨迹)
  visual_tools_->publishTrajectoryPath(traj, robot_state, false)
  visual_tools_->trigger()

  res.success           = true
  res.fraction          = fraction
  res.trajectory_points  = traj.points.size()
  res.message           = "ok: fraction=X%, Y pts"
```

### 3.9 异常退出

```
  catch (std::exception& e):
    res.success = false
    res.message = "exception: " + e.what()

  finally:
    busy_ = false   // 释放防重入锁
```

### 3.10 完整时序

```
Client                  LatteNode                   MoveIt2              RViz2
  │                        │                           │                    │
  ├─ replay_traj ─────────→│                           │                    │
  │                        ├─ ① loadLat/generatePattern │                   │
  │                        ├─ ② retarget                │                   │
  │                        ├─ ③ computeCartesianPath ──→│                   │
  │                        │   RobotTrajectory ←────────┤                   │
  │                        ├─ ④ publishTrajectoryPath ────────────────────→│
  │   (preview)            │                           │  robot ghost + EE │
  │ ←── {waypoints} ──────┤                           │                    │
  │                        │                           │                    │
  │   (action)             ├─ ⑤ safety check            │                   │
  │                        ├─ ⑥ execute ──────────────→│                   │
  │                        │   MoveItErrorCode ←───────┤  robot motion     │
  │                        ├─ publishTrajectoryPath ──────────────────────→│
  │ ←── {success} ────────┤                           │  final EE path    │
```

---

## 4. latte_math.hpp API 清单

```cpp
namespace latte_math {

// ── 数据结构 ──
struct CartesianTrajectory {
    std::vector<Eigen::Vector3d> positions;      // N 个
    std::vector<Eigen::Vector4d> orientations;   // N 个 (xyzw, 可空)
    double dt;
    std::string frame_id;                       // "base_link"
    double totalLength() const;
    std::vector<geometry_msgs::msg::Pose> toPoses(int step) const;
};

// ── .lat IO ──
std::optional<CartesianTrajectory> readLat(const std::string& path);
bool writeLat(const std::string& path, const CartesianTrajectory& cart);

// ── 四元数 (Hamilton xyzw, 与 ROS tf2 setRPY 一致) ──
Eigen::Matrix3d quatToRot(const Eigen::Vector4d& q);
Eigen::Vector4d rotToQuat(const Eigen::Matrix3d& R);
Eigen::Vector4d quatMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);
Eigen::Vector4d eulerDegToQuat(double roll, double pitch, double yaw);

// ── SE(3) Retarget ──
// 线路A: 完整朝向保留
CartesianTrajectory retargetTrajectory(
    const CartesianTrajectory& cart,
    const Eigen::Isometry3d& target,
    const Eigen::Vector3d& rpy_user_deg);

// 线路B: 位置完整R, 朝向仅yaw (保留pitch技能)
CartesianTrajectory retargetWithOrientationConstraint(
    const CartesianTrajectory& cart,
    const Eigen::Isometry3d& target,
    const Eigen::Vector3d& rpy_user_deg);

// ── 图案生成 (线路B, ~80行) ──
CartesianTrajectory generatePattern(
    const std::string& type,       // "heart"|"rosetta"|"tulip"|"swan"
    double cup_radius, double surface_z,
    double mix_h, double draw_h, double finish_h,
    double wiggle_amp, double wiggle_freq);

// ── 安全 ──
struct SafetyResult { bool safe; std::string msg; };
SafetyResult checkWorkspaceBounds(
    const CartesianTrajectory& cart,
    double x_min, double x_max, double y_min, double y_max,
    double z_min, double z_max);

} // namespace latte_math
```

---

## 5. latte_node.hpp

```cpp
class LatteNode : public rclcpp::Node {
public:
    explicit LatteNode(const rclcpp::NodeOptions& opts);
    void init();   // MoveGroup + MoveItVisualTools (延后 shared_from_this)

private:
    rclcpp::Service<ReplayLatteTrajectory>::SharedPtr srv_;
    void handleReplay(Request req, Response res);

    geometry_msgs::msg::Pose resolveTarget(const Request& req) const;

    // 4 个成员
    std::shared_ptr<MoveGroupInterface>        move_group_;
    std::shared_ptr<MoveItVisualTools>         visual_tools_;      // + loadTrajectoryPub
    std::atomic<bool> busy_{false};

    std::string planning_group_, base_frame_, ee_link_;
    WorkspaceSafetyConfig workspace_;
};
```

---

## 6. 删除清单

```
❌ latte_imitation/              Python ROS 包 (全部)
❌ latte_cartesian_planner/      C++ Plan+Execute 包 (合并)
❌ /latte/plan_and_execute       废弃服务
```

## 7. 保留/新增清单

```
✅ latte_art/                    Python 离线工具 (加 writeLat 导出)
✅ ivg_interfaces/               ReplayLatteTrajectory.srv (不变)
✅ scripts/convert_npz_to_lat.py  npz → .lat 一次性转换
➕ latte_node/                   新 C++ 包 (5 源文件)
```

## 8. 两线路对比

| | 线路 A: 录制回放 | 线路 B: 参数化 |
|------|------|------|
| 数据源 | .lat 文件 (npz 转换) | C++ 实时生成 (Eigen) |
| 朝向 | .lat 中已包含录制朝向 | C++ 生成 pitch 剖面 |
| Retarget | `retargetTrajectory` (完整R) | `retargetWithOrientationConstraint` (仅yaw) |
| Target | TF 当前位姿 / 手动 | lizhu_link + surface_z |
| MoveIt2 | computeCartesianPath | computeCartesianPath (相同) |
| 依赖 | 无 | 无 (全部 Eigen 内联) |

## 9. 与官方模式对照

| 官方教程 / API | 本项目对应 |
|------|------|
| `MoveGroupInterface(node, group)` | `move_group_` 成员 |
| `MoveItVisualTools(node, frame, topic, model)` | `visual_tools_` 成员 |
| `loadMarkerPub()` + `loadTrajectoryPub()` | `init()` |
| `deleteAllMarkers()` + `trigger()` | 每次 publish 前后 |
| `publishTrajectoryPath(traj, robot_state)` | ③ Preview + ⑥ Execute 后 |
| `publishTrajectoryPath` → `/move_group/display_planned_path` | RViz MotionPlanning "Planned Path" display |
| `automatically_declare_parameters_from_overrides(true)` | `main.cpp` |
| `shared_from_this()` 不在构造函数 | `init()` 延后 |
