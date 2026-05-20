# IVG2.0 零基础前置知识学习文档

> **适用对象**: 完全零基础，想看懂并亲手写出本项目的每一行代码
> **目标**: 按本文档顺序学习，达到能从零编写 IVG2.0 全部组件的能力
> **时间估算**: 全日制 6-8 周，业余 12-16 周
>
> **与部署文档的区别**: 本文档教你"为什么要这么写"，[ZERO-BASIS-REPLICATION.md](ZERO-BASIS-REPLICATION.md) 教你"具体怎么写每一行"
>
> 依据: COLON 全部源码、`DEPLOYMENT.md`、`docs/architecture.md`、`CLAUDE.md`

---

## 学习路径总览

```
阶段 1: Linux + C++ + CMake 基础          (7天)  ← 写 aubo_driver_ros2 的必备
阶段 2: ROS 2 核心编程                     (7天)  ← 所有包的通信基础
阶段 3: MoveIt 2 编程接口                  (5天)  ← demo_driver / tool_changer 的核心
阶段 4: URDF / XACRO 机器人建模            (3天)  ← aubo_description / aubo_moveit_config
阶段 5: Python + NumPy + 科学计算          (3天)  ← Python 包的基础
阶段 6: 计算机视觉与深度学习推理            (5天)  ← VPE / GraspNet / YOLO
阶段 7: Vue 3 + TypeScript 前端            (7天)  ← web/src/ 全部代码
阶段 8: ivg_interfaces 接口设计             (1天)  ← 理解 52 个自定义类型的设计思路
阶段 9: AUBO SDK 专属编程                   (2天)  ← 双连接 / RIB / LifecycleNode
```

每阶段列出 **必须掌握的技能**（标 ⭐）、**对应本项目代码**、**练习建议**

---

## 阶段 1: Linux + C++ + CMake 基础

> 本项目有 ~5000 行 C++ 代码（aubo_driver_ros2 + demo_driver + tool_changer），需要扎实的 C++ 和 CMake 基础

### 必须掌握的技能 ⭐

| 技能 | 为什么需要 | 对应本项目代码 |
|------|-----------|-------------|
| C++ 类、继承、多态 | 所有 C++ 节点都是继承 `rclcpp::Node` 的类 | `class GripperSwapWorker : public rclcpp::Node` |
| 智能指针 `shared_ptr` / `weak_ptr` | ROS 2 全面使用智能指针管理节点生命周期 | `std::make_shared<RobotController>(this)` |
| `enable_shared_from_this` 陷阱 | 构造函数中调用 `shared_from_this()` 报 `bad_weak_ptr` | `GripperSwapWorker::create()` 工厂方法模式 |
| STL 容器 (map/vector/array) | 大量使用 `std::map` / `std::vector` | `std::map<std::string, ToolConfig> tool_configs_` |
| lambda 表达式 | ROS 2 回调大量使用 lambda | `[this](auto& msg) { onToolStatus(msg); }` |
| CMake 基础: `find_package` / `add_executable` / `target_link_libraries` | 每个包都有 CMakeLists.txt | `ament_target_dependencies(gripper_swap_worker_node rclcpp ...)` |
| CMake 预编译库链接 | AUBO SDK 是预编译 `.so` | `target_link_libraries(aubo_dashboard_node ${AUBO_SDK_LIBS})` |
| `std::atomic` / 多线程安全 | SDK 回调线程与 ROS 2 线程间的数据安全 | `std::atomic<bool> shutdown_requested_{false}` |
| `sleep` / `chrono` 时间控制 | 运动控制中的等待与超时 | `std::chrono::milliseconds(200)` |

### 练习建议

1. 写一个 C++ 类，包含 `shared_ptr` 工厂方法 `static std::shared_ptr<MyClass> create()` 模式
2. 写一个 CMakeLists.txt 编译多个可执行文件并链接外部 `.so` 库
3. 写一个使用 `std::map` 从 YAML 文件加载配置的示例

### 参考资源

- 《C++ Primer》第 12 章（智能指针）、第 16 章（模板与泛型）
- CMake 官方教程: https://cmake.org/cmake/help/latest/guide/tutorial/

---

## 阶段 2: ROS 2 核心编程（C++ 和 Python）

> ROS 2 是项目的"神经系统"，所有组件通过它通信。这是**最重要的阶段**，需要同时掌握 C++ 和 Python 两套 API

### C++ ROS 2 编程 ⭐

| 技能 | 对应本项目代码 |
|------|-------------|
| 继承 `rclcpp::Node` 创建节点 | `class GripperSwapWorker : public rclcpp::Node` |
| `create_publisher<T>("topic", qos)` | `create_publisher<ivg_interfaces::msg::ToolChangerStatus>("/tool_changer_status", 10)` |
| `create_subscription<T>("topic", qos, callback)` | `create_subscription<std_msgs::msg::String>("/aubo/mode", ...)` |
| `create_service<T>("service", callback)` | `create_service<ivg_interfaces::srv::ChangeTool>("/change_tool", ...)` |
| `create_client<T>("service")` + `async_send_request` | `set_io_client_->async_send_request(request)` |
| `create_wall_timer(interval, callback)` | `create_wall_timer(std::chrono::seconds(5), ...)` |
| **CallbackGroup**: `MutuallyExclusive` vs `Reentrant` | `create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)` |
| **MultiThreadedExecutor** | `rclcpp::executors::MultiThreadedExecutor exec(options, 2)` |
| `shared_from_this()` 的正确使用时机 | 不能在构造函数中调用！延后到 `create()` 工厂方法中 |
| **QoS 配置**: `transient_local` / `reliable` / depth | `rclcpp::QoS(10).transient_local()` |

### Python ROS 2 编程 ⭐

| 技能 | 对应本项目代码 |
|------|-------------|
| 继承 `Node` 创建节点 | `class VisualPoseEstimationNode(Node)` |
| `create_publisher` / `create_subscription` | 与 C++ 类似，参数更 Pythonic |
| `create_service` + 回调 | `self.create_service(EstimatePose, '/estimate_pose', self._handle_estimate_pose)` |
| `self.declare_parameter` + `add_on_set_parameters_callback` | 参数事件回调模式 |
| `rclpy.spin(node)` / `MultiThreadedExecutor` | Python 也要配 Executor 避免阻塞 |
| `cv_bridge` 图像转换 | `bridge.cv2_to_imgmsg(cv_image, 'bgr8')` |
| `tf2_ros.Buffer` + `TransformListener` | TF 变换查询 |

### 关键知识点深入

#### Callback Group 死锁规则（必读！）

```
场景: 在 MutuallyExclusive callback group 的回调中，
调用 client->async_send_request(request).wait_for(timeout)
→ done-callback 继承了同一个 MutuallyExclusive 组的锁
→ 但该组当前正在执行原回调（等待 done-callback 返回）
→ 永久阻塞 = 死锁！

正确:
  service_cb_group_ = MutuallyExclusive  ← 服务回调
  client 调用放在独立线程或另一个 Reentrant 组中

本项目中:
  gripper_swap_worker : MutuallyExclusive + MultiThreadedExecutor(2)
  execute_grasp_pose_worker : Reentrant 组
```

#### ROS 2 参数隔离（与 ROS 1 完全不同）

```
ROS 1: rosparam 是全局参数服务器，所有节点共享
ROS 2: 每个节点各自维护独立参数副本

本项目关键陷阱:
  robot_state_publisher 不订阅 /robot_description 话题！
  只监听 /parameter_events 中自身参数的变更。
  
  → 要触发 setupURDF() 重建 TF 树，必须用 set_parameters()
  → 用 publish() 发送 /robot_description 消息不会生效！
```

### 练习建议

1. 写一对 C++ Publisher/Subscriber，发布 `/joint_states`
2. 写一个 C++ Service Server + Client，传入关节角返回 FK 结果
3. 写一个带 Action Server 的节点（仿照 JointTrajectoryController）
4. 用 Python 写同样的 Publisher/Subscriber/Service
5. 写一个 Python 节点用 `cv_bridge` 从 ROS Image 话题订阅图像并保存为文件

### 参考资源

- ROS 2 Humble 教程: https://docs.ros.org/en/humble/Tutorials.html（Beginner + Intermediate 全部完成）
- Callback Groups: https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html
- 参数机制设计: https://design.ros2.org/articles/ros_parameters.html
- LifecycleNode: https://design.ros2.org/articles/node_lifecycle.html
- rclcpp 源码: https://github.com/ros2/rclcpp/tree/humble

---

## 阶段 3: MoveIt 2 C++ 编程接口

> demo_driver（11 个 C++ 服务节点）和 tool_changer 全面使用 MoveGroupInterface 和 PlanningSceneInterface

### 必须掌握的 API ⭐

| API | 用途 | 本项目使用 |
|-----|------|----------|
| `MoveGroupInterface("manipulator")` | 创建运动规划组 | robot_controller.cpp |
| `setJointValueTarget(joints)` | 设定目标关节角 | `moveToJoints()` |
| `setPoseTarget(pose)` | 设定目标 TCP 位姿 | `moveToTargetXYZ()` |
| `move()` | 执行规划并运动（阻塞） | `moveToHome()` |
| `computeCartesianPath(waypoints, eef_step, jump_threshold)` | 笛卡尔空间直线路径 | `runCartesianPath()` |
| `setMaxVelocityScalingFactor(v)` / `setMaxAccelerationScalingFactor(a)` | 速度/加速度倍率 | 每次运动前 |
| `getCurrentPose()` / `getCurrentJointValues()` | 获取当前状态 | 状态查询服务 |
| `PlanningSceneInterface` | 管理碰撞场景 | scene_attach_worker |
| `AttachedCollisionObject` | 附着在连杆上的物体 | tools.yaml → ACO ADD/REMOVE |

### 关键概念

#### computeCartesianPath 返回值必须检查

```cpp
double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
// fraction: 0.0 ~ 1.0，表示路径完成比例
if (fraction < 0.95) {
    // 路径不完整，不要执行！
    RCLCPP_WARN(logger, "Cartesian path only %.1f%% complete", fraction * 100);
}
```

#### jump_threshold 跳变检测

```
jump_threshold = 0.0: 禁用跳变检测（本项目设置）
jump_threshold > 0.0: 
  |Δjoint_i| < jump_threshold × mean(|Δjoint|) → 判定为跳变
  跳变→警告，可能导致 IK 解不连续
```

#### `move()` 是阻塞的

```
需要 async spinner 才能让 move() 正常完成
否则 move() 内部等待的 done-callback 永远不会被调用
→ 死锁！
```

### 练习建议

1. 用 MoveGroupInterface 写一个 C++ 程序：设定 3 个航点 → `computeCartesianPath` → 画三角形
2. 给程序加上速度缩放因子参数
3. 写一个 PlanningScene 演示：添加碰撞物体 → 检测碰撞 → 移除

### 参考资源

- MoveGroupInterface API: https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html
- CartesianInterpolator: https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html
- Planning Scene 教程: https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html

---

## 阶段 4: URDF / XACRO 机器人建模

> 理解 `aubo_description/urdf/aubo_e5_10.urdf`（~400行）和 `aubo_moveit_config/config/aubo_e5.urdf.xacro`（~110行）

### 必须掌握 ⭐

| 概念 | 本项目体现 |
|------|----------|
| `<link>` 的定义（inertial/visual/collision） | 每个连杆有 3 个子元素 |
| `<joint>` 类型（revolute/continuous/fixed/prismatic） | 7 个 revolute 关节 + 多个 fixed 关节 |
| `origin` (xyz + rpy) | 连杆间的空间关系 |
| `<xacro:macro>` 参数化 | `gripper_link` 宏动态切换 5 种工具 |
| `<xacro:include>` 组合 | 基础 URDF + XACRO 宏 |
| `package://` 路径 | `package://aubo_description/meshes/collision/base_link.stl` |
| ros2_control `<transmission>` | 6 个传动定义 |
| `<ros2_control>` 硬件接口标签 | 仿真模式的硬件抽象 |

### 本项目的关键设计

```
TF 树结构:
world (URDF 根)
  └── base_link (fixed joint, identity)
        └── shoulder_joint → shoulder_Link
              └── upperArm_joint → upperArm_Link
                    └── foreArm_joint → foreArm_Link
                          └── wrist1_joint → wrist1_Link
                                └── wrist2_joint → wrist2_Link
                                      └── wrist3_joint → wrist3_Link
                                            ├── camera_Link (fixed, 眼在手上)
                                            ├── kuaihuan_Link (fixed, 快换法兰)
                                            │     └── {gripper}_link (fixed, 动态切换)
                                            └── tool_tcp (fixed, TCP 标记)

注意: base_link = world (URDF identity fixed joint)
→ 所有坐标都是 base_link 坐标系，TF 树 root 也是 base_link
```

### 练习建议

1. 手写一个 2 连杆臂的 URDF（base → joint1 → link1 → joint2 → link2）
2. 用 XACRO 宏参数化连杆长度
3. 用 `check_urdf` 工具验证

### 参考资源

- URDF XML 规范: http://wiki.ros.org/urdf/XML
- XACRO 文档: http://wiki.ros.org/xacro
- ros2_control URDF 标签: https://control.ros.org/humble/doc/ros2_control/ros2_control_doc/doc/hardware_components_userdoc.html

---

## 阶段 5: Python + NumPy + 科学计算

> 本项目 Python 包大量使用 NumPy 做矩阵/向量运算、四元数操作、点云处理

### 必须掌握 ⭐

| 技能 | 本项目使用 |
|------|----------|
| NumPy 数组操作 (`np.array` / reshape / 广播) | `ivg_utils/math.py` 全部函数 |
| 旋转矩阵 ↔ 四元数 ↔ 欧拉角 | `rotation_matrix_to_quaternion()` 等 |
| 齐次变换矩阵 4×4 | `pose_to_matrix()` / `matrix_to_pose()` |
| 四元数 Hamilton 约定 (xyzw) | `euler_deg_to_quat()` = 内旋 ZYX = 外旋 XYZ |
| SciPy 优化 (`least_squares`) | 手眼标定的非线性优化 |
| OpenCV 图像操作 (`cv2.imread` / 颜色空间 / 轮廓) | VPE 模板匹配 |
| YAML 解析 (`PyYAML`) | 所有配置文件 |
| `ament_index_python` 路径查找 | 运行时定位包内资源文件 |

### 关键陷阱

```python
# numpy 版本陷阱！
# 本项目必须用 numpy 1.23.5，不能用 2.x！
# numpy 2.x 移除了 np.float 别名 → transforms3d 报废
# cv_bridge (ROS Humble) 与 numpy 2.x 二进制不兼容

# 四元数约定陷阱！
# ROS 2 tf2 使用 Hamilton 四元数 (xyzw)
# 非 JPL (wxyz)！
# setRPY(r, p, y) 等价于 euler_deg_to_quat(r, p, y)
# 内旋 ZYX = 外旋 XYZ
```

### 练习建议

1. 实现 `ivg_utils/math.py` 中的全部函数（旋转矩阵/四元数/欧拉角互转）
2. 用 OpenCV 写模板匹配程序
3. 写一个 YAML 配置加载器（仿照 `tools.yaml` 的加载模式）

### 参考资源

- NumPy 官方教程: https://numpy.org/doc/stable/user/quickstart.html
- OpenCV Python 教程: https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html
- 四元数可视化: https://eater.net/quaternions

---

## 阶段 6: 计算机视觉与深度学习推理

> VPE（姿态估计）、GraspNet（抓取预测）、YOLO OBB（目标检测）三项视觉任务

### VPE: 基于模板匹配的工件姿态估计 ⭐

| 概念 | 本项目实现 |
|------|----------|
| 背景去除 (rembg) | `subprocess_rembg.py` – AI 自动抠图 |
| Canny 边缘检测 + 轮廓提取 | OpenCV 经典流程 |
| 模板匹配 (matchTemplate) | 滑窗匹配，找出最佳位置 |
| 深度图 → 3D 坐标 | `z * scale → (u-cx)*z/fx, (v-cy)*z/fy` |
| ICP 精配准 | 可选的精化步骤 |

### GraspNet: 6-DoF 抓取位姿预测 ⭐

| 概念 | 本项目实现 |
|------|----------|
| PointNet++ 点云编码 | `pointnet2/` CUDA 扩展 |
| VCoT-Grasp 模型 | `graspnet-baseline/logs/log_kn/checkpoint-rs.tar` |
| 场景点云输入 (20000点 → 采样) | `num_view=300` |
| 碰撞检测 (ModelFreeCollisionDetector) | `collision_thresh=0.01m` |
| NMS (非最大抑制) | `max_grasps_num=5` |
| Z 轴 180° 翻转修正 | `applyGraspZFlip180()` – GraspNet 输出 Z 轴与夹爪物理 Z 轴相反 |

### YOLO OBB: 旋转框目标检测 ⭐

| 概念 | 本项目实现 |
|------|----------|
| Ultralytics YOLO 推理 | `model = YOLO('yolo26n-obb.pt')` |
| OBB (Oriented Bounding Box) | 输出 5 参数: x, y, w, h, angle |
| ROS 话题发布 | 检测结果 → `MarkerArray` + JSON string |

### 练习建议

1. 用 OpenCV 实现完整的模板匹配 pipeline（读图 → 灰度 → 边缘 → 匹配 → 标注）
2. 在 GPU 上运行 GraspNet 推理，理解点云输入 → 抓取位姿输出的完整流程
3. 用 YOLO 跑一次 OBB 推理，输出标注图像

### 参考资源

- GraspNet 论文: https://arxiv.org/abs/2006.02976
- PointNet++ 论文: https://arxiv.org/abs/1706.02413
- Ultralytics OBB: https://docs.ultralytics.com/tasks/obb/
- OpenCV 模板匹配: https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html

---

## 阶段 7: Vue 3 + TypeScript + Vite 前端

> `web/src/` 下 ~5000 行代码，Vue 3 SPA + rosbridge WebSocket 通信 + Three.js 3D 渲染

### 必须掌握的技能 ⭐

| 技能 | 对应本项目代码 |
|------|-------------|
| Vue 3 Composition API (`<script setup>`) | 所有 `.vue` 文件 |
| `ref` / `reactive` / `shallowRef` / `computed` / `watch` | `useRos.ts` 中 `connected = ref(false)` |
| **`shallowRef` 陷阱**: ROSLIB.Ros 不能放入响应式系统 | Ros 对象是模块级单例，不用 ref 包装 |
| TypeScript 类型定义 (interface/type) | `constants/ros.ts` 中的类型 |
| Vue Router (`createWebHistory`) | `router/index.ts` 6 条路由 |
| Composable 模式 (`useXxx`) | `useRos` / `useRosTopic` / `useRuntime` 等 14 个 |
| **Composable 生命周期自动清理** | `onUnmounted` 中 `unsubscribe()` |
| Tailwind CSS v4 原子类 | `class="flex gap-4 p-4"` |
| Element Plus 组件库 | `el-button` / `el-tag` / `el-dialog` 等 |
| Vite 配置 (proxy / alias / plugins) | `vite.config.ts` |
| Three.js 3D 渲染 | `SceneManager.ts` / `UrdfModel.ts` |
| **MeshPhongMaterial** (与 RViz2 OGRE Phong 一致) | `UrdfModel.ts` |
| **ColladaLoader** + `<phong>` 材质保留 | 仅覆盖 `.color` (diffuse)，保留 specular/shininess |

### rosbridge JSON 协议（核心通信机制）

```
浏览器 ←→ WebSocket ws://IP:8090/ws/rosbridge ←→ rosbridge (:9090) ←→ ROS 2 DDS

协议操作:
  { "op": "subscribe",   "topic": "/joint_states", "type": "sensor_msgs/JointState" }
  { "op": "call_service", "service": "/change_tool", "args": {"tool_id": "gripper0"} }
  { "op": "send_action_goal", ... }

不使用 rosbridge 的:
  话题发布: 本项目前端只订阅不发布 ROS 话题（纯消费者）
  参数设置: 通过 call_service 调用 /node/set_parameters
```

### 本项目的架构原则

```
前端 (Vue 3) → 纯显示层，不做业务计算
BFF (FastAPI) → 静态文件 + WebSocket代理 + 视频代理
  ❌ 不允许: BFF 内 import rclpy、BFF 做轨迹计算、BFF 持有 TF 树
ROS 2 节点 → 所有业务逻辑（参数/服务/话题/Action）
AUBO SDK → 硬件通信
```

### 练习建议

1. 用 Vue 3 + Vite 写一个带路由的 SPA（3 个页面）
2. 写一个 `useRos` composable（建立 rosbridge 连接 + 订阅话题 + 调用服务）
3. 用 Three.js 渲染一个 URDF 机器人模型（`UrdfModel.ts` 的实现思路）

### 参考资源

- Vue 3 官方文档: https://cn.vuejs.org/guide/introduction.html
- Vite 官方文档: https://cn.vitejs.dev/guide/
- Tailwind CSS: https://tailwindcss.com/docs
- rosbridge 协议: https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md
- Three.js 文档: https://threejs.org/docs/

---

## 阶段 8: ivg_interfaces 接口设计思路

> 理解 52 个自定义 ROS 2 类型的分类思想和设计模式

### 分类体系

```
msg (17 个):
  机械臂底层 (8): Analog, Digital, IOStates, JointTrajectoryFeedback, MasterboardDataMsg,
                  RobotModeDataMsg, RobotStateRTMsg, ToolDataMsg
  机械臂高层 (6): RobotStatus, RobotIOStatus, ToolIOStatus, ImageData, NodeStatus, SystemLog
  相机 (1):       CameraStatus
  工具快换 (1):    ToolChangerStatus
  视觉 (1):       CartesianPosition

srv (35 个):
  运动控制 (12):   MoveToPose, PlanTrajectory, ExecuteTrajectory, MoveJoint, MoveLine,
                  Movel (@deprecated), SetRobotPose, GetCurrentState, SetSpeedFactor,
                  SetRobotEnable, SetCollisionClass, TeachStart (@deprecated)
  IO 操作 (3):     SetRobotIO（高层封装）、SetIO（底层常量枚举）、ReadRobotIO
  运动学 (2):      GetFK, GetIK
  工具快换 (3):    RunGripperSwap, ChangeTool, GetCurrentTool
  视觉位姿 (7):    EstimatePose, EstimatePose2D, ListTemplates, StandardizeTemplate,
                  UpdateParams, ProcessDebugStep, VisualizeGraspPose
  相机控制 (2):    SetCameraParameters, SoftwareTrigger
  工具配置 (4):    SetPayload (@deprecated), SetToolKinematics (@deprecated),
                  SetToolVoltage (@deprecated), SetSpeedSliderFraction (@deprecated)
  执行控制 (2):    ExecuteGraspPose, ReplayLatteTrajectory
```

### 设计模式

```xml
<!-- .srv 标准格式: 请求字段在上，响应字段在下，用 --- 分隔 -->
string tool_id         # 请求 → 要切换到哪个工具
---
bool success           # 响应 → 是否成功
string message         # 响应 → 详细信息
```

### 练习建议

1. 为自己的机器人项目设计 5 个 msg + 5 个 srv
2. 写一个 `ament_cmake` 包包含这些接口
3. 在其他包中使用这些接口（`find_package` + `#include`）

---

## 阶段 9: AUBO SDK 专属编程

> 本项目特有的 AUBO E5 硬件编程知识，与其他机械臂平台不同

### 关键知识 ⭐

| 知识 | 说明 |
|------|------|
| SDK 双连接: `conn_control_` (TCP2CAN+RIB) + `conn_status_` (状态+IO) | RIB 必须在同一条 TCP 连接上读写 |
| SDK 同步阻塞 (2-225ms/调用) | 不能放 ROS 2 实时循环，必须线程隔离 |
| SDK 回调线程 | 回调内不能调用 SDK API，只能 atomic 写入 + 日志 |
| TCP2CAN 流式发送 | 固定 5ms 间隔，超时丢弃当前点 |
| IO 引脚: `io_index + 32 = 硬件引脚` | 夹爪=6→38，快换=7→39 |
| Dashboard LifecycleNode: `Unconfigured→Inactive→Active` | `Inactive` 状态下 service 调用直接失败 |
| 20 个 SDK Service | getIK / getFK / setIO / setPayload / teachStart 等 |
| RIB 环形缓冲区 | 周期性查询诊断信息 |
| OTG (在线轨迹生成) | 五次多项式插值，stime 控制 |

### 练习建议

1. 写一个简单的 SDK 客户端：连接 `169.254.10.98:8899`，调用 getIK 和 getFK
2. 实现 `sdk_mutex_` 串行化所有 SDK 调用
3. 在 ROS 2 回调中通过原子变量发布 SDK 数据（不解引用 SDK 对象）

---

## 学习资源汇总

### 官方文档完整索引

| 领域 | 资源 |
|------|------|
| ROS 2 Humble | https://docs.ros.org/en/humble/ |
| MoveIt 2 Humble | https://moveit.picknik.ai/humble/index.html |
| ros2_control | https://control.ros.org/humble/index.html |
| Vue 3 | https://cn.vuejs.org/ |
| Vite | https://cn.vitejs.dev/ |
| TypeScript | https://www.typescriptlang.org/docs/ |
| CMake | https://cmake.org/cmake/help/latest/ |
| OpenCV Python | https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html |
| PyTorch | https://pytorch.org/tutorials/ |
| Three.js | https://threejs.org/docs/ |

### 本项目文档索引

| 文档 | 用途 |
|------|------|
| `CLAUDE.md` | 完整知识点速查、常见报错、架构规则 |
| `docs/architecture.md` | 6 层架构蓝图、数据流、设计决策 |
| `docs/PROCESS-FLOW.md` | 16 步启动流程、调试命令 |
| `docs/frontend-migration-plan.md` | 前端技术栈细节 |
| `DEPLOYMENT.md` | 完整部署步骤和版本号 |
| `aubo_ros2_ws/VERSIONS.md` | 精确版本清单 |
| `aubo_ros2_ws/docs/ZERO-BASIS-REPLICATION.md` | 逐包从零编写代码 |

---

*最后更新: 2026-05-20*
