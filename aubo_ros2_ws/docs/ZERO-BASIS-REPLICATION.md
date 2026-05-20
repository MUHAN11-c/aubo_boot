# IVG2.0 零基础从零复刻实现文档

> **适用对象**: 零基础，想亲手写出本项目的每一行代码
> **目标**: 从空的工作空间开始，逐包逐文件创建完整的 IVG2.0 系统
> **时间估算**: 按本文档顺序逐包实现，全日制 4-6 周
>
> **与部署文档的区别**: 本文档教你"怎么写每一行代码"，[DEPLOYMENT.md](../../DEPLOYMENT.md) 教你"怎么部署已有代码"
>
> 依据: 全部 `src/` 下源码、`CMakeLists.txt`、`package.xml`、`config/*.yaml`、`launch/*.py`
>
> **前置知识**: 建议先完成 [ZERO-BASIS-PREREQUISITES.md](ZERO-BASIS-PREREQUISITES.md) 中的学习路径

---

## 复刻总览

按依赖顺序逐包创建（箭头表示"依赖"）：

```
Part 1: ivg_interfaces     ← 最先：所有包的接口基础
  └→ Part 2: ivg_utils       ← Python 工具库（零 ROS 依赖）
  └→ Part 3: aubo_description ← URDF 机器人模型
      └→ Part 4: aubo_driver_ros2    ← C++ SDK 驱动层
          └→ Part 5: aubo_moveit_config ← MoveIt2 配置
              └→ Part 6: demo_driver        ← C++ 应用服务层
                  └→ Part 7: tool_changer       ← C++ 工具快换
                      └→ Part 8: graspnet_ros2      ← Python GraspNet
                      └→ Part 9: visual_pose_estimation  ← Python VPE
                      └→ Part 10: latte_imitation      ← Python 轨迹重定目标
                      └→ Part 11: coffee_latte_demo     ← Python IO 控制
                      └→ Part 12: percipio_camera       ← 相机驱动
                      └→ Part 13: aubo_ros2_web_dashboard ← Web 全栈
```

---

## Part 0: 创建工作空间

```bash
# 创建 ROS 2 工作空间
mkdir -p ~/ivg_ws/src
cd ~/ivg_ws

# 安装 ROS 2 Humble 和基础依赖（完整步骤见 DEPLOYMENT.md）
source /opt/ros/humble/setup.bash

# 工作空间目录结构
# src/              ← 所有包的源码
#   ivg_interfaces/
#   ivg_utils/
#   aubo_description/
#   aubo_driver_ros2/
#   ... 等 13 个包

# 首次编译：先编译 ivg_interfaces（其他包的依赖）
colcon build --packages-select ivg_interfaces
source install/setup.bash
# 之后每次创建新包后全量编译
colcon build
source install/setup.bash
```

---

## Part 1: ivg_interfaces — 统一自定义接口包

### 1.1 架构概览

**职责**: 定义项目中所有自定义 ROS 2 消息（msg）和服务（srv），**52 个类型**（17 msg + 35 srv）。这个包必须是 `ament_cmake` 类型，因为 ROS 2 的 `.msg`/`.srv` 需要通过 `rosidl_default_generators` 生成 C++/Python 代码。

**设计决策**: 本项目将原先分散在 5 个旧包（`aubo_msgs`、`demo_interface`、`percipio_camera_interface`、`tool_changer_interface`、`interface`）中的接口全部合并到一个包，统一命名前缀 `ivg_interfaces`。

### 1.2 需要创建的文件

```
ivg_interfaces/
├── CMakeLists.txt          ← 声明 52 个接口 + 依赖项
├── package.xml             ← 包元数据 + ament_cmake 类型
├── msg/                    ← 17 个消息定义
│   ├── RobotStatus.msg     ← 机械臂综合状态（最重要）
│   ├── ToolChangerStatus.msg
│   ├── IOStates.msg
│   ├── RobotIOStatus.msg
│   ├── CartesianPosition.msg
│   ├── JointTrajectoryFeedback.msg
│   ├── CameraStatus.msg
│   ├── SystemLog.msg
│   ├── NodeStatus.msg
│   ├── ImageData.msg
│   ├── RobotStateRTMsg.msg
│   ├── RobotModeDataMsg.msg
│   ├── MasterboardDataMsg.msg
│   ├── ToolDataMsg.msg
│   ├── ToolIOStatus.msg
│   ├── Analog.msg
│   └── Digital.msg
└── srv/                    ← 35 个服务定义
    ├── ChangeTool.srv      ← 工具切换
    ├── RunGripperSwap.srv  ← 快换操作
    ├── GetCurrentTool.srv
    ├── ExecuteGraspPose.srv
    ├── EstimatePose.srv    ← 姿态估计
    ├── SetRobotIO.srv
    ├── GetIK.srv
    ├── GetFK.srv
    ├── MoveToPose.srv
    ├── PlanTrajectory.srv
    ├── ExecuteTrajectory.srv
    ├── GetCurrentState.srv
    ├── SetSpeedFactor.srv
    ├── SetRobotEnable.srv
    ├── SetRobotPose.srv
    ├── ReadRobotIO.srv
    ├── MoveJoint.srv
    ├── MoveLine.srv
    ├── Movel.srv           ← @deprecated 保留
    ├── SetIO.srv           ← 带常量枚举的底层 IO
    ├── SetPayload.srv
    ├── SetSpeedSliderFraction.srv
    ├── SetCollisionClass.srv
    ├── SetToolKinematics.srv
    ├── SetToolVoltage.srv
    ├── TeachStart.srv
    ├── SoftwareTrigger.srv
    ├── SetCameraParameters.srv
    ├── EstimatePose2D.srv
    ├── ListTemplates.srv
    ├── StandardizeTemplate.srv
    ├── UpdateParams.srv
    ├── ProcessDebugStep.srv
    ├── VisualizeGraspPose.srv
    └── ReplayLatteTrajectory.srv ← 拉花轨迹回放
```

### 1.3 关键实现模式

#### package.xml

```xml
<package format="3">
  <name>ivg_interfaces</name>
  <version>1.0.0</version>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <!-- 生成器依赖: 将 .msg/.srv 编译成 C++/Python 代码 -->
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <!-- ROS 2 标准类型依赖: 本项目消息依赖 builtin/标准类型 -->
  <depend>builtin_interfaces</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>sensor_msgs</depend>
  <!-- 运行时只需要 rosidl_default_runtime -->
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

#### CMakeLists.txt 核心结构

```cmake
cmake_minimum_required(VERSION 3.5)
project(ivg_interfaces)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 消息依赖的标准包
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 声明所有 52 个接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/ToolChangerStatus.msg"
  # ... 共 17 msg
  "srv/ChangeTool.srv"
  "srv/RunGripperSwap.srv"
  # ... 共 35 srv
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs trajectory_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

#### msg 定义模式

```python
# RobotStatus.msg — 机械臂综合状态 (~50Hz)
std_msgs/Header header                    # 时间戳 + frame_id
bool is_online                            # 在线状态
bool enable                               # 使能状态
bool in_motion                            # 运动中
string planning_status                    # idle/planning/executing/error
float64[6] joint_position_rad             # 固定长度数组: 6 个关节
float64[6] joint_position_deg
geometry_msgs/Pose cartesian_position     # 嵌套 ROS 2 标准类型
```

#### srv 定义模式

```python
# ChangeTool.srv — 切换工具
string tool_id         # ─── 请求字段 ───
---
bool success           # ─── 响应字段 ───
int32 error_code
string message
```

**编译验证**:
```bash
colcon build --packages-select ivg_interfaces
source install/setup.bash
ros2 interface list | grep ivg_interfaces    # 应列出 52 个类型
ros2 interface show ivg_interfaces/msg/RobotStatus  # 应显示字段定义
```

---

## Part 2: ivg_utils — 共享 Python 工具库

### 2.1 架构概览

**职责**: 该项目所有 Python 包的共享工具函数。**零 ROS 依赖**（不 `import rclpy`），纯 Python 数学和常量定义。`ament_python` 类型（非 ament_cmake）。

**为什么零 ROS 依赖**: 这样 `ivg_utils` 可以被非 ROS 环境（训练脚本、MLflow、数据分析脚本）复用。

### 2.2 需要创建的文件

```
ivg_utils/
├── setup.py                   ← ament_python 入口
├── package.xml                ← 声明依赖 (仅 numpy)
├── ivg_utils/
│   ├── __init__.py            ← 空文件
│   ├── math.py                ← 所有数学变换（核心！）
│   ├── io.py                  ← IO 常量和映射
│   └── robot.py               ← 机器人工具常量
```

### 2.3 关键实现: math.py

```python
"""数学变换工具 — 零 ROS 依赖"""
import numpy as np

# ═══════ 四元数约定: Hamilton (xyzw) ═══════
# ROS 2 tf2 使用 Hamilton 四元数 (xyzw)
# 内旋 ZYX = 外旋 XYZ

def euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    """欧拉角 (度, 内旋 ZYX) → 四元数 xyzw"""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cx, sx = np.cos(r/2), np.sin(r/2)
    cy, sy = np.cos(p/2), np.sin(p/2)
    cz, sz = np.cos(y/2), np.sin(y/2)
    # Hamilton 乘法: q = q_yaw * q_pitch * q_roll
    qw = cz*cy*cx + sz*sy*sx
    qx = cz*cy*sx - sz*sy*cx
    qy = cz*sy*cx + sz*cy*sx
    qz = sz*cy*cx - cz*sy*sx
    return np.array([qx, qy, qz, qw])

def quat_to_rotation_matrix(q):
    """四元数 xyzw → 3×3 旋转矩阵"""
    x, y, z, w = q
    return np.array([[1-2*y*y-2*z*z, 2*x*y-2*w*z,   2*x*z+2*w*y],
                     [2*x*y+2*w*z,   1-2*x*x-2*z*z, 2*y*z-2*w*x],
                     [2*x*z-2*w*y,   2*y*z+2*w*x,   1-2*x*x-2*y*y]])

def rotation_matrix_to_quat(R):
    """3×3 旋转矩阵 → 四元数 xyzw"""
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        return np.array([(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s,
                         (R[1,0]-R[0,1])/s, s/4])
    # ... 处理其他三种情况

def pose_to_matrix(position, quat_xyzw):
    """位姿 → 4×4 齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = quat_to_rotation_matrix(quat_xyzw)
    T[:3, 3] = position
    return T

def matrix_to_pose(T):
    """4×4 齐次变换 → (position, quat_xyzw)"""
    return T[:3, 3], rotation_matrix_to_quat(T[:3, :3])

def transform_point(T, point):
    """用齐次变换矩阵变换一个 3D 点"""
    p_h = np.append(point, 1.0)
    return (T @ p_h)[:3]

# ... 更多函数: RPY 互转、SE(3) 乘法、逆变换
```

### 2.4 关键实现: io.py

```python
"""IO 常量定义 — 所有 Worker 统一引用"""

# IO 位置: io_index + 32 = 硬件引脚编号
IO_GRIPPER    = 6   # 夹爪 → 硬件引脚 38
IO_QUICK_SWAP = 7   # 快换盘 → 硬件引脚 39
IO_DO2_LATTE  = 2   # 打花器 → 硬件引脚 34
IO_DO4_COFFEE = 4   # 咖啡机 → 硬件引脚 36

# IO 语义常量 — 消除 true=打开 vs true=闭合 的歧义
GRIPPER_OPEN  = True
GRIPPER_CLOSE = False
QUICK_SWAP_LOCK   = True
QUICK_SWAP_RELEASE = False

# IO 类型字符串
DIGITAL_OUTPUT = "digital_output"
TOOL_IO        = "tool_io"
```

### 2.5 setup.py 标准结构

```python
from setuptools import find_packages, setup
package_name = 'ivg_utils'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
)
```

**编译验证**:
```bash
colcon build --packages-select ivg_utils
source install/setup.bash
python3 -c "from ivg_utils import math; print(math.euler_deg_to_quat(0,0,90))"
```

---

## Part 3: aubo_description — URDF 机器人模型

### 3.1 架构概览

**职责**: 提供 AUBO E5 机器人的 URDF 模型文件（含碰撞 mesh 和视觉 mesh）。`ament_cmake` 包，纯安装（无编译代码）。

**TF 树设计**: `world` → `base_link` (=identity fixed joint) → 6 个旋转关节 → `wrist3_Link` → `camera_Link + kuaihuan_Link + tool_tcp`

### 3.2 需要创建的文件

```
aubo_description/
├── CMakeLists.txt          ← 安装 mesh 和 urdf 到 share/
├── package.xml             ← 无依赖
├── meshes/
│   ├── collision/          ← 22 个 .stl 碰撞网格
│   │   ├── base_link.stl
│   │   ├── shoulder_Link.stl ~ wrist3_Link.stl
│   │   ├── camera_link.stl, kuaihuan_link.stl
│   │   ├── pedestal_link.stl, cup0_link.stl, lizhu_link.stl
│   │   └── gripper0/1/2_link.stl, gripper1coffeecup/milkcup_link.stl
│   └── visual/             ← 22 个 .stl 可视化网格 + 7 个 .dae
│       ├── (对应上述 .stl)
│       └── link0~link6.dae ← AUBO E5 的 DAE 视觉 mesh
└── urdf/
    └── aubo_e5_10.urdf     ← 主 URDF 文件 (~400行)
```

### 3.3 URDF 编写要点

```xml
<?xml version="1.0"?>
<robot name="aubo_e5">

  <!-- 根连杆: world (原点) -->
  <link name="world"/>

  <!-- 基座: 通过 fixed joint 连接到 world -->
  <link name="base_link">
    <inertial>...</inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://aubo_description/meshes/visual/base_link.stl"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://aubo_description/meshes/collision/base_link.stl"/></geometry>
    </collision>
  </link>

  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- 关节 1: shoulder -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_Link"/>
    <origin xyz="0 0 0.1215" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.05" upper="3.05" effort="150" velocity="2.618"/>
  </joint>

  <!-- 关节 2~6 类似定义 -->
  <!-- ... -->
</robot>
```

**CMakeLists.txt**: 纯安装包（无 `add_executable`），用 `install(DIRECTORY ... DESTINATION share/${PROJECT_NAME})` 安装 mesh/urdf。

---

## Part 4: aubo_driver_ros2 — C++ SDK 驱动层

### 4.1 架构概览

**职责**: 调用 AUBO SDK（预编译 .so 库）实现机械臂硬件通信。包含 4 个 C++ 可执行文件。

**4 个可执行文件的关系**:
```
aubo_dashboard_node          ← LifecycleNode，20 个 ROS 2 服务，用户操作入口
    使用 → AuboHardwareInterface (SDK 双连接封装)
aubo_state_broadcaster       ← SDK 回调驱动，发布 ~50Hz 状态话题
    使用 → AuboHardwareInterface
joint_trajectory_controller  ← Action Server，TCP2CAN 轨迹流式发送
    使用 → AuboHardwareInterface
aubo_callback_monitor        ← 监控 SDK 事件（断开/急停/碰撞等）
```

### 4.2 需要创建的文件

```
aubo_driver_ros2/
├── CMakeLists.txt                          ← 4 个可执行文件 + SDK 库链接
├── package.xml                             ← 依赖 rclcpp / rclcpp_lifecycle 等
├── include/aubo_driver_ros2/
│   ├── aubo_dashboard_node.h               ← Dashboard 类声明
│   ├── aubo_hardware_interface.h           ← SDK 封装类声明
│   ├── joint_trajectory_controller.h       ← JTC 类声明
│   ├── AuboRobotMetaType.h                 ← SDK 类型定义
│   ├── serviceinterface.h                  ← SDK ServiceInterface
│   ├── robotiomatetype.h, atomicops.h, readerwriterqueue.h
│   └── otg/                                ← 在线轨迹生成头文件
│       ├── OTGStep1.h, OTGStep2.h, OtgType3Com.h, OTGVelocity.h
│       ├── EquationSolutionStep1.h, otgnewslib.h
├── src/
│   ├── aubo_dashboard_node.cpp             ← 20 个服务实现 (~695行)
│   ├── aubo_hardware_interface.cpp         ← 双连接封装 (~500行)
│   ├── aubo_state_broadcaster.cpp          ← 回调 + 轮询状态发布 (~266行)
│   ├── joint_trajectory_controller.cpp     ← Action Server + 插值 (~244行)
│   ├── aubo_callback_monitor.cpp           ← SDK 事件监控
│   ├── dashboard_node_main.cpp             ← Dashboard 入口
│   └── joint_trajectory_controller_main.cpp ← JTC 入口
├── lib/lib64/                              ← 预编译 SDK .so
│   ├── aubocontroller/                     ← libauborobotcontroller.so.1.3.1
│   ├── log4cplus/, config/, protobuf/      ← 依赖库
│   └── libotgLib.a                         ← OTG 轨迹生成
└── config/
    ├── auborobot.conf, tracelog.properties
```

### 4.3 关键实现模式

#### CMakeLists.txt: 预编译库链接

```cmake
cmake_minimum_required(VERSION 3.5)
project(aubo_driver_ros2)
set(CMAKE_CXX_STANDARD 17)

# SDK .a 未用 -fPIC 编译，必须 -no-pie
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# ... 其他 find_package

# SDK 预编译库路径
set(AUBO_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64)
set(AUBO_SDK_LIBS
  ${AUBO_SDK_DIR}/aubocontroller/libauborobotcontroller.so
  ${AUBO_SDK_DIR}/otgLib.a
  ${AUBO_SDK_DIR}/log4cplus/liblog4cplus.so
  ${AUBO_SDK_DIR}/config/libconfig++.so
  ${AUBO_SDK_DIR}/protobuf/libprotobuf.so)

include_directories(include)

# ============================
# aubo_dashboard_node
# ============================
add_executable(aubo_dashboard_node
  src/aubo_dashboard_node.cpp
  src/aubo_hardware_interface.cpp
  src/dashboard_node_main.cpp)
ament_target_dependencies(aubo_dashboard_node
  rclcpp rclcpp_lifecycle sensor_msgs std_msgs ivg_interfaces)
target_link_libraries(aubo_dashboard_node ${AUBO_SDK_LIBS})

# ============================
# joint_trajectory_controller
# ============================
add_executable(joint_trajectory_controller
  src/joint_trajectory_controller.cpp
  src/aubo_hardware_interface.cpp
  src/joint_trajectory_controller_main.cpp)
ament_target_dependencies(joint_trajectory_controller
  rclcpp rclcpp_action trajectory_msgs control_msgs ivg_interfaces)
target_link_libraries(joint_trajectory_controller ${AUBO_SDK_LIBS})

# ============================
# aubo_state_broadcaster
# ============================
add_executable(aubo_state_broadcaster
  src/aubo_state_broadcaster.cpp
  src/aubo_hardware_interface.cpp
  src/aubo_state_broadcaster_main.cpp)
ament_target_dependencies(aubo_state_broadcaster
  rclcpp sensor_msgs std_msgs control_msgs ivg_interfaces)
target_link_libraries(aubo_state_broadcaster ${AUBO_SDK_LIBS})

install(TARGETS aubo_dashboard_node joint_trajectory_controller
                aubo_state_broadcaster aubo_callback_monitor
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_package()
```

#### AuboHardwareInterface: 双连接模式

```cpp
class AuboHardwareInterface {
public:
    bool connect(const std::string& host, int port) {
        // 连接 1: TCP2CAN 模式 (轨迹流式发送 + RIB 诊断)
        conn_control_ = RS::createClient();
        conn_control_->connect(host, port);
        conn_control_->setConnectMode(RS::TCP2CAN);
        // 连接 2: 普通模式 (状态查询 + IO 读写)
        conn_status_ = RS::createClient();
        conn_status_->connect(host, port);

        // 设置 TCP2CAN 相关的回调 (RoadPoint, JointStatus)
        conn_control_->setRoadPointCallback([this](auto& data) {
            // 回调运行在 SDK 内部线程上
            // 只能做 atomic 写入 + 日志，不能调用 SDK API！
            std::lock_guard<std::mutex> lock(callback_mutex_);
            memcpy(&latest_rp_, &data, sizeof(data));
        });
        return true;
    }

    // ⚠️ 所有 SDK 调用必须串行化（SDK 同步阻塞, 2-225ms）
    std::mutex sdk_mutex_;

    bool callSDK(std::function<bool(RS::Client*)> fn) {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        return fn(conn_status_.get());
    }

private:
    std::shared_ptr<RS::Client> conn_control_;  // TCP2CAN + RIB
    std::shared_ptr<RS::Client> conn_status_;   // 状态 + IO
};
```

#### aubo_dashboard_node: LifecycleNode + 20 个服务

```cpp
class AuboDashboardNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
        // 建立双 TCP 连接
        hw_ = std::make_shared<AuboHardwareInterface>();
        hw_->connect(server_host_, 8899);

        // 创建 20 个 ROS 2 服务
        get_ik_srv_ = create_service<ivg_interfaces::srv::GetIK>(
            "/aubo_driver/get_ik",
            [this](auto req, auto res) { handleGetIK(req, res); });

        get_fk_srv_ = create_service<ivg_interfaces::srv::GetFK>(
            "/aubo_driver/get_fk",
            [this](auto req, auto res) { handleGetFK(req, res); });

        set_io_srv_ = create_service<ivg_interfaces::srv::SetRobotIO>(
            "/aubo_driver/set_io",
            [this](auto req, auto res) { handleSetIO(req, res); });
        // ... 共 20 个服务
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
        // activate: 轻量级，持久化资源已创建
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
        // deactivate: 关闭 service 响应
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
        // shutdown: 断开 TCP 连接，清理 SDK 资源
        return CallbackReturn::SUCCESS;
    }

private:
    void handleGetIK(const std::shared_ptr<ivg_interfaces::srv::GetIK::Request> req,
                     std::shared_ptr<ivg_interfaces::srv::GetIK::Response> res) {
        std::lock_guard<std::mutex> lock(hw_->sdk_mutex_);
        // 调用 SDK 的 inverseKinematics() ...
    }
    // ... 20 个 handler
};
```

#### joint_trajectory_controller: Action Server + TCP2CAN 流控

```cpp
class JointTrajectoryController : public rclcpp::Node {
public:
    JointTrajectoryController() : Node("joint_trajectory_controller") {
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this, "/follow_joint_trajectory",
            [](auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
            [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
            [this](auto goal_handle) { executeTrajectory(goal_handle); });
    }

private:
    void executeTrajectory(std::shared_ptr<GoalHandle> goal_handle) {
        auto& traj = goal_handle->get_goal()->trajectory;
        // 预计算: 五次多项式插值所有轨迹点
        auto interpolated = interpolateTrajectory(traj);
        // 启动独立发送线程 (TCP2CAN 流式发送, 固定 5ms 间隔)
        send_thread_ = std::thread([this, &interpolated]() {
            for (auto& p : interpolated) {
                auto t0 = std::chrono::steady_clock::now();
                hw_->sendTrajectoryPoint(p);
                auto elapsed = std::chrono::steady_clock::now() - t0;
                if (elapsed < std::chrono::milliseconds(5))
                    std::this_thread::sleep_for(std::chrono::milliseconds(5) - elapsed);
                // 超时 → 放弃当前点
            }
        });
        // 反馈 + 目标检测 (5 帧 in 0.02 rad 容差)
        while (rclcpp::ok() && !goal_reached) {
            auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
            // 读取 RIB 中实际关节角
            goal_handle->publish_feedback(feedback);
        }
    }

    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    std::thread send_thread_;
};
```

---

## Part 5: aubo_moveit_config — MoveIt2 配置包

### 5.1 架构概览

**职责**: MoveIt2 的启动入口。包含 SRDF、运动学插件、OMPL 规划器、ros2_control 仿真配置。`ament_cmake` 纯安装包。

**核心设计**: `aubo_new_driver.launch.py` 通过 OpaqueFunction 在 launch 解析阶段 TCP 探测机械臂是否可达，决定真机/仿真模式。

### 5.2 需要创建的文件

```
aubo_moveit_config/
├── CMakeLists.txt                    ← 纯安装 (无编译)
├── package.xml
├── config/
│   ├── aubo_e5.urdf.xacro             ← XACRO 入口 (动态夹爪切换)
│   ├── aubo_e5.srdf                   ← 语义描述 (规划组/碰撞矩阵/命名位姿)
│   ├── aubo_e5.ros2_control.xacro     ← ros2_control 仿真硬件接口
│   ├── joint_limits.yaml              ← 速度/加速度限制
│   ├── joint_names.yaml               ← 控制器关节名
│   ├── kinematics.yaml                ← KDL 运动学插件
│   ├── moveit_controllers.yaml        ← 控制器映射
│   ├── ompl_planning.yaml             ← OMPL 规划器配置
│   ├── moveit.rviz                    ← RViz2 默认显示
│   └── ros2_controllers.yaml
├── launch/
│   ├── aubo_new_driver.launch.py      ← ★ 主启动文件 (TCP 探测)
│   ├── demo_driver_services.launch.py ← Demo Driver 服务启动
│   ├── move_group.launch.py           ← move_group 单独启动
│   └── rsp.launch.py                  ← RobotStatePublisher
└── scripts/
    └── aubo_mode.py                   ← 模式发布
```

### 5.3 关键实现: aubo_new_driver.launch.py 的 TCP 探测

```python
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, LifecycleNode
import socket

def _check_robot_reachable(context, *args, **kwargs):
    """OpaqueFunction: 在 launch 解析阶段执行 TCP 探测"""
    server_host = LaunchConfiguration("server_host").perform(context)
    mode = "simulation"
    try:
        sock = socket.create_connection((server_host, 8899), timeout=2.0)
        sock.close()
        mode = "real"
    except (socket.timeout, ConnectionRefusedError, OSError):
        pass  # 不可达 → simulation

    actions = []
    if mode == "real":
        # 真机: AUBO SDK 驱动节点
        actions.append(LifecycleNode(
            package="aubo_driver_ros2", executable="aubo_dashboard_node",
            name="aubo_dashboard"))
        actions.append(Node(
            package="aubo_driver_ros2", executable="aubo_state_broadcaster",
            name="aubo_state_broadcaster"))
        actions.append(Node(
            package="aubo_driver_ros2", executable="joint_trajectory_controller",
            name="joint_trajectory_controller"))
    else:
        # 仿真: ros2_control + mock_components
        actions.append(Node(
            package="controller_manager", executable="ros2_control_node",
            parameters=[robot_description, ros2_control_config]))
        actions.append(Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "joint_trajectory_controller"]))

    # 两种模式共享的节点
    actions.append(Node(package="aubo_moveit_config", executable="aubo_mode"))
    actions.append(IncludeLaunchDescription(move_group_launch))
    actions.append(IncludeLaunchDescription(rsp_launch))
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("server_host", default_value="169.254.10.98"),
        OpaqueFunction(function=_check_robot_reachable),
    ])
```

### 5.4 SRDF 关键条目

```xml
<!-- planning group 定义 -->
<group name="manipulator">
  <chain base_link="base_link" tip_link="tool_tcp"/>
</group>

<!-- 命名状态: 预设关节位形 -->
<group_state name="home" group="manipulator">
  <joint name="shoulder_joint" value="0.0"/>
  <joint name="upperArm_joint" value="-0.2"/>
  <joint name="foreArm_joint" value="1.5"/>
  <joint name="wrist1_joint" value="0.0"/>
  <joint name="wrist2_joint" value="1.57"/>
  <joint name="wrist3_joint" value="0.0"/>
</group_state>

<!-- ACM: 禁用自碰撞检查的连杆对（机械臂本体内部豁免） -->
<disable_collisions link1="shoulder_Link" link2="upperArm_Link" reason="Adjacent"/>
<!-- 工具 ACM 条目已注释 — 运行时由 scene_attach_worker 通过 ACO touch_links 动态处理 -->
```

---

## Part 6: demo_driver — C++ 应用服务层 (11 个节点)

### 6.1 架构概览

**职责**: 在 MoveIt 2 之上提供高层运动控制和抓取服务。11 个可执行文件分为两类：基础运动服务（8 个）和抓取 Worker（2 个 + 1 个监控节点）。

**核心设计**: `RobotController` 类封装所有 MoveIt 调用 + IO 控制，11 个节点全部通过组合模式使用它。

### 6.2 需要创建的文件

```
demo_driver/
├── CMakeLists.txt                ← 11 个可执行文件
├── package.xml                   ← 依赖 moveit_core / moveit_ros_planning_interface
├── include/demo_driver/
│   ├── robot_controller.h        ← ★ 核心共享类
│   ├── execute_grasp_pose_worker.h     ← 9 步抓取周期
│   ├── publish_grasps_client_worker.h  ← 11 步 GraspNet 抓取-放置
│   ├── execute_trajectory_server.h
│   ├── get_current_state_server.h
│   ├── move_to_pose_server.h
│   ├── plan_trajectory_server.h
│   ├── read_robot_io_server.h
│   ├── set_robot_enable_server.h
│   ├── set_robot_pose_server.h
│   ├── set_speed_factor_server.h
│   └── system_logger.h
├── src/
│   ├── robot_controller.cpp           ← MoveIt + IO 封装
│   ├── execute_grasp_pose_worker.cpp  ← ~1050行
│   ├── publish_grasps_client_worker.cpp ← ~1290行
│   ├── execute_trajectory_server.cpp
│   ├── get_current_state_server.cpp
│   ├── move_to_pose_server.cpp
│   ├── plan_trajectory_server.cpp
│   ├── read_robot_io_server.cpp
│   ├── set_robot_enable_server.cpp
│   ├── set_robot_pose_server.cpp
│   ├── set_speed_factor_server.cpp
│   └── system_monitor_node.cpp
└── launch/
    ├── execute_grasp_pose_worker.launch.py
```

### 6.3 关键实现: RobotController (组合模式)

```cpp
class RobotController {
public:
    explicit RobotController(rclcpp::Node* parent) : parent_(parent) {}

    // 工厂方法模式: 两阶段初始化（MoveGroupInterface 需要 shared_from_this()）
    void init() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            parent_->shared_from_this(), "manipulator");
        move_group_->setMaxVelocityScalingFactor(0.7);
        move_group_->setMaxAccelerationScalingFactor(0.3);
    }

    // ═══════ 关节空间运动 ═══════
    bool moveToJoints(const std::array<double, 6>& joints) {
        move_group_->setJointValueTarget(
            std::vector<double>(joints.begin(), joints.end()));
        auto result = move_group_->move();  // 阻塞调用
        return result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }

    bool moveToHome() { return moveToJoints(home_joints_); }

    // ═══════ 笛卡尔空间运动 ═══════
    bool moveCartesianPath(const std::vector<CartesianSegment>& segments) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        auto current = move_group_->getCurrentPose().pose;
        waypoints.push_back(current);

        for (auto& seg : segments) {
            geometry_msgs::msg::Pose wp = waypoints.back();
            switch (seg.axis) {
                case 'x': wp.position.x += seg.offset; break;
                case 'y': wp.position.y += seg.offset; break;
                case 'z': wp.position.z += seg.offset; break;
            }
            waypoints.push_back(wp);
        }

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(
            waypoints, 0.01, 0.0, trajectory);  // eef_step=0.01, jump_threshold=0.0

        if (fraction >= 0.95) {
            return move_group_->execute(trajectory) == MoveItErrorCode::SUCCESS;
        }
        return false;  // 路径不完整，不执行！
    }

private:
    rclcpp::Node* parent_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

// CartesianSegment — 笛卡尔运动原语
// axis 'x'/'y'/'z' = 沿 axis 走 offset 米
struct CartesianSegment { char axis; double offset; };
```

### 6.4 关键实现: ExecuteGraspPoseWorker (9 步抓取周期)

```cpp
class ExecuteGraspPoseWorker : public rclcpp::Node {
    // Reentrant callback group 避免死锁（服务回调 + client async_send_request）
    // 9 步抓取流程:
    //
    // 1. moveToHome("camera_pose")           ← 关节运动到观察位
    // 2. buildGraspPose()                   ← 从 VPE 结果构建目标
    // 3. transform gripper_tip → end_effector ← TF lookup
    // 4. openGripper()                      ← set_io(IO_GRIPPER=6, true)
    // 5. graspApproach()                    ← 6 航点笛卡尔路径 (接近→下降)
    // 6. closeGripper()                     ← set_io(IO_GRIPPER=6, false)
    // 7. liftUp(z + 0.2)                     ← 垂直抬升
    // 8. moveToPlace()                      ← home + offset
    // 9. openGripper() → returnToHome()      ← 释放工件，回起始位

    // 关键参数:
    // joint_velocity_scaling=0.7 / joint_acceleration_scaling=0.3
    // cartesian_max_points=40 (每段笛卡尔路径最多 40 个航点)
    // Z 安全下限 0.19m (防止撞桌)
};
```

### 6.5 关键实现: PublishGraspsClientWorker (11 步 GraspNet 抓取-放置)

```cpp
class PublishGraspsClientWorker : public rclcpp::Node {
    // 11 步抓取-放置流程:
    //
    // 窗口收集: 订阅 /grasp_poses_base → grasp_window_ (环形缓存 5 组)
    // 入场条件: 至少 3 组入库 (min_groups_before_pick=3)
    // 选择策略: prefer_vertical=true → 选 approach 最接近 -Z 的抓取
    // Z 翻转:   applyGraspZFlip180() — 修正 GraspNet 预测的 Z 轴方向
    // 补偿:     grasp_z_offset=-0.15 → gripper_tip → end_effector

    void selectBestGrasp() {
        for (auto& p : grasp_window_) {
            p = applyGraspZFlip180(p);  // Z 轴 180° 翻转
        }
        // 计算 approach 方向 (Z 轴) 的 dot product with world -Z
        // 选择最接近 -Z 的抓取
    }
};
```

---

## Part 7: tool_changer — C++ 工具快换 (2 个节点)

### 7.1 架构概览

**职责**: 物理快换运动控制 + MoveIt PlanningScene 碰撞体附着。2 个节点协作：

```
gripper_swap_worker  ← 物理运动 + IO 控制
    │ 发布 /tool_changer_status
    ▼
scene_attach_worker  ← 订阅 /tool_changer_status → 自动同步 ACO
    │ 发布 /attached_collision_object (ACO ADD/REMOVE)
    │ 发布 /planning_scene (world REMOVE 清残留)
```

**数据驱动**: `config/tools.yaml` 是唯一配置来源，新增工具只需编辑 YAML，无需修改 C++。

### 7.2 需要创建的文件

```
tool_changer/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── tools.yaml                    ← ★ 5 种工具的配置
├── include/tool_changer/
│   ├── gripper_swap_worker.h         ← 物理快换类
│   └── scene_attach_worker.h         ← 场景附着类
├── src/
│   ├── gripper_swap_worker.cpp       ← 7 步快换流程 (~750行)
│   └── scene_attach_worker.cpp       ← ACO 管理 (~380行)
├── launch/
│   └── gripper_swap_worker.launch.py ← 启动 2 个节点
└── urdf/                             ← 5 个工具的 URDF 片段
    ├── aubo_e5_gripper0.urdf
    ├── aubo_e5_gripper1.urdf
    ├── aubo_e5_gripper2.urdf
    ├── aubo_e5_gripper1coffeecup.urdf
    └── aubo_e5_gripper1milkcup.urdf
```

### 7.3 关键实现: tools.yaml 配置模式

```yaml
tools:
  gripper0:
    name: "气动夹爪 φ40"
    type: "gripper"
    mesh_visual: "package://aubo_description/meshes/visual/gripper0_link.stl"
    mesh_collision: "package://aubo_description/meshes/collision/gripper0_link.stl"
    dock_pose: { position: {x: 2.0, y: 0.29517, z: 0.01}, orientation: {x:0,y:0,z:0,w:1} }
    attach_offset: { position: {x:0,y:0,z:0.033}, orientation: {x:0,y:0,z:0,w:1} }
    touch_links: ["kuaihuan_Link", "camera_Link", "wrist3_Link", "tool_tcp"]
    dock_approach_joints: [1.137, 0.222, 1.598, -0.194, 1.571, 1.136]
    trajectory:
      strategy: "vertical"   # 纯 Z 轴升降
      depth: 0.210           # 下降深度 (m)
      lift: 0.210            # 抬升高度 (m)
      settle_sec: 0.5        # 稳定时间 (s)

  gripper2:
    # ...
    trajectory:
      strategy: "slide"      # Y 轴侧滑
      depth: 0.210
      lift: 0.210
      slide_y: 0.100         # 侧滑距离 (m)
      seat: 0.012            # 坐入深度 (m)
      settle_sec: 0.5
      release_sec: 0.3       # 释放锁定时长
      lock_sec: 0.5          # 锁定时长
```

### 7.4 关键实现: changeToTool (7 步快换流水线)

```cpp
bool GripperSwapWorker::changeToTool(const std::string& target_id) {
    const auto& target = tool_configs_[target_id];

    // ══ 1. 释放当前工具 ══
    if (!current_tool_.id.empty()) {
        moveToDockApproach(current);        // 关节运动到 dock 接近位
        sleepJointCartesianSwitchDelay();   // J→C 模式切换延时
        updateSceneAttachment(current.id, false);  // ★ 先卸碰撞体
        releaseTool(current);              // 笛卡尔松轨 + IO 释放
        publishToolStatus(false);          // ★ 物理松后立即清空状态
    }

    // ══ 2. 取目标工具 ══
    moveToDockApproach(target);           // 关节运动到新 dock
    sleepJointCartesianSwitchDelay();
    pickTool(target);                     // 笛卡尔取轨 + IO 锁紧
    publishToolStatus(true);              // ★ pick 成功后立即更新（不等回 home）

    // ══ 3. 回 Home ══
    sleepJointCartesianSwitchDelay();
    moveToHome();
    return true;
}
```

### 7.5 关键实现: scene_attach_worker 的 ACO 同步

```cpp
void SceneAttachWorker::onToolStatus(const ToolChangerStatus& msg) {
    if (msg.is_connected && msg.tool_id != current_attached_tool_) {
        // 新工具附着
        attachToolToScene(msg.tool_id);
    } else if (!msg.is_connected && !current_attached_tool_.empty()) {
        // 旧工具脱离
        detachToolFromScene(current_attached_tool_);
    }
}

void SceneAttachWorker::attachToolToScene(const std::string& tool_id) {
    // 先清除 world 中可能残留的同名对象
    removeWorldToolObject(tool_id);

    // AttachedCollisionObject ADD
    moveit_msgs::msg::AttachedCollisionObject att;
    att.object.id = "attached_tool_" + tool_id;
    att.object.operation = CollisionObject::ADD;
    att.link_name = "kuaihuan_Link";
    att.object.pose = tool_geometries_[tool_id].attach_offset;  // ← 唯一数据源
    att.touch_links = tool_geometries_[tool_id].touch_links;
    // mesh_poses[0] 为单位姿态（offset 已体现在 object.pose 中）
    att.object.mesh_poses = {identityPose()};

    attached_object_pub_->publish(att);  // 通过 /attached_collision_object 发布
}

void SceneAttachWorker::removeWorldToolObject(const std::string& tool_id) {
    // detach 后 ACO 对象可能"掉落"到 world 中成为普通碰撞对象
    // 通过 /planning_scene 发送 world REMOVE diff 清理
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    CollisionObject obj;
    obj.id = "attached_tool_" + tool_id;
    obj.operation = CollisionObject::REMOVE;
    scene.world.collision_objects.push_back(obj);
    planning_scene_pub_->publish(scene);
}
```

### 7.6 并发模型

```cpp
// gripper_swap_worker: MutuallyExclusive + MultiThreadedExecutor(2)
//   → 服务回调不并发（安全），但有独立的 spin 线程 (=不阻塞主线程)
// scene_attach_worker: 默认单线程 spin
//   → 简单场景，不需要并发

void GripperSwapWorker::run() {
    // 等待 /aubo/mode 话题（确认仿真/真机模式）
    // 创建 MultiThreadedExecutor(2) 启动 spin
    rclcpp::executors::MultiThreadedExecutor exec(options, 2);
    exec.add_node(shared_from_this());
    std::thread spinner([&exec]() { exec.spin(); });
    // 主线程休眠，直到 shutdown
    while (rclcpp::ok() && !shutdown_requested_)
        sleep(0.5);
}
```

---

## Part 8-13: Python 包和 Web 层（汇总）

由于篇幅限制，以下包创建模式与上文类似，本节省略完整文件列表，仅给出**核心实现要点**：

### Part 8: graspnet_ros2 — Python GraspNet 包

```
核心流程:
  相机点云 → PointNet++ 编码 → VCoT-Grasp 解码
  → NMS (max_grasps_num=5) → 碰撞检测 → TF lookup 转到 base_link
  → 发布 /grasp_poses_base (PoseArray)
  → Z 轴 180° 翻转 (applyGraspZFlip180)

关键文件:
  graspnet_ros2/
  ├── setup.py (ament_python)
  ├── launch/graspnet_demo_points_with_tf.launch.py
  ├── graspnet_ros2/
  │   └── graspnet_demo_points_node.py ← 主节点
  └── graspnet-baseline/               ← 上游代码 + CUDA 扩展
      ├── models/graspnet.py             ← GraspNet 模型
      ├── pointnet2/                     ← CUDA 扩展
      └── graspnetAPI/                   ← 抓取后处理
```

### Part 9: visual_pose_estimation_python — Python VPE 包

```
核心流程:
  彩色图 → rembg 去背景 → Canny 边缘 + 轮廓 → 模板匹配
  深度图 → 3D 坐标计算
  → 发布 /estimate_pose 服务

关键文件:
  visual_pose_estimation_python/
  ├── setup.py
  ├── launch/visual_pose_estimation_python.launch.py
  ├── launch/visual_pose_estimation_web.launch.py
  └── visual_pose_estimation_python/
      ├── ros2_communication.py      ← ROS 2 节点核心
      ├── pose_estimator.py          ← 姿态估计算法
      └── preprocess_and_feature_logic.py ← 前后处理
```

### Part 10: latte_imitation — Python 轨迹重定目标

```
核心算法 (Se3RelRetargeter 语义):
  R_rel = R(rpy_user) @ R(q_cup)         ← 相对旋转
  p_new = R_rel @ (p - p0) + p_cup       ← 位置变换
  q_new = q_rel * q_orig (Hamilton)      ← 四元数组合

关键文件:
  latte_imitation/
  ├── DESIGN.md                          ← 完整理论依据
  ├── setup.py
  ├── launch/start_latte_pour.launch.py
  └── latte_imitation/
      ├── core_node.py                   ← 服务: /latte_imitation/replay_trajectory
      ├── retarget.py                    ← SE(3) 重定目标算法
      ├── workspace_safety.py            ← 工作空间安全检查
      └── markers.py                     ← RViz2 Marker 可视化
```

### Part 11: coffee_latte_demo — Python IO 控制

```
核心: 通过 /aubo_driver/set_io 控制 DO2 (打花器) 和 DO4 (咖啡机)

关键文件:
  coffee_latte_demo/
  ├── setup.py
  ├── launch/coffee_latte_demo.launch.py
  └── coffee_latte_demo/
      └── latte_node.py                  ← 服务: /set_latte_do2, /set_latte_do4
```

### Part 12: percipio_camera — 相机驱动

```
核心: 驱动 Percipio FM830 深度相机，发布彩色/深度/点云话题

关键文件:
  percipio_camera/
  ├── setup.py
  ├── launch/percipio_camera.launch.py
  └── percipio_camera/
      └── percipio_camera_node_driver.py ← 相机驱动节点
```

### Part 13: aubo_ros2_web_dashboard — Web 全栈

```
架构:
  浏览器 (Vue 3 SPA) ←→ WebSocket /ws/rosbridge ←→ FastAPI 网关 (:8090) ←→ rosbridge (:9090)
                                                         │
                                                    web_video_server (:8089)

关键文件:
  aubo_ros2_web_dashboard/
  ├── setup.py                            ← ament_python + web/dist 安装
  ├── launch/web_dashboard.launch.py      ← 启动 4 个进程
  ├── aubo_ros2_web_dashboard/
  │   ├── config.py                       ← YAML 配置读取
  │   └── gateway/
  │       ├── app.py                      ← FastAPI 应用工厂 (路由注册)
  │       └── routes/
  │           ├── upstream_proxy.py       ← WebSocket + HTTP 流代理
  │           ├── ivg_runtime.py          ← /api/v1/runtime (BFF)
  │           ├── health.py               ← /health
  │           └── robot_mesh.py           ← /api/ivg/robot-mesh/*
  ├── config/defaults.yaml                ← 所有默认配置 (唯一来源)
  └── web/                                ← Vue 3 前端
      ├── package.json                    ← npm 依赖
      ├── vite.config.ts                  ← Vite 配置 (proxy)
      ├── index.html                      ← SPA 入口
      └── src/
          ├── main.ts                     ← createApp + Router
          ├── App.vue                     ← 根布局
          ├── router/index.ts             ← 6 条路由
          ├── composables/ros/useRos.ts   ← ★ ROS 连接管理
          ├── composables/ros/useRosTopic.ts ← 话题订阅
          ├── composables/ros/useRosService.ts ← 服务调用
          ├── composables/api/useRuntime.ts ← 运行时配置加载
          ├── components/ivg/Robot3dViewer.vue ← ★ 3D 可视化
          ├── components/ivg/RobotStatusBar.vue ← 状态栏
          ├── lib/three_urdf/UrdfModel.ts ← URDF 加载 + Three.js 渲染
          ├── lib/three_urdf/SceneManager.ts ← Three.js 场景管理
          ├── lib/tf_math.ts              ← 四元数/变换工具
          └── views/                      ← 6 个页面视图
```

### 13.x 关键实现: useRos.ts 单例模式

```typescript
// 模块级单例 — 所有组件共享同一条 ROS 连接
let rosInstance: Ros | null = null
const connected = ref(false)        // 独立 ref (Ros 不能放入响应式!)

export function useRos() {
    async function connect(): Promise<void> {
        if (connectPromise) return connectPromise  // 去重
        const url = rosbridgeWsUrl()
        rosInstance = new Ros({ url })
        return new Promise<void>((resolve, reject) => {
            rosInstance!.on('connection', () => { connected.value = true; resolve() })
            rosInstance!.on('error', (err) => { reject(err) })
        })
    }

    function subscribe(topic: string, msgType: string) {
        const sub = new Topic({
            ros: rosInstance!, name: topic,
            messageType: msgType, throttle_rate: 0
        })
        sub.subscribe((msg) => { /* 分发到注册的 handler */ })
    }

    return { connected: readonly(connected), connect, subscribe, callService }
}
```

### 13.x 关键实现: FastAPI WebSocket 代理

```python
@ws_router.websocket("/ws/rosbridge")
async def rosbridge_websocket_proxy(websocket: WebSocket) -> None:
    await websocket.accept()
    target = f"ws://{rosbridge_host}:{rosbridge_port}/"

    async with websockets.connect(target) as upstream:
        # 并发双向转发: client → upstream, upstream → client
        async def client_to_upstream():
            while True:
                msg = await websocket.receive()
                if msg.get("text"):
                    await upstream.send(msg["text"])

        async def upstream_to_client():
            async for message in upstream:
                await websocket.send_text(message)

        await asyncio.gather(client_to_upstream(), upstream_to_client())
```

### 13.x 关键实现: Three.js URDF + Phong 管线

```typescript
// UrdfModel.ts — 加载 URDF + 渲染为 Three.js MeshPhongMaterial
// 保持与 RViz2 OGRE Phong 一致的着色模型

async function loadUrdf(url: string): Promise<THREE.Group> {
    const xml = await fetch(url).then(r => r.text())
    const robot = await parser.parse(xml)

    for (const link of robot.links) {
        const group = new THREE.Group()
        for (const visual of link.visuals) {
            const geometry = await loadMesh(visual.geometry.filename)
            const material = new THREE.MeshPhongMaterial({
                color: visual.color,              // URDF 颜色 → diffuse
                specular: 0x111111,               // DAE 默认 specular
                shininess: 30                     // DAE 默认 shininess
            })
            // ⚠️ 不替换为 MeshStandardMaterial!
            // ColladaLoader 从 DAE <phong> 创建材质
            // 保持 Phong → 前端与 RViz2 着色模型一致
            group.add(new THREE.Mesh(geometry, material))
        }
        robotGroup.add(group)
    }
    return robotGroup
}
```

---

## 编译与构建策略

### 首次编译顺序

```bash
cd ~/ivg_ws
source /opt/ros/humble/setup.bash

# 1. 先编译接口包（其他所有包的依赖）
colcon build --packages-select ivg_interfaces
source install/setup.bash

# 2. 编译 Python 工具包（零 ROS 依赖，无循环依赖）
colcon build --packages-select ivg_utils
source install/setup.bash

# 3. 编译 C++ 驱动层（依赖 SDK .so 的路径配置）
colcon build --packages-select aubo_driver_ros2
source install/setup.bash

# 4. 编译应用层（依赖 MoveIt2 + ivg_interfaces）
colcon build --packages-select demo_driver tool_changer
source install/setup.bash

# 5. 编译 Python 感知层
colcon build --packages-select visual_pose_estimation_python graspnet_ros2 latte_imitation coffee_latte_demo
source install/setup.bash

# 6. 编译 Web Dashboard（需要先 npm run build 前端）
colcon build --packages-select aubo_ros2_web_dashboard
source install/setup.bash
```

### 增量编译（日常开发）

```bash
# 修改 C++ 代码后:
colcon build --packages-select <修改的包> && source install/setup.bash

# 修改 Python 代码后:
colcon build --packages-select <修改的包> && source install/setup.bash

# 修改 Vue 前端后:
cd web/ && npm run build  # 无需 colcon build
```

---

## 关键设计模式总结

| 模式 | 使用场景 | 示例 |
|------|---------|------|
| **工厂方法 + 两阶段初始化** | 构造时不能用 `shared_from_this()` | `GripperSwapWorker::create()` |
| **组合模式** (非继承) | 多个节点共享同一 MoveIt 封装 | `RobotController` 被 11 个节点复用 |
| **数据驱动架构** | 配置与代码分离 | `tools.yaml` 是唯一配置来源 |
| **模块级单例** (Vue 3) | 所有组件共享 ROS 连接 | `useRos()` 中的 `let rosInstance` |
| **Composable 模式** (Vue 3) | 可复用的有状态逻辑 | `useRosTopic` / `useRuntime` / `useMJPEGStream` |
| **callback group 隔离** | 服务回调 + client 调用的并发安全 | `MutuallyExclusive` + `MultiThreadedExecutor` |
| **QoS transient_local** | latched 数据 (需要新订阅者立马收到) | `/attached_collision_object` / `/planning_scene` |
| **OpaqueFunction 模式** | launch 解析阶段做决策 | TCP 探测决定真机/仿真 |
| **BFF 纯代理** (零 ROS 依赖) | FastAPI 不做业务逻辑 | 仅静态文件 + WS 代理 + HTTP 代理 |

---

## 复刻检查清单

```markdown
□ Part 1: ivg_interfaces
  □ 52 个 .msg/.srv 文件已创建
  □ colcon build --packages-select ivg_interfaces 通过
  □ ros2 interface list | grep ivg_ 列出 52 个类型

□ Part 2: ivg_utils
  □ math.py 中四元数/旋转矩阵/欧拉角互转函数已实现
  □ python3 -c "from ivg_utils import math" 无报错

□ Part 3: aubo_description
  □ URDF 中 6 个 revolute 关节的 joint limits 正确
  □ check_urdf aubo_e5_10.urdf 通过

□ Part 4: aubo_driver_ros2
  □ 4 个可执行文件编译成功
  □ ldd 检查无缺失 .so
  □ Dashboard LifecycleNode: configure → activate 状态转换正常

□ Part 5: aubo_moveit_config
  □ TCP 探测 OpaqueFunction 能区分真机/仿真
  □ ros2 launch aubo_moveit_config aubo_new_driver.launch.py 仿真模式正常启动

□ Part 6: demo_driver
  □ 11 个可执行文件编译成功
  □ /execute_single_grasp 服务可用 (仿真模式)
  □ 9 步抓取周期 log 输出顺序正确

□ Part 7: tool_changer
  □ /change_tool 服务可用
  □ tools.yaml 加载正确 (5 种工具)
  □ ACO 的 /attached_collision_object 发布后 move_group 能看到碰撞体

□ Part 8-12: Python 包
  □ graspnet_ros2: /grasp_poses_base 话题有数据
  □ visual_pose_estimation: /estimate_pose 服务可用
  □ latte_imitation: /replay_trajectory 服务可用 (preview mode)
  □ coffee_latte_demo: /set_latte_do2 /set_latte_do4 服务可用

□ Part 13: aubo_ros2_web_dashboard
  □ Vue 3 前端 npm run build 成功
  □ web/dist/index.html 存在
  □ 浏览器打开 :8090 能看到 6 个页面
  □ rosbridge WebSocket 连接正常
  □ 3D 模型正常渲染
```

---

*最后更新: 2026-05-20*
*依据: 全部 src/ 下源码、CMakeLists.txt、package.xml、launch 文件、config 文件*
