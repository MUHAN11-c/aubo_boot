# AUBO SDK → ROS2 接口完整映射

> 对应源码: `include/aubo_driver_ros2/serviceinterface.h` (1343行, `class ServiceInterface`)

---

## 映射原则

| SDK 函数类型 | ROS2 映射方式 | 原因 |
|-------------|-------------|------|
| 实时轨迹流 (TCP2CAN) | `HardwareInterface::writeTrajectoryPoints()` | 200Hz 高频, 阻塞调用需独立线程 |
| 状态查询 (高频 ≥50Hz) | `HardwareInterface` + `StateBroadcaster` 定时发布 | 避免每个回调都调 SDK |
| 状态查询 (低频/按需) | `/aubo/get_*` Dashboard 服务 | 按需调用, 不占带宽 |
| 系统管理 (上电/关机) | `/aubo/startup` 等 Dashboard 服务 | 阻塞数秒, 独立 Executor 处理 |
| 运动控制 (SDK API) | `/aubo/move_*` Dashboard 服务 | 需先退出 TCP2CAN, 自动切换 |
| 配置修改 | `/aubo/set_*` Dashboard 服务 + 运行时参数 | 运动停止时生效 |
| IO 读写 | `HardwareInterface` (读) + Dashboard (写) | 读高频轮询, 写按需 |
| FK/IK/坐标变换 | `/aubo/get_fk` 等 Dashboard 服务 | 纯计算, 始终可用 |
| SDK 事件回调 | `HardwareInterface::registerCallbacks()` | SDK 内部线程, 回调中只做原子写入 |

---

## 1. 系统/连接

### 1.1 robotServiceLogin
```cpp
int robotServiceLogin(const char* host, int port,
                      const char* userName, const char* password);
int robotServiceLogin(const char* host, int port,
                      const char* userName, const char* password,
                      aubo_robot_namespace::RobotType &robotType,
                      aubo_robot_namespace::RobotDhPara &robotDhPara);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ 每个 HardwareInterface 实例调用 2 次 (conn_control_ + conn_status_) |
| **ROS2 接口** | `HardwareInterface::init()` 内部调用 |
| **阻塞时间** | 1-3s (TCP 握手 + 认证) |
| **重试策略** | 默认5次, 通过 `max_retries` 参数控制 |
| **冲突规则** | 必须在所有其他 API 调用之前成功 |

**ROS2 调用方式**:
```cpp
auto hw = std::make_shared<AuboHardwareInterface>();
hw->init("169.254.10.98", 8899);  // 内部 login ×2
// 成功后 hw->isConnected() == true
```

### 1.2 robotServiceLogout
```cpp
int robotServiceLogout();
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ 析构/关闭时调用 |
| **ROS2 接口** | `HardwareInterface::shutdown()` 内部调用 |
| **冲突规则** | 调用前应先 `leaveTcp2CanbusMode()` |

### 1.3 robotServiceGetConnectStatus
```cpp
void robotServiceGetConnectStatus(bool &connectStatus);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ➖ 驱动自行维护 `connected_` 原子变量 |
| **ROS2 接口** | `HardwareInterface::isConnected()` |

### 1.4 robotServiceRobotHandShake
```cpp
int robotServiceRobotHandShake(bool isBlock);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ➖ login 后可选握手 |
| **ROS2 接口** | `HardwareInterface::init()` 末尾 (可选) |

---

## 2. 运动控制

> **关键约束**: 以下所有 SDK 运动 API 与 TCP2CAN 模式互斥。
> Dashboard 服务调用前会自动执行 `ensureTcp2CanOff()`。

### 2.1 robotServiceJointMove (关节运动)
```cpp
int robotServiceJointMove(aubo_robot_namespace::wayPoint_S &wayPoint, bool IsBolck);
int robotServiceJointMove(double jointAngle[ARM_DOF], bool IsBolck);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ `AuboAPICallback` 和 `/aubo/move_joint` 服务 |
| **ROS2 接口** | `/aubo/move_joint` (std_srvs::Trigger) |
| **输入** | 参数 `move_joint_target`: 逗号分隔6关节角(rad), 如 `"0, -0.5, 1.2, 0, 0.8, 0"` |
| **IsBolck** | Dashboard 统一用 `false` (不阻塞 Executor) |
| **运动类型** | 关节空间点到点 (各关节独立运动, 路径不保证直线) |

**ROS2 调用示例**:
```bash
ros2 service call /aubo/move_joint std_srvs/srv/Trigger {}
# 目标关节位置从参数读取, 运行时可设:
ros2 param set /aubo_dashboard move_joint_target "0,-0.5,1.2,0,0.8,0"
```

### 2.2 robotServiceLineMove (直线运动)
```cpp
int robotServiceLineMove(aubo_robot_namespace::wayPoint_S &wayPoint, bool IsBolck);
int robotServiceLineMove(double jointAngle[ARM_DOF], bool IsBolck);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ➖ 已映射, 待实现完整服务 |
| **ROS2 接口** | `/aubo/move_line` |
| **运动类型** | 笛卡尔空间直线 (TCP 沿直线运动, 姿态线性插值) |
| **注意** | 直线运动会触发逆解, 奇异点附近可能失败 |

### 2.3 robotServiceRotateMove (旋转运动)
```cpp
int robotServiceRotateMove(
    const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord,
    const double rotateAxisOnUserCoord[3], double rotateAngle, bool IsBolck);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ➖ 已映射, 待实现 |
| **ROS2 接口** | `/aubo/move_rotate` |
| **参数** | 用户坐标系 + 转轴方向 `[x,y,z]` + 转角(rad) |
| **用途** | 绕指定轴纯旋转运动, 保持 TCP 位置不变 |

### 2.4 robotServiceTrackMove (轨迹运动)
```cpp
int robotServiceTrackMove(aubo_robot_namespace::move_track subMoveMode, bool IsBolck);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ➖ |
| **ROS2 接口** | `/aubo/move_track` |
| **子模式** | `ARC_CIR`(圆弧/圆), `CARTESIAN_MOVEP`(MOVEP), `CARTESIAN_CUBICSPLINE`, `JOINT_UBSPLINEINTP` |
| **前置条件** | 需先通过 `robotServiceAddGlobalWayPoint` 构建路点列表 |
| **交融半径** | 通过 `robotServiceSetGlobalBlendRadius` 设置 (MOVEP 模式) |

**使用流程**:
```
1. robotServiceClearGlobalWayPointVector()
2. robotServiceAddGlobalWayPoint() ×N  (添加路点)
3. robotServiceSetGlobalBlendRadius(0.02)  (MOVEP 需要)
4. robotServiceTrackMove(CARTESIAN_MOVEP, false)
```

### 2.5 robotServiceTeachStart / robotServiceTeachStop (示教)
```cpp
int robotServiceTeachStart(aubo_robot_namespace::teach_mode mode, bool direction);
int robotServiceTeachStop();
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `/aubo/teach_start`, `/aubo/teach_stop` |
| **示教模式** | `JOINT1`~`JOINT6`(关节示教), `MOV_X/Y/Z`(位置示教), `ROT_X/Y/Z`(姿态示教) |
| **方向** | `true`=正方向, `false`=反方向 |
| **注意** | 示教模式下机器人持续运动直到调用 `teachStop` |

### 2.6 rootServiceRobotMoveControl (运动控制)
```cpp
int rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand cmd);
// RobotMoveStop=0, RobotMovePause, RobotMoveContinue
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `/aubo/stop`, `/aubo/pause`, `/aubo/resume` |
| **与 robotMoveFastStop 区别** | FastStop 立即停止 (不减速), Stop 正常减速停止 |

### 2.7 其他运动函数

| SDK Function | 签名 | 用途 | ROS2 接口 |
|-------------|------|------|---------|
| `robotServiceFollowModeJointMove` | `int(double[ARM_DOF])` | 跟随模式关节运动 | ➖ 未使用 |
| `robotMoveLineToTargetPosition` | `int(const Coord..., const Pos&, const ToolInEndDesc&, bool)` | 保持姿态直线运动到目标位置 | ➖ 未使用 |
| `robotMoveJointToTargetPosition` | `int(const Coord..., const Pos&, const ToolInEndDesc&, bool)` | 保持姿态关节运动到目标位置 | ➖ 未使用 |
| `robotMoveLineToTargetPositionByRelative` | `int(const Coord..., const MoveRelative&, bool)` | 相对偏移直线运动 | ➖ 未使用 |
| `robotMoveJointToTargetPositionByRelative` | `int(const Coord..., const MoveRelative&, bool)` | 相对偏移关节运动 | ➖ 未使用 |
| `getJointAngleByTargetPositionKeepCurrentOri` | `int(const Coord..., const Pos&, const ToolInEndDesc&, double*)` | 保持姿态 IK 计算 | ➖ 未使用 |

---

## 3. 运动属性

> 运动属性函数用于配置 SDK 内置的运动规划器参数。
> 所有 Set 函数应在运动停止时调用, Get 函数始终可用。

### 3.1 全局运动速度/加速度设置

```cpp
int robotServiceInitGlobalMoveProfile();                             // 恢复默认值
int robotServiceSetGlobalMoveJointMaxAcc(const JointVelcAccParam&);  // 关节最大加速度 (°/s²)
int robotServiceSetGlobalMoveJointMaxVelc(const JointVelcAccParam&); // 关节最大速度 (°/s)
int robotServiceSetGlobalMoveEndMaxLineAcc(double);                  // 末端直线最大加速度 (m/s²)
int robotServiceSetGlobalMoveEndMaxLineVelc(double);                 // 末端直线最大速度 (m/s)
int robotServiceSetGlobalMoveEndMaxAngleAcc(double);                 // 末端姿态最大角加速度 (rad/s²)
int robotServiceSetGlobalMoveEndMaxAngleVelc(double);                // 末端姿态最大角速度 (rad/s)
int robotServiceSetJerkAccRatio(double);                             // 加加速度比例
```

| 维度 | 说明 |
|------|------|
| **默认值** | 关节速度 25°/s, 关节加速度 25°/s², 末端速度 3m/s, 末端加速度 3m/s² |
| **ROS2 接口** | Dashboard 参数, 运行时通过 `ros2 param set` 修改 |
| **生效时机** | 下次调用运动 API 时生效 |
| **Get 函数** | 每个 Set 都有对应的 Get, 用于查询当前值 |

**示例**:
```bash
ros2 param set /aubo_dashboard joint_max_vel "30.0,30.0,30.0,45.0,45.0,45.0"
ros2 service call /aubo/move_joint std_srvs/srv/Trigger {}  # 新速度生效
```

### 3.2 路点管理 (用于 robotServiceTrackMove)

```cpp
void robotServiceClearGlobalWayPointVector();                          // 清空路点列表
int robotServiceAddGlobalWayPoint(const wayPoint_S &wayPoint);         // 添加路点 (笛卡尔)
int robotServiceAddGlobalWayPoint(const double jointAngle[ARM_DOF]);   // 添加路点 (关节角)
void robotServiceGetGlobalWayPointVector(vector<wayPoint_S> &);        // 获取路点列表
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | 内部使用 (轨迹控制器), 或 Dashboard `/aubo/add_waypoint` `/aubo/clear_waypoints` |

### 3.3 交融半径 (MOVEP 模式)

```cpp
float robotServiceGetGlobalBlendRadius();     // 获取交融半径, 范围 0.0~0.05m
int robotServiceSetGlobalBlendRadius(float);  // 设置交融半径
```

| 维度 | 说明 |
|------|------|
| **用途** | 在 MOVEP 轨迹模式下, 路点之间平滑过渡 |
| **ROS2 接口** | `/aubo/set_blend_radius` 或参数 `blend_radius` |

### 3.4 圆弧/圆轨迹

```cpp
int robotServiceGetGlobalCircularLoopTimes();   // 获取圆轨迹圈数
void robotServiceSetGlobalCircularLoopTimes(int); // 0=圆弧, >0=圆 (圈数)
```

### 3.5 提前到位 (Arrival Ahead)

```cpp
int robotServiceSetNoArrivalAhead();                           // 关闭提前到位
int robotServiceSetArrivalAheadDistanceMode(double distance);  // 距离模式 (米)
int robotServiceSetArrivalAheadTimeMode(double second);        // 时间模式 (秒)
int robotServiceSetArrivalAheadBlendDistanceMode(double);      // 交融距离模式 (米)
```

| 维度 | 说明 |
|------|------|
| **用途** | 允许机器人在到达目标前就平滑过渡到下一个目标 (提高效率) |
| **ROS2 接口** | 参数 `arrival_ahead_dist`, `arrival_ahead_time` |

### 3.6 偏移量

```cpp
int robotServiceSetMoveRelativeParam(const MoveRelative &relativeMoveOnBase);         // 基于基座标系
int robotServiceSetMoveRelativeParam(const MoveRelative &, const CoordCalibrate...&); // 基于用户坐标系
```

| 维度 | 说明 |
|------|------|
| **用途** | 运动时叠加偏移量 (如焊接轨迹偏移) |
| **ROS2 接口** | `/aubo/set_move_relative` |

### 3.7 示教坐标系

```cpp
int robotServiceSetTeachCoordinateSystem(const CoordCalibrateByJointAngleAndTool &coordSystem);
```

| 维度 | 说明 |
|------|------|
| **用途** | 设置示教运动的参考坐标系 (默认基座标系) |

---

## 4. TCP2CAN 透传模式

> 轨迹流控制的核心接口。进入 TCP2CAN 后, 控制器停止自身轨迹规划,
> 等待外部通过 CAN 总线逐点下发关节位置。

### 4.1 robotServiceEnterTcp2CanbusMode / LeaveTcp2CanbusMode
```cpp
int robotServiceEnterTcp2CanbusMode();
int robotServiceLeaveTcp2CanbusMode();
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ 轨迹控制器 `on_activate()` / `on_deactivate()` |
| **ROS2 接口** | `HardwareInterface::enterTcp2CanbusMode()` / `leaveTcp2CanbusMode()` |
| **模式互斥** | TCP2CAN ON → SDK 运动 API 不可用 (见 `SDK_CONFLICT_RULES.md`) |
| **RIB** | 进入 TCP2CAN 后, 控制器的 RIB (Robot Interface Buffer) 开始接收数据 |

### 4.2 robotServiceSetRobotPosData2Canbus (单点/批量)
```cpp
int robotServiceSetRobotPosData2Canbus(double jointAngle[ARM_DOF]);                  // 单点
int robotServiceSetRobotPosData2Canbus(const vector<wayPoint_S> &wayPointVector);     // 批量
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ 主轨迹流 (批量) + OTG 紧急减速 (单点) |
| **ROS2 接口** | `HardwareInterface::writeTrajectoryPoints()` / `writeTrajectoryPoint()` |
| **RIB 消费** | 机器人以 ~200Hz 消费, 每个路点占 6 个 RIB 槽位 |
| **延迟尖峰** | 批量调用平均 5-15ms, 尖峰可达 225ms (EMA 补偿) |
| **流量控制** | 基于真实 RIB 填充量自适应调整发送批量 |

**流量控制策略** (参考 `PORTING_MOTION_FIX.md`):
```cpp
// RIB=400 为满载, RIB<200 时加速发送, RIB=0 时 1ms 等待
if (current_macsz < expect_macsz && queue_not_empty) {
    int cnt = min(max_cnt_per_send, ceil((expect_macsz - current_macsz) / 6.0));
    auto waypoints = tryPopWaypoint(cnt);
    hw->writeTrajectoryPoints(waypoints);
}
```

---

## 5. 工具接口

> 工具配置用于描述安装在法兰盘末端的工具 (夹爪、焊枪等) 的物理属性。
> 正确设置工具参数影响: 动力学模型精度、碰撞检测灵敏度、运动规划轨迹质量。

### 5.1 工具动力学参数
```cpp
int robotServiceSetNoneToolDynamicsParam();                          // 清空 (无工具)
int robotServiceSetToolDynamicsParam(const ToolDynamicsParam &param); // 设置
int robotServiceGetToolDynamicsParam(ToolDynamicsParam &param);       // 查询
```

```cpp
// ToolDynamicsParam 结构体
typedef struct {
    double positionX, positionY, positionZ;  // 工具重心位置 (相对法兰盘)
    double payload;                          // 工具质量 (kg)
    ToolInertia toolInertia;                // 工具惯量 (预留, 设为0)
} ToolDynamicsParam;
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `/aubo/set_payload` (Trigger, 参数 `payload_mass`) |
| **生效时机** | 下次运动指令时生效 |
| **默认值** | 重心 (0,0,0), 质量 0 |

### 5.2 工具运动学参数 (TCP)
```cpp
int robotServiceSetNoneToolKinematicsParam();                           // 清空 (TCP在法兰盘中心)
int robotServiceSetToolKinematicsParam(const ToolKinematicsParam &param); // 设置
int robotServiceGetToolKinematicsParam(ToolKinematicsParam &param);       // 查询
int robotServiceSetRobotTool(const ToolInEndDesc &robotTool);            // 设置工具 (别名)
```

```cpp
// ToolInEndDesc (即 ToolKinematicsParam) 结构体
typedef struct {
    Pos toolInEndPosition;       // TCP 相对法兰盘的位置 (x,y,z, 米)
    Ori toolInEndOrientation;    // TCP 相对法兰盘的姿态 (w,x,y,z)
} ToolInEndDesc;
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `/aubo/set_tool_kinematics` |
| **用途** | 设置 TCP (工具中心点) 位置和姿态 |
| **示例** | 夹爪长度 0.15m, 沿 Z 轴 → `(0, 0, 0.15)` |

### 5.3 工具标定
```cpp
int robotServiceToolCalibration(
    const vector<wayPoint_S> &wayPointPosVector,   // 多个位姿 (位置标定)
    char poseCalibMethod,                           // 标定方法
    ToolInEndDesc &toolInEndDesc);                  // 输出: 标定结果

int robotServiceToolCalibration(
    const vector<wayPoint_S> &wayPointPosCalibVector,
    const vector<wayPoint_S> &wayPointOriCalibVector,
    ToolKinematicsOriCalibrateMathod poseCalibMethod,
    ToolInEndDesc &toolInEndDesc);
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `/aubo/calibrate_tool` |
| **原理** | 控制机器人到达多个位姿, 通过最小二乘法计算 TCP |

---

## 6. IO 接口

> AUBO 控制器提供 3 类 IO:
> - **接口板 IO** (Board IO): UserDI/DO (16+16), ControllerDI/DO (8+8), UserAI/AO (4+4)
> - **工具端 IO** (Tool IO): 4 路可配置数字 IO (DI/DO), 2 路 AI
> - **安全 IO** (Safety IO): 紧急停止、防护停止等

### 6.1 读取 IO 状态
```cpp
int robotServiceGetBoardIOStatus(
    const vector<RobotIoType> ioType,        // 输入: 要查询的 IO 类型列表
    vector<RobotIoDesc> &statusVector);      // 输出: IO 状态列表

int robotServiceGetBoardIOConfig(
    const vector<RobotIoType> &ioType,
    vector<RobotIoDesc> &configVector);      // IO 配置 (名称、地址等)

int robotServiceGetAllToolDigitalIOStatus(vector<RobotIoDesc> &);
int robotServiceGetAllToolAIStatus(vector<RobotIoDesc> &);
int robotServiceGetToolIoStatus(string name, double &value);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ `readSafetyIOStatus()` / `readFullIOStatus()` |
| **ROS2 接口** | `HardwareInterface` 内部读取 + `StateBroadcaster` 定时发布到 `/aubo_driver/io_states` |
| **查询频率** | 50Hz (运动期间跳过, 避免 SDK 阻塞竞争) |

**IO 类型枚举**:
```cpp
enum RobotIoType {
    RobotBoardUserDI, RobotBoardUserDO,           // 用户数字 IO
    RobotBoardControllerDI, RobotBoardControllerDO, // 控制器数字 IO
    RobotBoardUserAI, RobotBoardUserAO,           // 用户模拟 IO
    RobotToolAO                                     // 工具端模拟输出
};
```

### 6.2 写入 IO
```cpp
// 按地址设置
int robotServiceSetBoardIOStatus(RobotIoType type, int addr, double value);
// 按名称设置
int robotServiceSetBoardIOStatus(RobotIoType type, string name, double value);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ `HardwareInterface::writeIOCommand()` |
| **ROS2 接口** | `/aubo/set_io` (demo_interface::srv::SetRobotIO) |
| **IO 类型** | `digital_output` → DO, `analog_output` → AO, `tool_io` → 工具 IO |

**示例**:
```bash
# 设置数字输出通道0为高电平
ros2 service call /aubo/set_io demo_interface/srv/SetRobotIO \
  "{io_type: 'digital_output', io_index: 0, value: 1.0}"
```

### 6.3 工具端 IO 配置
```cpp
int robotServiceSetToolPowerVoltageType(ToolPowerType type);     // 0/12/24V
int robotServiceGetToolPowerVoltageType(ToolPowerType &type);
int robotServiceGetToolPowerVoltageStatus(double &value);

int robotServiceSetToolDigitalIOType(ToolDigitalIOAddr addr, ToolIOType type); // 设为输入/输出
int robotServiceSetToolDOStatus(ToolDigitalIOAddr addr, IO_STATUS value);      // 输出值
int robotServiceSetToolPowerTypeAndDigitalIOType(ToolPowerType, ToolIOType×4); // 批量配置
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `HardwareInterface::writeToolIOCommand()` / `writeToolPowerType()` |
| **工具 IO 地址** | `TOOL_DIGITAL_IO_0`~`TOOL_DIGITAL_IO_3` |

---

## 7. 运动学

> 正解 (FK): 关节角 → 笛卡尔位姿。逆解 (IK): 笛卡尔位姿 → 关节角。
> 纯计算, 不依赖 TCP2CAN 状态, 始终可在 conn_status_ 上调用。

### 7.1 正解 (FK)
```cpp
int robotServiceRobotFk(const double *jointAngle, int size, wayPoint_S &wayPoint);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ |
| **ROS2 接口** | `/aubo/get_fk` (aubo_msgs::srv::GetFK) |
| **输入** | 6 个关节角 (rad) |
| **输出** | 笛卡尔位置 (x,y,z, 米) + 姿态四元数 (w,x,y,z) |

### 7.2 逆解 (IK)
```cpp
int robotServiceRobotIk(
    const double *startPointJointAngle,   // 参考关节角 (用于选解)
    const Pos &position,                  // 目标位置
    const Ori &ori,                       // 目标姿态
    wayPoint_S &wayPoint);                // 输出: 关节角

int robotServiceRobotIk(
    const Pos &position,
    const Ori &ori,
    vector<wayPoint_S> &wayPointVector);  // 输出: 所有解
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ |
| **ROS2 接口** | `/aubo/get_ik` (aubo_msgs::srv::GetIK) |
| **参考关节角** | IK 有多解, 选最接近 `startPointJointAngle` 的解 |

### 7.3 坐标变换
```cpp
static int baseToUserCoordinate(const Pos&, const Ori&, const CoordCalibrateByJointAngleAndTool&,
                                 const ToolInEndDesc&, Pos&, Ori&);
static int userToBaseCoordinate(const Pos&, const Ori&, const CoordCalibrateByJointAngleAndTool&,
                                 const ToolInEndDesc&, Pos&, Ori&);
static int baseToBaseAdditionalTool(const Pos&, const Ori&, const ToolInEndDesc&, Pos&, Ori&);
static int userCoordPointToBasePoint(const Pos&, const CoordCalibrateByJointAngleAndTool&, Pos&);
static int endOrientation2ToolOrientation(Ori& tcpOriInEnd, const Ori& endOri, Ori& toolOri);
static int toolOrientation2EndOrientation(Ori& tcpOriInEnd, const Ori& toolOri, Ori& endOri);
static int getTargetWaypointByPosition(const wayPoint_S&, const CoordCalibrateByJointAngleAndTool&,
                                        const Pos&, const ToolInEndDesc&, wayPoint_S&);
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `/aubo/base_to_user`, `/aubo/user_to_base` 等 |
| **用途** | 基座标系 ↔ 用户坐标系 ↔ 工具坐标系的位姿变换 |

### 7.4 四元数 ↔ 欧拉角
```cpp
int quaternionToRPY(const Ori &ori, Rpy &rpy);
int RPYToQuaternion(const Rpy &rpy, Ori &ori);
int robotServiceOriMatrixToQuaternion(double eerot[], Ori &result);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ➖ 驱动在 `fillCartesianPoseAndRpy()` 中自行计算 |
| **ROS2 接口** | `/aubo/quat_to_rpy`, `/aubo/rpy_to_quat` |

---

## 8. 状态/诊断

### 8.1 关节状态
```cpp
int robotServiceGetCurrentWaypointInfo(wayPoint_S &wayPoint);     // 当前法兰盘位姿
int robotServiceGetJointAngleInfo(JointParam &jointParam);        // 当前关节角
int robotServiceGetRobotJointStatus(JointStatus *jointStatus, int size); // 关节角+速度+电流+温度
```

| SDK 调用 | ROS2 接口 | 频率 | 包含数据 |
|---------|---------|------|---------|
| `getCurrentWaypointInfo` | `HardwareInterface::readJointState()` | ≤200Hz | 6关节角 + 笛卡尔位姿 |
| `getJointAngleInfo` | 状态话题 | 50Hz | 6关节角 |
| `getRobotJointStatus` | `/aubo/get_joint_status` 服务 | 按需 | 关节角+速度+电流+温度+错误码 |

```cpp
// JointStatus 结构体
typedef struct {
    int    jointCurrentI;       // 关节电流
    int    jointSpeedMoto;      // 关节速度
    float  jointPosJ;           // 关节角 (rad)
    // ... 还有温度、错误码等
} JointStatus;
```

### 8.2 诊断与状态
```cpp
int robotServiceGetRobotDiagnosisInfo(RobotDiagnosis &robotDiagnosisInfo); // 诊断 (含 RIB)
int robotServiceGetRobotCurrentState(RobotState &state);                    // 当前状态
int robotServiceGetMacCommunicationStatus(bool &value);                     // MAC 通信状态
int robotServiceGetIsRealRobotExist(bool &value);                           // 是否有真实机器人
int robotServiceGetRobotDevInfoService(RobotDevInfo &devInfo);              // 设备信息
int robotServiceGetRobotWorkMode(RobotWorkMode &mode);                      // 仿真/真实模式
int robotServiceGetRobotGravityComponent(RobotGravityComponent &);          // 重力分量
int robotServiceIsOnlineMode(bool &isOnlineMode);                           // 联机模式
int robotServiceIsOnlineMasterMode(bool &isOnlineMasterMode);               // 联机主模式
int robotServiceGetJoint6Rotate360EnableFlag(bool &value);                  // J6 360° 旋转
```

| SDK 调用 | ROS2 接口 | 说明 |
|---------|---------|------|
| `getRobotDiagnosisInfo` | `HardwareInterface::readDiagnosis()` | RIB 缓冲量, 流量控制核心 |
| `getIsRealRobotExist` | `HardwareInterface::init()` 内部 | 区分仿真/真实机器人 |
| `getRobotDevInfoService` | `/aubo/get_robot_info` | 序列号、固件版本等 |
| 其余 Get 函数 | `/aubo/get_*` 服务 或 状态话题 | 按需查询或定时发布 |

### 8.3 碰撞与安全
```cpp
int robotServiceGetRobotCollisionCurrentService(int &collisionGrade);   // 查询碰撞等级
int robotServiceSetRobotCollisionClass(int grade);                      // 设置碰撞等级 (1-10)
int robotServiceCollisionRecover();                                     // 碰撞恢复
int robotServiceSetRobotMaxACC(int maxAcc);                             // 最大加速度
int robotServiceGetRobotSafetyConfig(RobotSafetyConfig &safetyConfig);  // 安全配置
int robotServiceSetRobotSafetyConfig(const RobotSafetyConfig &);        // 设置安全配置
int robotServiceGetOrpeSafetyStatus(OrpeSafetyStatus &safetyStatus);    // ORPE 安全状态
```

| 维度 | 说明 |
|------|------|
| **碰撞等级** | 1(敏感)~10(不敏感), 默认6 |
| **碰撞恢复** | 碰撞后必须调用此函数才能继续运动 |
| **ROS2 接口** | `/aubo/set_collision_class`, `/aubo/collision_recover`, `/aubo/get_safety_config` |

---

## 9. 控制 (上电/关/刹车)

### 9.1 启动与关机
```cpp
int rootServiceRobotStartup(
    const ToolDynamicsParam &toolDynamicsParam,   // 工具动力学参数
    uint8 collisionClass,                         // 碰撞等级
    bool readPose,                                // 是否读取当前位置
    bool staticCollisionDetect,                   // 静态碰撞检测
    int boardBaxAcc,                              // 接口板最大加速度
    ROBOT_SERVICE_STATE &result,                  // 输出: 结果状态
    bool IsBolck = true);                         // 是否阻塞等待完成

int robotServiceRobotShutdown(bool IsBolck = true);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ `startup` 在 Dashboard 中使用 |
| **ROS2 接口** | `/aubo/startup` (阻塞, 2-5s), `/aubo/shutdown` |
| **执行内容** | `startup`: 上电 + 松刹车 + 设置碰撞等级 + 设置动力学参数 |

### 9.2 电源与刹车
```cpp
int robotServicePowerControl(bool value);     // true=上电, false=断电
int robotServiceReleaseBrake();               // 松刹车
int rootServiceRobotControl(RobotControlCommand cmd); // 通用控制命令
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `/aubo/power_control`, `/aubo/brake_release` |

### 9.3 安全 IO 与 ORPE
```cpp
int robotServiceSetRobotOrpePause(uint8 data);           // 接口板上位机暂停
int robotServiceSetRobotOrpeStop(uint8 data);            // 接口板上位机停止
int robotServiceSetRobotOrpeError(uint8 data[], int len); // 接口板上位机错误
int robotServiceClearSystemEmergencyStop(uint8 data);    // 解除系统急停
int robotServiceClearReducedModeError(uint8 data);       // 解除缩减模式错误
int robotServiceRobotSafetyguardResetSucc(uint8 data);   // 防护重置成功
int robotServiceSetRobotAtOriginPose();                  // 回原点
```

| 维度 | 说明 |
|------|------|
| **ROS2 接口** | `/aubo/safety_reset` 等 (待实现) |
| **用途** | 与外部安全系统集成时使用 |

---

## 10. SDK 回调

> SDK 通过内部线程推送数据, 用户注册回调函数接收。
> **警告**: 回调在 SDK 内部线程执行, 不能调用任何 SDK API。

### 10.1 事件回调 (RobotEventCallback)
```cpp
typedef void (*RobotEventCallback)(const RobotEventInfo *eventInfo, void *arg);

struct RobotEventInfo {
    RobotEventType  eventType;       // 事件类型
    int             eventCode;       // 事件代码
    string          eventContent;    // 事件描述
};

int robotServiceRegisterRobotEventInfoCallback(RobotEventCallback ptr, void *arg);
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ 已在 `HardwareInterface` 中注册 |
| **默认** | 事件回调**默认启用, 无法关闭** |
| **关键事件** | `socketDisconnected`, `remoteEmergencyStop`, `softEmergency`, `collision`, `armCanbusError`, `ArmPowerOff`, `MacDataInterruptWarning` |

**完整事件列表** (49个):
```
armCanbusError, remoteHalt, remoteEmergencyStop, jointError,
forceControl, exitForceControl, softEmergency, exitSoftEmergency,
collision, collisionStatusChanged, tcpParametersSucc, powerChanged,
ArmPowerOff, mountingPoseChanged, encoderError, encoderLinesError,
singularityOverspeed, currentAlarm, toolioError,
robotStartupPhase, robotStartupDoneResult, robotShutdownDone,
atTrackTargetPos, SetPowerOnDone, ReleaseBrakeDone,
robotControllerStateChaned, robotControllerError,
socketDisconnected, robotControlException, trackPlayInterrupte,
staticCollisionStatusChanged, MountingPoseWarning,
MacDataInterruptWarning, ToolIoError, InterfacBoardSafeIoEvent,
RobotHandShakeSucc, RobotHandShakeFailed, RobotErrorInfoNotify,
InterfacBoardDIChanged, InterfacBoardDOChanged,
InterfacBoardAIChanged, InterfacBoardAOChanged,
UpdateJoint6Rot360Flag, RobotMoveControlDone/StopDone/PauseDone/ContinueDone,
RobotSwitchToOnlineMaster/Slave, ConveyorTrackRobotStartup/Catchup
```

### 10.2 关节状态回调 (RealTimeJointStatusCallback)
```cpp
typedef void (*RealTimeJointStatusCallback)(
    const JointStatus *jointStatus, int size, void *arg);

int robotServiceSetRealTimeJointStatusPush(bool enable);         // 先启用推送
int robotServiceRegisterRealTimeJointStatusCallback(ptr, arg);   // 再注册回调
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ✅ 在 `registerCallbacks()` 中可选注册 |
| **推送频率** | SDK 内部决定 (通常 ~100Hz) |
| **优势** | 比轮询 `getCurrentWaypointInfo` 延迟更低 |

### 10.3 路点回调 (RealTimeRoadPointCallback)
```cpp
typedef void (*RealTimeRoadPointCallback)(const wayPoint_S *wayPoint, void *arg);
```

### 10.4 末端速度回调 (RealTimeEndSpeedCallback)
```cpp
typedef void (*RealTimeEndSpeedCallback)(double speed, void *arg);
```

---

## 11. 传送带跟踪 (15 函数)

> 用于与传送带同步的视觉抓取场景。需要在真实传送带硬件环境下使用。

```
robotServiceSetConveyorEncoderReset()      robotServiceEnableConveyorTrack()
robotServiceSetConveyorStartup()           robotServiceGetConveyorEncoderVal()
robotServiceSetConveyorStop()              robotServiceSetRobotConveyorTrackMaxVelc()
robotServiceSetConveyorDir()               robotServiceSetRobotConveyorTrackMaxAcc()
robotServiceSetConveyorVelc()              robotServiceSetRobotConveyorSystemDelay()
robotServiceSetEncoderValPerMeter()        robotServiceAppendObject2ConveyorTrackQueue()
robotServiceSetStartWindowUpstream()       robotServiceSetStartWindowDownstream()
robotServiceSetConveyorTrackDownstream()
```

| 维度 | 说明 |
|------|------|
| **使用状态** | ➖ 全部未使用, 预留 Dashboard `/aubo/conveyor_*` 服务 |

---

## 12. 离线轨迹 (5 函数)

```
robotServiceOfflineTrackWaypointAppend(vector)  robotServiceOfflineTrackMoveStartup()
robotServiceOfflineTrackWaypointAppend(file)    robotServiceOfflineTrackMoveStop()
robotServiceOfflineTrackWaypointClear()
```

| 维度 | 说明 |
|------|------|
| **用途** | 从文件加载预录轨迹并回放 |
| **替代方案** | 可通过 `joint_trajectory_controller` 的 FollowJointTrajectory 实现标准轨迹回放 |

---

## 13. 固件升级 (3 函数)

```cpp
int robotServiceUpdateRobotBoardFirmware(cmd, data, length);
int robotServiceGetBoardFirmwareUpdateResultService(bool &value);
int robotServiceGetRobotEthernetDeviceName(string &ethernetDeviceName);
```

| 维度 | 说明 |
|------|------|
| **建议** | 固件升级通过独立工具完成, 不集成到 ROS2 节点 |

---

## 14. 其他 (Weave/Recognition/Camera)

```cpp
int robotServiceSetWeaveMoveParameters(const WeaveMove &weaveMove);              // 摆动焊接
int robotServiceSetRobotRecognitionParam(const RobotRecongnitionParam &param);    // 视觉识别参数
int robotServiceGetRobotRecognitionParam(int type, RobotRecongnitionParam &param);
int robotServiceSetRobotCameraCalib(const RobotCameraCalib &robotCameraCalib);   // 手眼标定
int robotServiceSetRobotJointOffset(RobotJointOffset &jointOffset);              // 关节零位补偿
```

| 维度 | 说明 |
|------|------|
| **WeaveMove** | 焊接摆动参数 (振幅、频率、波形) |
| **JointOffset** | 关节碰撞补偿, 范围 0.00~0.51° |

---

## 使用状态统计

| 类别 | 总数 | ✅ 已使用 | ➖ 已映射(预留) | 待评估 |
|------|:---:|:---:|:---:|:---:|
| 系统/连接 | 5 | 2 | 2 | 1 |
| 运动控制 | 16 | 2 | 8 | 6 |
| 运动属性 | 28 | 0 | 20 | 8 |
| TCP2CAN | 4 | 4 | 0 | 0 |
| 工具接口 | 9 | 0 | 9 | 0 |
| IO 接口 | 16 | 6 | 10 | 0 |
| 运动学 | 15 | 2 | 11 | 2 |
| 状态/诊断 | 17 | 3 | 12 | 2 |
| 控制 | 8 | 1 | 6 | 1 |
| SDK 回调 | 7 | 3 | 4 | 0 |
| 传送带 | 15 | 0 | 15 | 0 |
| 离线轨迹 | 5 | 0 | 5 | 0 |
| 安全 IO/ORPE | 10 | 0 | 10 | 0 |
| 固件 | 3 | 0 | 3 | 0 |
| 其他 | 5 | 0 | 5 | 0 |
| **总计** | **~163** | **23** | **120** | **20** |

---

## 实现优先级

### P0: 已实现 (核心路径)
- ✅ TCP2CAN 轨迹流 → `HardwareInterface`
- ✅ 关节状态读取 → `readJointState`
- ✅ RIB 诊断 → `readDiagnosis`
- ✅ 系统启动 → `/aubo/startup`
- ✅ 急停/碰撞检测 → `RobotEventCallback`
- ✅ FK/IK → `/aubo/get_fk`, `/aubo/get_ik`
- ✅ 关节运动 → `/aubo/move_joint`

### P1: 建议下一批
- `move_line`, `move_rotate` — 完整直接运动控制
- `set_payload`, `set_tool_kinematics` — 工具配置
- `get_robot_info`, `get_diagnosis`, `get_safety_config` — 诊断查询
- `teach_start/stop` — 示教模式

### P2: 按需实现
- 运动属性 Set/Get (28个) — 运行时参数调优
- 坐标变换 (10个) — 高级应用
- 离线轨迹 (5个) — 录制回放
- 传送带跟踪 (15个) — 视觉抓取产线

### P3: 低优先级/不需要
- 固件升级 — 独立工具
- 相机标定 — 独立工具
- 关节零位补偿 — 工厂校准
