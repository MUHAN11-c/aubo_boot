/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017-2018, AUBO Robotics
 * Ported to ROS2: Hardware Interface abstraction layer.
 *
 * 设计参考: UR URPositionHardwareInterface / ros2_control SystemInterface
 *
 * 职责: 封装 AUBO SDK (ServiceInterface) 的所有通信操作，向上层提供:
 *   - read():  从机器人读取状态 (关节位置/速度、IO、诊断) → 反馈给 ROS
 *   - write(): 向机器人发送命令 (轨迹点、IO 命令、运动指令) → 控制机器人
 *   - 生命周期管理: init → enterTcp2Canbus → leaveTcp2Canbus → shutdown
 *   - SDK 回调注册: RobotEventCallback, RealTimeJointStatusCallback 等
 *
 * 连接模型 (精简为 2 个 ServiceInterface):
 *   - conn_control_: TCP2CAN 模式，用于轨迹流发送 + RIB 诊断查询
 *   - conn_status_:  普通模式，用于状态查询 + IO 读写 + SDK 事件回调
 *
 * 线程安全:
 *   - read()/write() 在控制循环线程调用 (RT 路径)
 *   - readIOStatus()/writeIOCommand() 在异步线程调用 (非RT 路径)
 *   - SDK 回调在 SDK 内部线程执行，回调中仅做原子写入 + 日志
 */

#ifndef AUBO_DRIVER_ROS2_AUBO_HARDWARE_INTERFACE_H_
#define AUBO_DRIVER_ROS2_AUBO_HARDWARE_INTERFACE_H_

#include <string>
#include <vector>
#include <atomic>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <ivg_interfaces/msg/robot_io_status.hpp>

#include "aubo_driver_ros2/AuboRobotMetaType.h"
#include "aubo_driver_ros2/serviceinterface.h"

namespace aubo_driver {

/**
 * 硬件接口层 —— 对 AUBO SDK 的完整封装。
 *
 * 使用方式:
 *   1. 构造 AuboHardwareInterface 对象
 *   2. 调用 init(server_host, port) 连接机器人 (内部 login ×2)
 *   3. 调用 registerCallbacks() 注册 SDK 事件/状态回调
 *   4. 调用 enterTcp2CanbusMode() 进入轨迹流模式
 *   5. 控制循环中:
 *        readJointState(joints, velocity)   // 读关节状态
 *        readDiagnosis(rib_size)            // 读 RIB 缓冲量
 *        writeTrajectoryPoints(waypoints)   // 写轨迹点
 *   6. 异步线程中:
 *        readIOStatus(...)    // 读 IO 状态
 *        writeIOCommand(...)  // 写 IO 命令
 *   7. 关闭时:
 *        leaveTcp2CanbusMode()  // 退出轨迹流模式
 *        shutdown()             // logout ×2
 */
class AuboHardwareInterface {
public:
    AuboHardwareInterface();
    ~AuboHardwareInterface();

    // ========================================================================
    // 生命周期接口
    // ========================================================================

    /**
     * 初始化硬件接口 —— 连接机器人控制器。
     * 内部创建 2 个 ServiceInterface 连接并分别 login:
     *   - conn_control_: 用于 TCP2CAN 轨迹流 + RIB 诊断
     *   - conn_status_:  用于状态查询 + IO + 事件回调
     *
     * @param server_host  机器人控制器 IP 地址，默认 "127.0.0.1"
     * @param server_port  机器人控制器端口，默认 8899
     * @param max_retries  login 最大重试次数，默认 5
     * @return true 连接成功, false 连接失败
     */
    bool init(const std::string& server_host = "169.254.10.98",
              int server_port = 8899, int max_retries = 5);

    /**
     * 进入 TCP2CAN 透传模式 —— 启用轨迹流控制。
     * 在此模式下，控制器停止自身轨迹规划，等待外部逐点下发关节位置。
     * 只有 conn_control_ 进入 TCP2CAN 模式，conn_status_ 保持普通模式。
     *
     * @return true 成功, false 失败
     */
    bool enterTcp2CanbusMode();

    /**
     * 退出 TCP2CAN 透传模式 —— 恢复控制器自身轨迹规划。
     * @return true 成功, false 失败
     */
    bool leaveTcp2CanbusMode();

    /**
     * 关闭硬件接口 —— logout 所有连接。
     * 调用前应先 leaveTcp2CanbusMode()。
     */
    void shutdown();

    /**
     * 查询是否已连接机器人控制器。
     * @return true 已连接, false 未连接
     */
    bool isConnected() const { return connected_; }

    /** 查询是否处于 TCP2CAN 透传模式 */
    bool isInTcp2CanMode() const { return tcp2can_mode_; }

    // ========================================================================
    // 实时路径接口 —— 由控制循环线程调用 (read/write 热路径)
    // ========================================================================

    /**
     * 读取当前关节状态 (位置 + 速度)。
     * 内部调用 robotServiceGetCurrentWaypointInfo() → 读取 6 个关节角度。
     * 速度由连续两次 read 之间的位置差分计算。
     *
     * 调用频率: 建议 ≤ 200Hz (SDK 调用约 2-12ms)
     * 线程: 控制循环线程
     *
     * @param joints[out]   输出 6 个关节位置 (rad)，调用者提供 ≥6 的数组
     * @param velocity[out] 输出 6 个关节速度 (rad/s)，可为 nullptr
     * @return true 读取成功, false SDK 调用失败或未连接
     */
    bool readJointState(double joints[6], double velocity[6] = nullptr);

    /**
     * 读取机器人诊断信息 (RIB 缓冲量)。
     * 内部调用 robotServiceGetRobotDiagnosisInfo() → macTargetPosDataSize。
     *
     * RIB (Robot Interface Buffer) 是控制器内部的 CAN 总线发送缓冲区，
     * 以 6 为单位 (6 个关节为一个路点)。缓冲区容量约 400 槽位。
     * RIB=0 意味着控制器无数据可发 → 机器人停止。
     *
     * 调用频率: 建议与轨迹发送同频，用于流量控制
     * 线程: 控制循环线程 (通过 conn_control_)
     *
     * @param rib_buffer_size[out] 输出 RIB 当前填充量
     * @return true 读取成功
     */
    bool readDiagnosis(int& rib_buffer_size);

    /** 在 conn_status_ (普通连接) 上查诊断 — 避免 TCP2CAN 连接缓存问题 */
    bool readDiagnosisOnStatus(int& rib_buffer_size);

    /**
     * 发送轨迹点批量数据到机器人。
     * 内部调用 robotServiceSetRobotPosData2Canbus(vector)。
     *
     * 流量控制策略 (参考 PORTING_MOTION_FIX.md):
     *   - RIB < 200 时加速发送 (每周期多取点)
     *   - 单次最大批量 8 点 (max_cnt_per_send)
     *   - 发送前先查 RIB，基于真实值决定发送量
     *   - TCP 延迟尖峰 (225ms) 通过 EMA 平滑补偿
     *
     * 调用频率: ~250Hz (可配置)
     * 线程: 控制循环线程 (通过 conn_control_)
     *
     * @param waypoints 路点向量，每个路点包含 6 个关节角度
     * @return true 发送成功
     */
    bool writeTrajectoryPoints(
        const std::vector<aubo_robot_namespace::wayPoint_S>& waypoints);

    /**
     * 发送单个轨迹点 (用于 OTG 紧急减速)。
     * 内部调用 robotServiceSetRobotPosData2Canbus(double[6])。
     *
     * @param joint_angles 6 个关节角度 (rad)
     * @return true 发送成功
     */
    bool writeTrajectoryPoint(const double joint_angles[6]);

    // ========================================================================
    // 异步路径接口 —— 由异步线程调用 (非实时，≤50Hz)
    // ========================================================================

    /**
     * 读取控制器接口板 IO 状态 (数字输入/输出)。
     * 内部调用 robotServiceGetBoardIOStatus() 获取 UserDI/DO 和 ControllerDI/DO。
     *
     * 调用频率: 建议 ≤ 50Hz (单次调用约 5-15ms，查询多种 IO 类型)
     * 线程: 异步 IO 线程
     *
     * @param emergency_stopped[out]  输出: 是否紧急停止 (digitalIn[0]==0 或 [8]==0)
     * @param protective_stopped[out] 输出: 是否防护停止 (digitalIn[1]==0 或 [9]==0)
     * @return true 读取成功
     */
    bool readSafetyIOStatus(bool& emergency_stopped, bool& protective_stopped);

    /**
     * 读取所有 IO 状态并填充 RobotIOStatus 消息。
     * 合并了 UserDI/DO, ControllerDI/DO, UserAI/AO, Tool IO 的查询。
     *
     * @param io_msg[out] 输出 IO 状态消息引用
     * @return true 读取成功
     */
    bool readFullIOStatus(ivg_interfaces::msg::RobotIOStatus& io_msg);

    /**
     * 设置接口板 IO 输出。
     * 内部调用 robotServiceSetBoardIOStatus(type, addr, value)。
     *
     * @param type  IO 类型 (RobotBoardUserDO, RobotBoardUserAO, RobotToolAO)
     * @param addr  IO 地址
     * @param value IO 值 (数字: 0/1, 模拟: 电压值)
     * @return true 设置成功
     */
    bool writeIOCommand(aubo_robot_namespace::RobotIoType type,
                        int addr, double value);

    /**
     * 设置工具端数字 IO 类型和状态。
     * 内部调用 robotServiceSetToolDigitalIOType + robotServiceSetToolDOStatus。
     *
     * @param io_index 工具 IO 索引
     * @param is_output true 设为输出模式, false 设为输入模式
     * @param value     IO 值 (仅在 is_output=true 时有效)
     * @return true 设置成功
     */
    bool writeToolIOCommand(int io_index, bool is_output, double value);

    /**
     * 设置工具电源电压类型。
     * 内部调用 robotServiceSetToolPowerVoltageType。
     *
     * @param power_type 电源类型 (0/12/24V)
     * @return true 设置成功
     */
    bool writeToolPowerType(int power_type);

    // ========================================================================
    // SDK 回调接口
    // ========================================================================

    /**
     * 用户回调类型 —— 当 SDK 事件发生时被调用。
     * 回调在 SDK 内部线程执行，不能调用 SDK API，只能做原子写入+日志。
     *
     * @param eventType    事件类型 (RobotEventType 枚举)
     * @param eventCode    事件代码
     * @param eventContent 事件描述字符串
     */
    using EventCallback = std::function<void(
        int eventType, int eventCode, const std::string& eventContent)>;

    /**
     * 用户回调类型 —— 当关节状态推送时被调用。
     * 回调在 SDK 内部线程执行。
     *
     * @param joints     6 个关节位置 (rad)
     * @param velocity   6 个关节速度 (rad/s)
     * @param current    6 个关节电流 (A)
     */
    struct JointFull {
        double pos[6]{}, vel[6]{}, cur[6]{};      // 位置/速度/电流
        double vol[6]{}, temp[6]{};               // 电压/温度
        double tgt_cur[6]{}, tgt_vel[6]{}, tgt_pos[6]{};  // 目标值
        int    err[6]{};                          // 错误码
    };
    using JointStateCallback = std::function<void(const JointFull&)>;

    /**
     * 用户回调类型 —— 当路点推送时被调用。
     * @param wp 当前路点信息 (关节角 + 笛卡尔位姿)
     */
    using WaypointCallback = std::function<void(
        const aubo_robot_namespace::wayPoint_S& wp)>;

    /**
     * 用户回调类型 —— 当末端速度推送时被调用。
     * @param speed 末端速度 (m/s)
     */
    using SpeedCallback = std::function<void(double speed)>;

    /**
     * 注册所有 SDK 回调和启用推送。
     * 调用时机: 在 init() 成功后、enterTcp2CanbusMode() 前调用。
     *
     * 注册的回调:
     *   - RobotEventCallback:  socket 断开、急停、碰撞、供电状态等事件
     *   - RealTimeJointStatusCallback: 关节位置/速度/电流实时推送
     *   - RealTimeRoadPointCallback: 路点实时推送
     *   - RealTimeEndSpeedCallback: 末端速度实时推送
     *
     * @param event_cb  事件回调 (可为 nullptr 表示不注册)
     * @param joint_cb  关节状态回调 (可为 nullptr)
     * @param waypoint_cb 路点回调 (可为 nullptr)
     * @param speed_cb  速度回调 (可为 nullptr)
     */
    void registerCallbacks(
        EventCallback event_cb = nullptr,
        JointStateCallback joint_cb = nullptr,
        WaypointCallback waypoint_cb = nullptr,
        SpeedCallback speed_cb = nullptr);

    // ========================================================================
    // 直接 SDK 调用接口 —— 暴露给 DashboardNode / 高级运动服务
    // ========================================================================

    /**
     * 获取控制连接的 ServiceInterface 引用 (TCP2CAN 模式)。
     * 用于 Dashboard 节点直接调用 motion 相关 API。
     *
     * ⚠️ 调用者负责线程安全 —— 不要在控制循环线程读写时并发调用。
     */
    ServiceInterface& controlService() { return conn_control_; }

    /**
     * 获取状态连接的 ServiceInterface 引用 (普通模式)。
     * 用于 Dashboard 节点直接调用 FK/IK/IO/config 相关 API。
     *
     * ⚠️ 调用者负责线程安全。
     */
    ServiceInterface& statusService() { return conn_status_; }

private:
    // 内部 SDK 回调处理 (静态函数，通过 user_data 访问 this)
    static void onRobotEvent(
        const aubo_robot_namespace::RobotEventInfo* info, void* arg);
    static void onJointStatus(
        const aubo_robot_namespace::JointStatus* status, int size, void* arg);
    static void onRoadPoint(
        const aubo_robot_namespace::wayPoint_S* wp, void* arg);
    static void onEndSpeed(double speed, void* arg);

    // 两个 ServiceInterface 连接
    ServiceInterface conn_control_;   // TCP2CAN 模式: 轨迹流发送 + RIB 诊断
    ServiceInterface conn_status_;    // 普通模式: 状态查询 + IO + 事件回调

    // 连接状态
    std::atomic<bool> connected_{false};
    std::atomic<bool> tcp2can_mode_{false};

    // 用户注册的回调
    EventCallback event_cb_;
    JointStateCallback joint_cb_;
    WaypointCallback waypoint_cb_;
    SpeedCallback speed_cb_;

    // 上次关节位置和时间戳 (用于速度估算)
    double last_joints_[6] = {0};
    bool has_last_joints_{false};
    std::chrono::steady_clock::time_point last_read_time_;
};

}  // namespace aubo_driver

#endif  // AUBO_DRIVER_ROS2_AUBO_HARDWARE_INTERFACE_H_
