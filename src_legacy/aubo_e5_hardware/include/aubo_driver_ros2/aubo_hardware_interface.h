/*
 * Jazzy AUBO Hardware Interface — SDK 封装层（非 ros2_control）。
 *
 * 职责：
 *   - read():  从机械臂读取关节状态 / RIB 诊断
 *   - write(): 向机械臂下发轨迹点 / IO 命令
 *   - 生命周期: init → enterTcp2Canbus → leaveTcp2Canbus → shutdown
 *   - conn_status_ 上的 SDK 事件/状态推送回调注册
 *
 * 连接模型（2 条 ServiceInterface 连接）：
 *   - conn_control_: TCP2CAN 模式 — 轨迹流 + RIB 诊断；
 *     仅 init(need_control=true) 时登录，status-only 模式下控制面方法不可用
 *   - conn_status_:  普通模式 — 状态查询 + IO + 事件回调；
 *     任何模式都登录（dashboard 用 init(need_control=false) 只建此连接）
 *
 * 线程安全（H2：同一 ServiceInterface 实例无线程安全，同实例必须串行）：
 *   - control_mutex_: 所有经 conn_control_ 的公开方法入口持有
 *   - status_mutex_:  所有经 conn_status_ 的公开方法入口持有；
 *     readJointState 的 last_joints_ 速度微分状态同受此锁保护
 *   - 锁只在单次 SDK 调用 / 一个无 sleep 的重试序列期间持有；
 *     禁止持锁跨 rclcpp 调用 / sleep / publish
 *   - 两条连接的调用点天然分离，任何代码路径不得同时持有两把锁（无锁序问题）
 *   - 原子成员（connected_/tcp2can_mode_/rib_cache_）读取不加锁
 *   - controlService()/statusService() 返回裸 ServiceInterface 引用、
 *     绕过 HI 锁：调用方须自行串行（dashboard 有自己的 sdk_mutex_）
 *   - SDK 回调在 SDK 内部线程执行：只允许原子写 + 日志（H6，既有约束不变）
 */

#ifndef AUBO_DRIVER_ROS2_HARDWARE_INTERFACE_H_
#define AUBO_DRIVER_ROS2_HARDWARE_INTERFACE_H_

#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace aubo_driver {

class AuboHardwareInterface {
public:
    AuboHardwareInterface();
    ~AuboHardwareInterface();

    // ── Lifecycle ──
    // need_control=false：status-only 模式，只登录 conn_status_（dashboard 管理面），
    // 跳过 conn_control_ login，控制面方法不可用
    bool init(const std::string& server_host = "169.254.10.98",
              int server_port = 8899, bool need_control = true,
              int max_retries = 8);
    bool enterTcp2CanbusMode();
    bool leaveTcp2CanbusMode();
    void shutdown();
    bool isConnected() const { return connected_; }
    bool isInTcp2CanMode() const { return tcp2can_mode_; }
    bool stopMotion();
    // 查询是否连接真实机械臂（conn_status_，status_mutex_ 保护）
    bool getIsRealRobotExist(bool& real_exist);

    // ── RT path (control loop) ──
    bool readJointState(double joints[6], double velocity[6] = nullptr);
    bool readDiagnosis(int& rib_buffer_size);
    // H12: RIB 原子缓存（-1 = 尚无有效值），仅供展示/诊断；
    // 禁止回喂 sendLoop 门控——门控必须用当轮实时查询值（Fix14 教训）
    int lastRib() const { return rib_cache_.load(); }
    bool writeTrajectoryPoints(
        const std::vector<aubo_robot_namespace::wayPoint_S>& waypoints);
    bool writeTrajectoryPoint(const double joint_angles[6]);

    // ── Async path (IO / safety) ──
    bool readSafetyIOStatus(bool& emergency_stopped, bool& protective_stopped);
    bool writeIOCommand(aubo_robot_namespace::RobotIoType type,
                        int addr, double value);
    bool writeToolIOCommand(int io_index, bool is_output, double value);
    bool writeToolPowerType(int power_type);

    // ── SDK callbacks ──
    using EventCallback = std::function<void(
        int eventType, int eventCode, const std::string& eventContent)>;
    struct JointFull {
        double pos[6]{}, vel[6]{}, cur[6]{};
        double vol[6]{}, temp[6]{};
        double tgt_cur[6]{}, tgt_vel[6]{}, tgt_pos[6]{};
        int    err[6]{};
    };
    using JointStateCallback = std::function<void(const JointFull&)>;
    using WaypointCallback = std::function<void(
        const aubo_robot_namespace::wayPoint_S& wp)>;
    using SpeedCallback = std::function<void(double speed)>;

    void registerCallbacks(
        EventCallback event_cb = nullptr,
        JointStateCallback joint_cb = nullptr,
        WaypointCallback waypoint_cb = nullptr,
        SpeedCallback speed_cb = nullptr);

    // ── Direct SDK access (for Dashboard) ──
    // 返回裸 ServiceInterface 引用，绕过 HI 的 control_mutex_/status_mutex_；
    // 调用方须自行串行（dashboard 有自己的 sdk_mutex_）
    ServiceInterface& controlService() { return conn_control_; }
    ServiceInterface& statusService()  { return conn_status_; }

private:
    static void onRobotEvent(
        const aubo_robot_namespace::RobotEventInfo* info, void* arg);
    static void onJointStatus(
        const aubo_robot_namespace::JointStatus* status, int size, void* arg);
    static void onRoadPoint(
        const aubo_robot_namespace::wayPoint_S* wp, void* arg);
    static void onEndSpeed(double speed, void* arg);

    ServiceInterface conn_control_;
    ServiceInterface conn_status_;

    // H2: 同一 ServiceInterface 实例无线程安全 —— 按连接分锁；
    // 只在单次 SDK 调用期间持有，禁止持锁跨 rclcpp 调用/sleep
    std::mutex control_mutex_;
    std::mutex status_mutex_;

    std::atomic<bool> connected_{false};
    std::atomic<bool> tcp2can_mode_{false};

    // H12: RIB 原子缓存（-1 = 尚无有效值），readDiagnosis 成功后写入；
    // 仅供展示/诊断，禁止回喂 sendLoop 门控（Fix14 教训）
    std::atomic<int> rib_cache_{-1};

    // conn_control_ 是否已登录（status-only init 时为 false）；
    // 仅 init/shutdown 生命周期路径访问，无需加锁
    bool control_logged_in_{false};

    EventCallback event_cb_;
    JointStateCallback joint_cb_;
    WaypointCallback waypoint_cb_;
    SpeedCallback speed_cb_;
    // registerCallbacks 的推送注册结果（仅日志用，生命周期路径访问）
    bool push_joint_ok_{false};
    bool push_waypoint_ok_{false};
    bool push_speed_ok_{false};

    double last_joints_[6] = {0};
    bool has_last_joints_{false};
    std::chrono::steady_clock::time_point last_read_time_;
};

}  // namespace aubo_driver

#endif  // AUBO_DRIVER_ROS2_HARDWARE_INTERFACE_H_
