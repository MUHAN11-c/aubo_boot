/*
 * AUBO E5 ros2_control SystemInterface 插件（Phase 2，UR 范式）。
 *
 * 设计依据：doc/REFACTORING.md §3（SDK/RIB 专项分析）、§3.8（Jazzy 特性落点）、
 * §四 Phase 2。本类只做"标准化封装"：SDK 客户端复用现有 AuboHardwareInterface
 * （作为成员，其线程安全边界 H1–H13 不动），SDK 阻塞调用全部隔离在内部线程。
 *
 * 线程模型（SDK 阻塞 p50 3.6ms / p99 18ms / ~1.3% 概率 1s 整超时，§8.2）：
 *   - ros2_control RT 线程: read()/write() 零 SDK 调用、零日志、零堆分配——
 *     read() 只拷原子缓存进 state interfaces；write() 只做无锁入队。
 *   - sender 线程: 出队 setpoint → 批量 ≤8 发 TCP2CAN（conn_control_）。
 *     RIB 流控逐字移植自 joint_trajectory_controller.cpp sendLoop（H4/H5/H12）：
 *     有数据每轮实时查 RIB、300/350 溢出保险门控、批量 min2/max8、EMA 耗时补偿、
 *     1/4ms 自适应睡眠、空闲 250ms 降频；唯一改动是目标工作水位带 [60,120] 槽
 *     （§3.3 问题 1：压低取消残余窗口至 47–94ms）+ 冻结检测兜底。
 *   - status 线程: 10Hz 轮询 readJointState（conn_status_），结果写原子缓存
 *     （推送在本固件 + TCP2CAN 组合下断链 2/2 复现，保持关闭；10Hz 为
 *     2026-07-24 实测稳定工作点，服务器对更高频查询节流）。
 *
 * 取消/抢占语义（§3.3 问题 1，RIB 按 ~4.7ms/点消费率排空、非瞬清，§8.7/§8.8）：
 *   stock JTC 取消只停发新 setpoint，HI 收不到显式 abort → 取消残余 = 水位带
 *   自然排空（≤94ms，低水位带本身就是对策）；冻结检测仅作"缓冲卡死"兜底：
 *   setpoint 连续 kFreezeCycles 周期无变化且 RIB>0（超过自然排空时间）→ stopMotion。
 *
 * 队满与断连（§3.3 问题 4）：SPSC 容量 2048（write() 上采样 ×2 后
 *   ≈10s @200pts/s 存量），队满不允许丢点 → 置原子错误标志，
 *   read() 返回 ERROR 走 ros2_control 标准停控制器路径；
 *   断连由 HI onRobotEvent 置 connected_=false，read() 检测到即返回 ERROR。
 *
 * 已知裁剪（相对 REFACTORING §四 Phase 2 完整清单，留 TODO）：
 *   - 无独立 async 线程：stopMotion 由 sender 线程直接调用（内部有 control_mutex_
 *     保护）；IO 轮询/NaN 信箱、trajectory/abort GPIO、activate 消费率校准、
 *     断连自动重连（§8.1 冷却期 ~6s 需退避重试）均留待后续阶段。
 */

#ifndef AUBO_DRIVER_ROS2_AUBO_ROS2_SYSTEM_HPP_
#define AUBO_DRIVER_ROS2_AUBO_ROS2_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "aubo_driver_ros2/aubo_hardware_interface.h"
#include "aubo_driver_ros2/readerwriterqueue.h"

namespace aubo_driver {

class AuboRos2System : public hardware_interface::SystemInterface
{
public:
    using CallbackReturn = hardware_interface::CallbackReturn;

    AuboRos2System() = default;
    ~AuboRos2System() override;

    // ── Lifecycle (H10 顺序: configure=login×2+回调, activate=enterTcp2Canbus+线程,
    //    deactivate=stop→leave, cleanup=logout) ──
    CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface::ConstSharedPtr>
    on_export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface::SharedPtr>
    on_export_command_interfaces() override;

    // ── RT path: 零 SDK 调用、零日志、零堆分配 ──
    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    void senderLoop();   // sendLoop 流控移植 (conn_control_)
    void statusLoop();   // ~30Hz 状态轮询 + 250ms 安全 IO 兜底 (conn_status_)
    void stopThreads();

    static constexpr int kNJoint = 6;

    // RIB 流控参数 (sendLoop 逐字沿用，REFACTORING §六红线)
    static constexpr int kRibGateWarn = 300;   // 溢出保险: 最小批量 2
    static constexpr int kRibGateHigh = 350;   // 溢出保险: 单点
    // 目标工作水位带 [100,200] 槽（2026-07-25 由 [60,120] 上调）：v1.3.1
    // 写路径仍有偶发 ~220ms 尖峰（消费 ~200 pts/s ≈ 44 点），旧带下限 60
    // 恰被排空到 0 形成中段卡顿（m13 实测 RIB_IDLE ×2 + 220ms 尖峰 ×2 实锤）。
    // 100 槽 ≈ 500ms 吸收窗覆盖 p99 尖峰；代价是自然排空尾段 ~1s。
    static constexpr int kRibBandLow = 100;
    static constexpr int kRibBandHigh = 200;
    // 冻结检测: setpoint 连续 ~350 个 sender 周期 (~1.8-3.5s) 无变化且 RIB>0 → stop。
    // 阈值必须大于满水位带自然排空时间（200 槽 ÷ 消费 ~200 pts/s ≈ 1s），
    // 随水位带上调（原 250，对应旧 [60,120] 带）；定位为"缓冲卡死"兜底。
    static constexpr int kFreezeCycles = 350;

    // 逐周期跳变钳制阈值 (回跳/追赶根治)：URDF 关节限速 2.5964 rad/s ×
    // 10ms 周期 = 0.026 rad——合法轨迹（含 1.0 全速缩放）永不触发；
    // 超此值只可能是 hold 重置/取消/恢复追赶产生的瞬时大跳变。
    // （2026-07-25 由 0.0065 上调：0.0065=0.65 rad/s 是 v2.5.3 写产能
    // ~125 pts/s 时代的配套值，换 v1.3.1（~660 pts/s）后会把 1.0 缩放
    // 合法轨迹钳到 0.65 rad/s——RViz 全速真机爬行、偏差累积触发
    // PATH_TOLERANCE 中止，两个用户侧现象同一根因。）
    static constexpr double kMaxStepPerCycle = 0.026;
    // 反馈失新闩锁阈值 12s（2026-07-24 下午修订，原 5s）：SDK 写停滞
    // 0.4-2s 期间服务器全局串行会拖垮 status 查询，实测反馈缺口可 >10s；
    // 5s 会把"停滞伴生失新"误判为失联并杀死整个会话（FAULT_STOP 三连）。
    // 失新期间机械臂只在授权走廊内执行 RIB 存量后原地 hold（路径门
    // ±0.02 rad 仍是硬约束），风险有界；真正的断连由
    // RobotEvent_socketDisconnected 事件即时检测（read() 立即 ERROR），
    // 不依赖本阈值。12s 只作为"推送与轮询双路全死"的最后兜底。
    static constexpr int64_t kFeedbackFaultNs = 12LL * 1000 * 1000 * 1000;
    // "运动中"判据窗口：最近一次实际入队距今 <2s 视为轨迹执行中。
    // 2026-07-24 实测：SDK 查询偶发 ~5s 同步阻塞（约 0.2-0.5%），空闲期
    // 单次阻塞不应闩锁 traj_fault_ 杀死组件（原地 hold 本就是安全态）；
    // 只有轨迹执行中失联 >kFeedbackFaultNs 才闩锁（此时 RIB 已排空、
    // 后续轨迹失明，必须中止）。
    static constexpr int64_t kMotionRecentNs = 2LL * 1000 * 1000 * 1000;
    // 路径走廊容差（2026-07-25 由 0.02 放宽到 0.05）：named_pose 轨迹是
    // 关节空间直线，但 OMPL/RRTConnect + Ruckig 的规划路径合法地偏离直线
    // （实测 wrist1 在进度 96% 处偏 0.0213 rad，0.02 容差把合法路径误杀）。
    // 0.05 仍是有界防护：只放行 home↔camera_pose 连线附近的命令，
    // 挡住 SDK 假数据/错误轨迹级别的偏离（此类偏离通常 ≫0.1 rad）。
    static constexpr double kPathTolerance = 0.05;
    static constexpr double kEndpointTolerance = 0.03;
    // 反馈失新阈值：>0.5s 仅用于 sender 线程的可观测性告警（2026-07-24 起
    // write() 不再据此暂停入队——短暂失明期间在授权直线走廊内继续流式是
    // 安全的，暂停-恢复反而让流式塌缩导致 JTC 中止）；>12s 彻底失联见
    // kFeedbackFaultNs。
    static constexpr int64_t kFeedbackStaleNs = 500LL * 1000 * 1000;
    bool commandOnAllowedPath(
        const std::array<double, kNJoint>& command, double& progress) const noexcept;

    // SDK 客户端 (双连接 + 分锁，H1–H13 边界不动)
    std::unique_ptr<AuboHardwareInterface> hw_;
    std::string robot_ip_{"169.254.10.98"};
    bool allow_motion_commands_{false};
    int robot_port_{8899};
    std::vector<std::string> joint_names_;
    const std::array<double, kNJoint> home_pose_{
        0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0};
    const std::array<double, kNJoint> camera_pose_{
        -0.27411168813705444, 0.4963911175727844, 1.7700852155685425,
        -0.2978658676147461, 1.571584939956665, -0.2750104069709778};
    std::array<double, kNJoint> joint_min_{};
    std::array<double, kNJoint> joint_max_{};

    // 接口存储 (state/command interface 经 value_ptr 绑定，框架直接读写)
    std::array<double, kNJoint> hw_pos_cmd_{};
    std::array<double, kNJoint> hw_pos_state_{};
    std::array<double, kNJoint> hw_vel_state_{};

    // status 线程 → read() 的原子缓存 (SPSC，无锁)
    std::array<std::atomic<double>, kNJoint> state_pos_{};
    std::array<std::atomic<double>, kNJoint> state_vel_{};
    std::atomic<bool> state_valid_{false};
    // 反馈新鲜度时间戳 (steady_clock ns)：status 线程每次读成功时更新。
    // sender 据此做"观测降级"判断（2026-07-22 审查 F1：反馈失新 ≠ 机械臂失控，
    // 只做 WARN/超时故障上报，绝不 stopMotion——否则 JTC 续跑产生追赶跳变）
    std::atomic<int64_t> state_ok_ns_{0};

    // 轨迹级故障闩锁（2026-07-22 审查 F2）：任何"杀死当前轨迹执行"的动作
    // （freeze-stop、安全闩锁、>5s 反馈失新）统一置位 → read() 返回 ERROR →
    // 框架停控制器、goal 被干净 abort。止动与告知必须原子配对。
    std::atomic<bool> traj_fault_{false};
    // 安全闩锁：SDK 安全事件（软急停/远程急停/碰撞/关节错误/CAN 错误/
    // 控制器错误/MacDataInterrupt）或安全 IO 轮询命中 → 置位 → sender 停止并清队。
    // 闩锁保持到 deactivate/cleanup（现场 dashboard recover 后重新 activate 才复位）。
    std::atomic<bool> safety_fault_{false};
    std::atomic<bool> command_fault_{false};
    // 故障原因码（2026-07-24 晚诊断性增强）：静默闩锁点只写原子，
    // senderLoop 的 FAULT_STOP 分支打印——RT 路径保持零日志。
    // 0=none 1=feedback_stale 2=corridor 3=path_reversal 4=queue_overflow
    // 5=diag_fail 6=send_fail 7=freeze 8=safety_event 9=safety_io
    std::atomic<int> fault_cause_{0};
    // cause=2/3 时被拒命令快照（仅 write() RT 线程写 + sender 线程读，
    // rej_seq_ 为奇偶发布协议：写前后各 +1，读者见到奇数则数据 torn 重读）
    std::array<double, kNJoint> rej_cmd_{};
    std::array<double, kNJoint> rej_ref_{};
    double rej_progress_{0.0};
    int rej_site_{0};   // 1=raw corridor 2=clamped corridor 3=reversal
    std::atomic<uint32_t> rej_seq_{0};

    // write() → sender 线程的无锁队列 (容量 2048；write() 上采样 ×2 后
    // ≈10s @200pts/s 存量，§3.3 问题 4)：覆盖 >5s 极端 SDK 写停滞，
    // 停滞期不丢点不溢出。
    moodycamel::ReaderWriterQueue<std::array<double, kNJoint>> setpoint_queue_{2048};
    std::atomic<bool> queue_overflow_{false};   // 队满置位 (不丢点)，read() 报 ERROR
    std::atomic<uint64_t> setpoint_seq_{0};     // 每入队一点 +1，冻结检测用
    std::atomic<int64_t> last_enqueue_ns_{0};   // 最近入队时间，运动中判据用

    // 轨迹激活判据存储 (仅 write() RT 线程访问，无需原子)
    std::array<double, kNJoint> last_written_cmd_{};
    bool has_last_written_cmd_{false};
    double last_path_progress_{0.0};
    int path_direction_{0};

    // TCP2CAN 单批点数上限（硬件参数 tcp2can_batch_max，clamp [1,32]，默认 8）。
    // 2026-07-24 full_test 实测 batch 4/8/16/32 均可被服务器接受（与 07-23
    // 探针"batch 32 被拒"矛盾——批量上限是服务器状态相关的）。大批次 =
    // 更少调用次数 = 更少撞停滞的机会，产能可超消费率使 RIB 攒起缓冲。
    // 运行时降级：大批次重试仍失败 → 拆成 ≤8 小批并降回 8（见 senderLoop）。
    int tcp2can_batch_max_{8};

    // 内部线程 (SDK 阻塞调用全部隔离于此)
    std::thread sender_thread_;
    std::thread status_thread_;
    std::atomic<bool> threads_running_{false};
};

}  // namespace aubo_driver

#endif  // AUBO_DRIVER_ROS2_AUBO_ROS2_SYSTEM_HPP_
