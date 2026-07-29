// ============================================================================
// aubo_e5_hardware.hpp —— AUBO E5 机械臂 ros2_control SystemInterface 插件
// 声明（旧版 SDK 1.3.1，TCP2CAN 位置流 + RIB 流量控制）。
//
// 职责与行为基线：见 aubo_e5_hardware.cpp 文件头（蓝本逐条出处：
// aubo_boot /home/wjz/桌面/aubo_boot，ROS1 /home/wjz/aubo_robot）。
// UR 驱动仅作为 Jazzy ros2_control 风格参照（旧式接口导出、异步 IO 线程）。
//
// 线程模型（哪个函数在哪个线程跑）：
//   - read()/write()   RT 线程（controller_manager 控制循环），纯内存操作。
//   - sendLoop()       发送线程，独占 conn_control_（RIB 查询 + 喂点）。
//   - ioLoop()         IO 异步线程，独占 conn_status_（IO、安全轮询、停止原语）。
//   - SDK 回调         SDK 内部推送线程，只写缓存/原子量。
// 单实例不并发约束：SDK ServiceInterface 实例非线程安全，故两条连接分别
// 只被一个线程触碰（见 conn_control_/conn_status_ 声明处注释）。
// ============================================================================

#ifndef AUBO_E5_HARDWARE__AUBO_E5_HARDWARE_HPP_
#define AUBO_E5_HARDWARE__AUBO_E5_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

// 随包携带（vendored）的 AUBO SDK 1.3.1 头文件 + moodycamel SPSC 队列。
#include "AuboRobotMetaType.h"
#include "readerwriterqueue.h"
#include "serviceinterface.h"

namespace aubo_e5_hardware
{

class AuboE5Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AuboE5Hardware)

  AuboE5Hardware() = default;
  ~AuboE5Hardware() override;

  using hardware_interface::SystemInterface::on_init;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  // 错误/收尾路径与正常 deactivate/cleanup 走同一份 teardown（停线程含
  // 板载停止原语 -> 退 TCP2CAN -> 注销回调/logout -> 清队列），保证任何
  // 异常序列下资源都被完整释放（此前缺这两个覆写，error 后线程/连接泄漏）。
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---------------------------------------------------------------- 常量
  static constexpr std::size_t kNumJoints = 6;
  // 权威关节顺序（方案 A.2）<=> SDK 数组下标 0..5。
  static const std::array<std::string, kNumJoints> kJointNames;
  // 电机 RPM -> 关节 rad/s（方案 A.6）：{2pi/60/121 x3, 2pi/60/101 x3}，
  // 蓝本 aubo_boot aubo_callback_monitor.cpp:29-32。
  static const std::array<double, kNumJoints> kV2R;

  // 行为基线常量（方案 A 节；刻意不可配置，保证与蓝本实测行为一致）。
  static constexpr int kLoginRetries = 5;              // A.5：登录重试次数
  static constexpr double kFirstFrameTimeoutSec = 5.0; // 启动流程第 3 步：首帧超时
  static constexpr int kRibFailFaultThreshold = 5;     // 健康看门狗：RIB 连查失败上限
  static constexpr int kSlowdownBatch1 = 2;            // A.7：rib_slowdown_1 <= rib < rib_slowdown_2 时每批 2 点
  static constexpr int kSlowdownBatch2 = 1;            // A.7：rib >= rib_slowdown_2 时每批 1 点
  static constexpr int kIdleDiagIntervalMs = 250;      // sendLoop：空闲时 RIB 轮询周期
  static constexpr int kFastSleepMs = 1;               // sendLoop 自适应睡眠（快转档）
  static constexpr std::size_t kBusyQueueWatermark = 40;  // sendLoop：队列深度 > 40 -> 快转
  static constexpr int kLowRibWatermark = 200;            // sendLoop：rib < 200 -> 快转
  static constexpr unsigned kSafetyIoDivisor = 5;      // 20ms * 5  = ~10Hz 安全 IO 轮询
  static constexpr unsigned kIoStateDivisor = 25;      // 20ms * 25 = 500ms 低频 IO 状态轮询
  static constexpr int kIoLoopPeriodMs = 20;           // IO 异步线程周期
  static constexpr int kStopRetryTimeoutMs = 1000;     // RobotMoveStop 重试超时，超时改用 robotMoveFastStop
  // teardown/on_deactivate 等 ioLoop 执行完停止原语的轮询参数：间隔 5ms、
  // 超时 1500ms（须大于 kStopRetryTimeoutMs=1000，给 FastStop 降级留出余量，
  // 超时仍未完成则放弃并 WARN —— 卡住 teardown 会连带卡死整个 CM 状态机）。
  static constexpr int kStopAckPollMs = 5;
  static constexpr int kStopAckTimeoutMs = 1500;
  static constexpr unsigned char kStartupCollisionClass = 6;  // A.8：startup 碰撞等级
  static constexpr int kStartupBoardMaxAcc = 1000;            // A.8：startup boardMaxAcc
  static constexpr double kResamplePeriodSec = 0.005;  // A.1：重采样 5ms 点距
  // 板载用户 DO 写按名称 "U_DO_XX"（S4；原 pin+32 地址写法因头文件/文档
  // 基地址歧义弃用，详见 handleIoCommands 注释）。
  static constexpr std::size_t kControllerDiCount = 30;  // 蓝本 digitalIn[30] 宽度
  // 安全 IO 引脚（A.4；aubo_boot aubo_hardware_interface.cpp:258-261）：
  // DI0/8 -> 急停，DI1/9 -> 防护停。
  static constexpr int kEstopPinA = 0;
  static constexpr int kEstopPinB = 8;
  static constexpr int kProtectivePinA = 1;
  static constexpr int kProtectivePinB = 9;

  // "无新命令"哨兵：IO 命令槽以 NaN 表示空闲，IO 线程据此跳过。
  static constexpr double kNoNewCmd = std::numeric_limits<double>::quiet_NaN();

  // trajectory_passthrough 传递状态机的各状态值（UR passthrough 契约，
  // 本地重写实现；语义详见 .cpp 中 write() 的注释块）。
  static constexpr double kTransferIdle = 0.0;           // 0 空闲
  static constexpr double kTransferAccepted = 1.0;       // 1 已受理，可发下一点
  static constexpr double kTransferPoint = 2.0;          // 2 控制器请求传递一个点
  static constexpr double kTransferDoneCmd = 3.0;        // 3 控制器宣告点发完
  static constexpr double kTransferInMotion = 4.0;       // 4 执行中（队列消费中）
  static constexpr double kTransferDone = 5.0;           // 5 执行完毕（硬件回写）
  static constexpr double kTransferNewTrajectory = 6.0;  // 6 新轨迹到达

  enum Health : int
  {
    kHealthOk = 0,
    kHealthEstop = 1,
    kHealthFault = 2,
  };

  // ------------------------------------------------------------------- 类型
  // 以下取值全部由 URDF hardware 参数注入（方案 B 表）。
  struct Params
  {
    std::string robot_ip;
    int server_port = 0;
    std::string sdk_username;
    std::string sdk_password;
    int send_period_ms = 0;
    int rib_target = 0;
    int rib_slowdown_1 = 0;
    int rib_slowdown_2 = 0;
    int batch_min = 0;
    int batch_max = 0;
    double ema_alpha = 0.0;
    std::array<double, 3> ema_boost_ms{};
    int stop_retry_ms = 0;
    int prefill_points = 0;
    int force_start_delay_ms = 0;
    bool speed_guard_enabled = false;
    std::array<double, kNumJoints> max_joint_velocity{};
    std::array<double, kNumJoints> max_joint_acceleration{};
    double point_spacing_s = 0.0;
    double same_point_eps = 0.0;
    double dedup_threshold = 0.0;
    int state_timeout_ms = 0;
    bool auto_power_on = false;
  };

  // SDK 推送帧的本地快照（jointStatusCallback 写、RT read() 读）。
  // 字段逐一对应 SDK JointStatus（AuboRobotMetaType.h:841-853）；电流/电压
  // 等保留 SDK 原始单位，换算集中在 read() 里做。
  struct JointStateSnapshot
  {
    std::array<double, kNumJoints> pos{};       // jointPosJ [rad]
    std::array<double, kNumJoints> vel_moto{};  // jointSpeedMoto [RPM]（电机侧）
    std::array<double, kNumJoints> tag_pos{};   // jointTagPosJ [rad]
    // jointTagSpeedMoto [RPM]（电机侧目标速度，float；换算同 vel_moto 走 kV2R）
    std::array<double, kNumJoints> tag_vel_moto{};
    // jointCurrentI：关节电流，SDK 原始单位（int，驱动器读数；SDK 未给量纲，
    // 不猜测换算系数，原样透传，消费侧自行标定）。
    std::array<double, kNumJoints> current_i{};
    std::array<double, kNumJoints> temp{};      // jointCurTemp [°C]
    std::array<int, kNumJoints> err{};          // jointErrorNum（uint16）
    std::chrono::steady_clock::time_point stamp{};
    bool valid = false;
  };

  // SDK 事件回调的本地拷贝（robotEventCallback 写 -> ioLoop 消费）。
  // 不用 RobotEventInfo 本体：eventType 在这里转成 int，避免队列元素类型
  // 依赖 SDK 头里枚举的具体大小端/对齐（也顺便抹平 C 枚举强转告警）。
  struct EventItem
  {
    int type = 0;         // RobotEventType 数值
    int code = 0;         // eventCode
    std::string content;  // eventContent（拷贝，见 robotEventCallback 注释）
  };

  // 控制器经 passthrough 接口传来的轨迹点（写线程 -> 发送线程）。
  struct Setpoint
  {
    std::array<double, kNumJoints> pos{};
    std::array<double, kNumJoints> vel{};
    std::array<double, kNumJoints> acc{};
    double time_from_start = 0.0;
    uint64_t generation = 0;  // 所属代际，用于丢弃已取消轨迹的残留点
  };

  // 重采样后的 5ms 规划点（发送线程内部流转，最终喂给 TCP2CAN）。
  struct PlanningState
  {
    std::array<double, kNumJoints> joint_pos{};
    std::array<double, kNumJoints> joint_vel{};
    std::array<double, kNumJoints> joint_acc{};
  };

  // ------------------------------------------------------------------ 辅助函数
  bool parseParams();
  bool validateInterfaceLayout() const;

  void sendLoop();
  void ioLoop();
  // 完整拆卸：停线程（含板载停止原语）-> 退 TCP2CAN -> 注销推送回调/双
  // logout -> 清队列 -> health 复位。on_error/on_shutdown 共用；全程幂等
  // （joinable/logged_in_/tcp2can_active_ 逐层防御），任意状态调用都安全。
  void teardown();
  // teardown 与 on_deactivate 的公共前段：停发送线程 -> 经 ioLoop 请求
  // 板载停止并等其完成 -> 停 IO 线程 -> 退出 TCP2CAN。
  void stopThreadsAndLeaveTcp2Can();
  void resampleSetpoints();
  void quinticInterpolate(
    const Setpoint & last, const Setpoint & curr, double t, double T, PlanningState & out) const;
  bool readRibLevel(int & rib);
  std::vector<aubo_robot_namespace::wayPoint_S> popGuardedBatch(std::size_t count);
  void handleIoCommands();
  void pollSafetyIo();
  void pollIoStates();

  static void jointStatusCallback(const aubo_robot_namespace::JointStatus * status, int size, void * arg);
  static void robotEventCallback(const aubo_robot_namespace::RobotEventInfo * info, void * arg);

  // 事件上报（ioLoop 侧）：drain 事件队列 -> 按严重级别打日志 + 更新
  // event_type_/event_code_ 原子量。eventToString/eventLogLevel 的取值
  // 覆盖 AuboRobotMetaType.h:895-1018 的 RobotEventType 全部枚举值。
  void drainEventQueue();
  static const char * eventToString(int type);
  // 返回 RCUTILS_LOG_SEVERITY 风格级别：0=DEBUG 1=INFO 2=WARN 3=ERROR。
  static int eventLogLevel(int type);

  // ------------------------------------------------------------------ 成员
  Params params_;

  // 两条 SDK 连接（A.5；aubo_boot aubo_hardware_interface.cpp:38-58）：
  // conn_control_ 只被发送线程使用，conn_status_ 只被 IO 异步线程使用，
  // 每个实例任意时刻只有一个线程在调用 —— SDK ServiceInterface 非线程安全，
  // 单实例并发调用会损坏内部状态，故按连接做线程归属隔离。
  // （SDK 的 ServiceInterface 是全局类，不在 aubo_robot_namespace 里。）
  ServiceInterface conn_control_;
  ServiceInterface conn_status_;
  bool tcp2can_active_ = false;
  // 双连接登录状态跟踪（只被 CM 生命周期线程读写，无需原子）：on_configure
  // 开头据此决定是否先走 logout 序列 —— error 恢复时 CM 可能不经 cleanup
  // 直接再次 configure，对同一 ServiceInterface 实例重复 login 的行为 SDK
  // 未定义，必须先登出。
  bool logged_in_ = false;

  // SDK 推送 -> 快照缓存（RealtimeThreadSafeBox：回调线程写、RT 线程读，
  // 无锁；写冲突时丢帧）。注意 Jazzy 的 try_get() 是 best-effort
  // try_lock：撞上写锁即返回 nullopt —— 调用方必须把 nullopt 当"瞬时
  // 撞锁"而非"无数据"（read() 经 last_snap_ 缓存回退处理，见其实现注释）。
  realtime_tools::RealtimeThreadSafeBox<JointStateSnapshot> state_box_;

  // RT read() 的本地快照缓存（只被 RT 线程读写，无需原子/box）：try_get
  // 撞锁失败时回退复用上一帧；新鲜度由 read() 里对 stamp 的
  // state_timeout_ms 超时检查保证（缓存帧超龄照样触发 stale FAULT）。
  JointStateSnapshot last_snap_{};
  bool have_last_snap_ = false;
  // try_get 取数失败计数（RT read() 写、on_error() 读；诊断计数器，
  // relaxed 足够）：让"靠缓存硬顶"的降级在故障汇总里可观测，而不是隐形。
  std::atomic<uint64_t> read_box_misses_{0};

  // 双队列轨迹流水线（4.3）：setpoint_queue_ 传控制器轨迹点（RT write()
  // 生产 -> 发送线程消费），send_queue_ 存重采样后的 5ms 点（发送线程内部
  // 自产自销）。
  // 清队语义：moodycamel 队列是 SPSC，不能在另一个线程"代为清空"（生产者
  // 可能正持有写指针，外部 try_dequeue 会与消费端竞争），所以其他线程只
  // 置 clear_motion_ 标志 + 递增 generation_ 代际，真正的清空动作由
  // 发送线程（消费端）自己完成；残留的旧代际点由代际比对丢弃。
  moodycamel::ReaderWriterQueue<Setpoint> setpoint_queue_{4096};
  moodycamel::ReaderWriterQueue<PlanningState> send_queue_{16384};

  // SDK 事件队列（robotEventCallback 生产 -> ioLoop 消费，SPSC）。有界 64：
  // 事件频率极低，64 条足够覆盖突发；满则丢弃并计入 dropped_events_，
  // 由 ioLoop 打 WARN 兜底（丢事件不能让回调阻塞 SDK 推送线程）。
  moodycamel::ReaderWriterQueue<EventItem> event_queue_{64};
  std::atomic<uint64_t> dropped_events_{0};
  // 最近一次事件的类型/码值（ioLoop 写、RT read() 读 -> aubo_io/event_type、
  // event_code 状态接口）。必须用 std::atomic<double>：ioLoop 与 RT 线程
  // 并发读写，plain double 是未定义行为（同 send_rate_pps_ 的处理）。
  // "无事件"哨兵是 -1 而非 0：0 与 RobotEvent_armCanbusError 枚举同值，
  // 用 0 当初值会让 io 控制器把"尚无事件"误显示成 CAN 总线错误（N9）。
  std::atomic<double> event_type_{-1.0};
  std::atomic<double> event_code_{0.0};

  // 跨线程协调（全部原子量）。
  std::atomic<uint64_t> generation_{0};
  std::atomic<bool> clear_motion_{false};
  std::atomic<bool> stop_motion_requested_{false};
  std::atomic<int> health_{kHealthOk};
  std::atomic<bool> emergency_stopped_{false};
  std::atomic<bool> protective_stopped_{false};
  std::atomic<bool> power_on_{false};
  std::atomic<bool> collision_{false};
  std::atomic<int> rib_level_{0};
  std::atomic<uint64_t> setpoints_resampled_{0};
  // 发送线程指标块每秒刷新一次的瞬时吞吐 [pts/s]，read() 读给
  // aubo_io/send_rate_pps 状态接口（原来是只打日志，见 sendLoop 指标块）。
  std::atomic<double> send_rate_pps_{0.0};
  std::atomic<bool> send_running_{false};
  std::atomic<bool> io_running_{false};
  std::thread send_thread_;
  std::thread io_thread_;

  // 发送线程的局部状态声明在 sendLoop() 内；速度守卫的滤波位置放这里，
  // 因为 on_activate() 需要把它复位到当前实际位置。
  std::array<double, kNumJoints> guard_joint_filter_{};

  // 传递状态机状态（仅 write()/RT 线程读写）。drain_wait_active_ = 正在
  // 等待发送队列排空，排空后向控制器回写 DONE(5)。
  bool drain_wait_active_ = false;
  bool abort_latched_ = false;
  bool trajectory_active_ = false;
  uint64_t points_received_ = 0;
  double expected_points_ = 0.0;
  // N8：read() 推送超时的原因标志（RT read() 写 -> ioLoop 读后清），
  // 供 ioLoop 的 health 边沿日志分辨"推送超时"与其他 FAULT 来源。
  std::atomic<bool> push_stale_fault_{false};
  // N8 补充：read() 返回 ERROR 的分支原因与失败时快照年龄（RT 线程只写
  // 原子量，日志由 on_error() 在非 RT 上下文统一输出）。背景：ioLoop 的
  // health 边沿上报在故障窗口内可能正阻塞于 conn_status_ 的同步 SDK 调用，
  // 返回后直接进 teardown 而错过上报；on_error 是 CM 错误路径的必经点，
  // 不会漏。取值：0=无错误, 1=快照缺失/无效, 2=推送超时(stale),
  // 3=health 非 OK（事件/发送面 FAULT）。
  std::atomic<int> read_error_reason_{0};
  std::atomic<int64_t> read_error_snapshot_age_ms_{-1};

  // ---------------------------------------------------------------- 接口存储
  // 关节接口。
  std::array<double, kNumJoints> hw_position_commands_{};
  std::array<double, kNumJoints> hw_position_states_{};
  std::array<double, kNumJoints> hw_velocity_states_{};

  // trajectory_passthrough GPIO 命令接口（硬件把状态机反馈回写进
  // transfer_state / abort，控制器据此读推进）。
  std::array<double, kNumJoints> traj_setpoint_positions_{};
  std::array<double, kNumJoints> traj_setpoint_velocities_{};
  std::array<double, kNumJoints> traj_setpoint_accelerations_{};
  double traj_transfer_state_ = kTransferIdle;
  double traj_time_from_start_ = 0.0;
  double traj_abort_ = 0.0;
  double traj_trajectory_size_ = 0.0;

  // speed_scaling GPIO 状态（恒定 1.0；本驱动不做执行期调速缩放）。
  double speed_scaling_factor_state_ = 1.0;

  // aubo_io GPIO。
  static constexpr std::size_t kNumBoardDIO = 16;
  static constexpr std::size_t kNumBoardAIO = 4;
  static constexpr std::size_t kNumToolIO = 2;
  std::array<double, kNumBoardDIO> io_do_cmd_{};
  std::array<double, kNumBoardAIO> io_ao_cmd_{};
  std::array<double, kNumToolIO> io_tool_do_cmd_{};
  std::array<double, kNumToolIO> io_tool_ao_cmd_{};
  double io_set_async_success_ = kNoNewCmd;
  std::array<double, kNumBoardDIO> io_di_state_{};
  std::array<double, kNumBoardAIO> io_ai_state_{};
  std::array<double, kNumToolIO> io_tool_di_state_{};
  std::array<double, kNumToolIO> io_tool_ai_state_{};
  double io_estop_state_ = 0.0;
  double io_protective_stop_state_ = 0.0;
  double io_power_on_state_ = 0.0;
  double io_collision_state_ = 0.0;
  double io_in_motion_state_ = 0.0;
  double io_rib_level_state_ = 0.0;
  std::array<double, kNumJoints> io_joint_error_state_{};
  // 反馈增强（SDK JointStatus 采全）：目标位置/速度、电流、温度。
  // tag_vel 在 read() 里由 tag_vel_moto * kV2R 换算（与关节速度同一路径）。
  std::array<double, kNumJoints> io_tag_pos_state_{};
  std::array<double, kNumJoints> io_tag_vel_state_{};
  std::array<double, kNumJoints> io_joint_current_state_{};
  std::array<double, kNumJoints> io_joint_temp_state_{};
  // 发送流水线可观测性：重采样点队列深度 [点]、瞬时吞吐 [pts/s]。
  double io_send_queue_points_state_ = 0.0;
  double io_send_rate_pps_state_ = 0.0;
  // 事件/健康上报（SDK RobotEventInfo + health_）：由 read() 从
  // event_type_/event_code_/health_ 原子量拷贝（跨线程交接全走原子量，
  // 状态接口存储本身只被 RT read() 写，与 estop/power_on 同一模式）。
  double io_event_type_state_ = 0.0;
  double io_event_code_state_ = 0.0;
  double io_health_state_ = 0.0;
};

}  // namespace aubo_e5_hardware

#endif  // AUBO_E5_HARDWARE__AUBO_E5_HARDWARE_HPP_
