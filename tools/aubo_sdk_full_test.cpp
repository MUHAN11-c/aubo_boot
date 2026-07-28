// AUBO SDK 2.5.3 全面测试程序（全部现场实测，不引用历史结论）。
//
// 运动硬约束：只允许 home 与 camera_pose 两个关节目标点之间的 MoveJ
// （robotServiceJointMove），目标值硬编码为常量，不提供任意位姿输入接口。
//
// 位姿来源：src/aubo_e5_moveit_config/config/aubo_e5.srdf 的 group_state
//   home / camera_pose（group manipulator）。SRDF 按字母序列出关节，
//   这里已转换为 SDK 关节顺序：shoulder, upperArm, foreArm, wrist1, wrist2, wrist3。
//
// 阶段：
//   S0 ABI 自检        S1 连接/登录/登出/重登录
//   S2 只读状态面      S3 轮询基准（10Hz x 300 + 突发上限）
//   S4 实时推送        S5 TCP2CAN 写通道（无运动，当前位姿原样回写）
//   S6 运动（仅 home <-> camera_pose，低速）
//   S7 推送+TCP2CAN 并发（破坏性，放最后，结束后自动重连验证恢复）
//   S8 清理与汇总
//
// Build:
//   SDK=src/aubo_e5_hardware/vendor/aubo_sdk_2_5_3
//   g++ -std=c++17 -I $SDK/include tools/aubo_sdk_full_test.cpp
//     -o build/diagnostics/full_test -L $SDK/lib -laubo_sdk -Wl,-rpath,$PWD/$SDK/lib
//
// Usage:
//   full_test [--host IP] [--port N] [--skip-motion] [--rounds N] [--skip-coexist]

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace {

namespace an = aubo_robot_namespace;

// ---------------------------------------------------------------------------
// 允许的运动目标（仅此两个，硬编码）
// ---------------------------------------------------------------------------
constexpr int kDof = an::ARM_DOF;
// SRDF group_state "home"，SDK 关节序 shoulder, upperArm, foreArm, wrist1, wrist2, wrist3
const double kHomePose[kDof] = {0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0};
// SRDF group_state "camera_pose"，同上关节序
const double kCameraPose[kDof] = {
  -0.27411168813705444, 0.4963911175727844, 1.7700852155685425,
  -0.2978658676147461, 1.571584939956665, -0.2750104069709778};

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop.store(true); }

// ---------------------------------------------------------------------------
// 结果记录
// ---------------------------------------------------------------------------
struct TestResult {
  std::string stage;
  std::string name;
  bool passed;
  std::string detail;
};

std::vector<TestResult> g_results;

void record(const char *stage, const char *name, bool passed,
            const std::string &detail)
{
  g_results.push_back({stage, name, passed, detail});
  std::cout << "  [" << (passed ? "PASS" : "FAIL") << "] " << name;
  if (!detail.empty()) {
    std::cout << "  (" << detail << ")";
  }
  std::cout << '\n';
}

template<typename T>
std::string num(T v)
{
  std::ostringstream oss;
  oss << v;
  return oss.str();
}

// ---------------------------------------------------------------------------
// 延迟统计
// ---------------------------------------------------------------------------
struct Stats {
  double min_ms = std::numeric_limits<double>::infinity();
  double max_ms = 0.0;
  double total_ms = 0.0;
  std::size_t count = 0;
  std::vector<double> samples;

  void add(double ms)
  {
    min_ms = std::min(min_ms, ms);
    max_ms = std::max(max_ms, ms);
    total_ms += ms;
    ++count;
    samples.push_back(ms);
  }

  double avg() const { return count ? total_ms / count : 0.0; }

  double percentile(double p)
  {
    if (samples.empty()) return 0.0;
    std::sort(samples.begin(), samples.end());
    return samples[std::min(
      samples.size() - 1,
      static_cast<std::size_t>(p / 100.0 * samples.size()))];
  }

  std::string str()
  {
    if (count == 0) return "no samples";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "n=" << count << " min/avg/p99/max(ms)=" << min_ms << '/' << avg()
        << '/' << percentile(99.0) << '/' << max_ms;
    return oss.str();
  }
};

const char *stateName(an::RobotState state)
{
  switch (state) {
    case an::RobotStopped: return "stopped";
    case an::RobotRunning: return "running";
    case an::RobotPaused: return "paused";
    case an::RobotResumed: return "resumed";
  }
  return "unknown";
}

double maxJointError(const double a[kDof], const double b[kDof])
{
  double err = 0.0;
  for (int i = 0; i < kDof; ++i) {
    err = std::max(err, std::fabs(a[i] - b[i]));
  }
  return err;
}

bool readJoints(ServiceInterface &svc, double out[kDof])
{
  an::JointParam param{};
  if (svc.robotServiceGetJointAngleInfo(param) != an::InterfaceCallSuccCode) {
    return false;
  }
  for (int i = 0; i < kDof; ++i) {
    out[i] = param.jointPos[i];
  }
  return true;
}

// 各阶段开始前确保连接存活；掉线则重新登录
bool ensureConnected(ServiceInterface &svc, const std::string &host, int port)
{
  bool connected = false;
  svc.robotServiceGetConnectStatus(connected);
  if (connected) return true;
  std::cout << "  [reconnect] connection lost, re-login...\n";
  svc.robotServiceLogout();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const int rc = svc.robotServiceLogin(host.c_str(), port, "aubo", "123456");
  std::cout << "  [reconnect] rc=" << rc << '\n';
  return rc == an::InterfaceCallSuccCode;
}

// ---------------------------------------------------------------------------
// S4 推送回调（回调内不做任何 SDK 调用）
// ---------------------------------------------------------------------------
std::atomic<std::size_t> g_push_count{0};
std::mutex g_push_mutex;
std::vector<std::chrono::steady_clock::time_point> g_push_stamps;

void onJointStatusPush(const an::JointStatus *status, int size, void *arg)
{
  (void)status; (void)size; (void)arg;
  const std::size_t index = g_push_count.fetch_add(1, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(g_push_mutex);
  if (index < g_push_stamps.size()) {
    g_push_stamps[index] = std::chrono::steady_clock::now();
  }
}

// ---------------------------------------------------------------------------
// TCP2CAN 流式插补运动：当前位姿 -> target（仅限 home/camera_pose 两端点）
// 轨迹为两端点间的关节空间余弦插值，不使用任何其他位姿。
// ---------------------------------------------------------------------------
struct StreamResult {
  double err = -1.0;      // 到位最大关节误差 rad
  double total_s = 0.0;   // 发送+排空总耗时
  int points = 0;
  int failures = 0;
  bool aborted = false;
  bool saw_running = false;
};

StreamResult streamMoveTcp2Can(ServiceInterface &svc, const double *target,
                               bool observe_state)
{
  StreamResult res;
  double start[kDof];
  if (!readJoints(svc, start)) {
    res.aborted = true;
    return res;
  }

  double delta[kDof], max_delta = 0.0;
  for (int i = 0; i < kDof; ++i) {
    delta[i] = target[i] - start[i];
    max_delta = std::max(max_delta, std::fabs(delta[i]));
  }
  if (max_delta < 1e-6) {
    res.err = 0.0;
    double actual[kDof];
    if (readJoints(svc, actual)) res.err = maxJointError(actual, target);
    return res;
  }

  constexpr double kPointDt = 0.010;   // 路径点间隔 10ms
  constexpr double kMaxSpeed = 0.25;   // 关节平均速度上限 rad/s
  constexpr int kRibHigh = 120;
  constexpr int kBatchMax = 8;
  const double duration = std::max(2.0, max_delta / kMaxSpeed);
  const int n = static_cast<int>(duration / kPointDt) + 1;
  std::vector<std::array<double, kDof>> path(n);
  for (int k = 0; k < n; ++k) {
    const double tau = static_cast<double>(k) / (n - 1);
    const double s = 0.5 * (1.0 - std::cos(M_PI * tau));  // 余弦缓动
    for (int i = 0; i < kDof; ++i) {
      path[k][i] = start[i] + s * delta[i];
    }
  }

  const auto t0 = std::chrono::steady_clock::now();
  std::size_t idx = 0;
  int last_rib = 0, cycle_count = 0;
  an::RobotDiagnosis diag{};
  while (idx < path.size() && res.failures < 10 && !g_stop.load()) {
    const auto cycle = std::chrono::steady_clock::now();
    if (svc.robotServiceGetRobotDiagnosisInfo(diag) ==
        an::InterfaceCallSuccCode) {
      last_rib = diag.macTargetPosDataSize;
    } else {
      ++res.failures;
    }
    if (observe_state && !res.saw_running && (++cycle_count % 20 == 0)) {
      an::RobotState st{};
      if (svc.robotServiceGetRobotCurrentState(st) ==
            an::InterfaceCallSuccCode &&
          st == an::RobotRunning) {
        res.saw_running = true;
      }
    }
    if (last_rib < kRibHigh) {
      const int batch = std::min(
        {kBatchMax, kRibHigh - last_rib,
         static_cast<int>(path.size() - idx)});
      std::vector<an::wayPoint_S> pts(batch, an::wayPoint_S{});
      for (int b = 0; b < batch; ++b) {
        for (int i = 0; i < kDof; ++i) {
          pts[b].jointpos[i] = path[idx + b][i];
        }
      }
      if (svc.robotServiceSetRobotPosData2Canbus(pts) ==
          an::InterfaceCallSuccCode) {
        idx += batch;
        last_rib += batch;
      } else {
        ++res.failures;
      }
    }
    std::this_thread::sleep_until(cycle + std::chrono::milliseconds(5));
  }
  res.points = static_cast<int>(idx);
  if (res.failures >= 10 || g_stop.load()) {
    svc.rootServiceRobotMoveControl(an::RobotMoveStop);
    res.aborted = true;
    return res;
  }

  // 等 RIB 排空（机械臂走完缓存点）
  bool drained = false;
  const auto d0 = std::chrono::steady_clock::now();
  while (!g_stop.load() &&
         std::chrono::steady_clock::now() - d0 < std::chrono::seconds(20)) {
    if (svc.robotServiceGetRobotDiagnosisInfo(diag) ==
          an::InterfaceCallSuccCode &&
        diag.macTargetPosDataSize == 0) {
      drained = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  res.total_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t0).count();
  if (!drained) {
    res.aborted = true;
    return res;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  double actual[kDof];
  if (readJoints(svc, actual)) res.err = maxJointError(actual, target);
  return res;
}

}  // namespace

int main(int argc, char **argv)
{
  std::string host = "169.254.10.98";
  int port = 8899;
  bool skip_motion = false;
  bool skip_coexist = false;
  int rounds = 2;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--host" && i + 1 < argc) {
      host = argv[++i];
    } else if (arg == "--port" && i + 1 < argc) {
      port = std::atoi(argv[++i]);
    } else if (arg == "--skip-motion") {
      skip_motion = true;
    } else if (arg == "--skip-coexist") {
      skip_coexist = true;
    } else if (arg == "--rounds" && i + 1 < argc) {
      rounds = std::max(1, std::atoi(argv[++i]));
    } else {
      std::cerr << "unknown argument: " << arg << '\n';
      return 64;
    }
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::cout << "=== AUBO SDK 2.5.3 全面测试（现场实测）===\n"
            << "target=" << host << ':' << port
            << "  skip_motion=" << std::boolalpha << skip_motion
            << "  rounds=" << rounds
            << "  skip_coexist=" << skip_coexist << '\n';

  ServiceInterface svc;

  // -------------------------------------------------------------------------
  std::cout << "\n--- S0 ABI 自检 ---\n";
  {
    std::ostringstream oss;
    oss << "sizeof(JointStatus)=" << sizeof(an::JointStatus)
        << " sizeof(wayPoint_S)=" << sizeof(an::wayPoint_S)
        << " sizeof(RobotDiagnosis)=" << sizeof(an::RobotDiagnosis)
        << " sizeof(ServiceInterface)=" << sizeof(ServiceInterface);
    std::cout << "  " << oss.str() << '\n';
    // 2.5.3 配套头文件的 JointStatus 为 36 字节（旧头文件 34 字节，混用即 ABI 错误）
    record("S0", "ABI sizeof(JointStatus)==36", sizeof(an::JointStatus) == 36,
           oss.str());
  }

  // -------------------------------------------------------------------------
  std::cout << "\n--- S1 连接与登录 ---\n";
  {
    const int login = svc.robotServiceLogin(host.c_str(), port, "aubo", "123456");
    record("S1", "login", login == an::InterfaceCallSuccCode,
           "rc=" + num(login));
    if (login != an::InterfaceCallSuccCode) {
      std::cerr << "login failed, abort\n";
      return 10;
    }

    bool connected = false;
    svc.robotServiceGetConnectStatus(connected);
    record("S1", "connect_status", connected, connected ? "connected" : "not connected");

    bool real = false;
    const int rc_real = svc.robotServiceGetIsRealRobotExist(real);
    record("S1", "real_robot_exist", rc_real == an::InterfaceCallSuccCode && real,
           "rc=" + num(rc_real) + " value=" + num(real));

    bool mac = false;
    const int rc_mac = svc.robotServiceGetMacCommunicationStatus(mac);
    record("S1", "mac_communication", rc_mac == an::InterfaceCallSuccCode,
           "rc=" + num(rc_mac) + " value=" + num(mac));

    an::RobotWorkMode mode{};
    const int rc_mode = svc.robotServiceGetRobotWorkMode(mode);
    record("S1", "work_mode", rc_mode == an::InterfaceCallSuccCode,
           std::string("rc=") + num(rc_mode) + " mode=" +
             (mode == an::RobotModeReal ? "real" : "simulator"));

    const int logout = svc.robotServiceLogout();
    record("S1", "logout", logout == an::InterfaceCallSuccCode, "rc=" + num(logout));

    const int relogin = svc.robotServiceLogin(host.c_str(), port, "aubo", "123456");
    record("S1", "re-login", relogin == an::InterfaceCallSuccCode,
           "rc=" + num(relogin));
    if (relogin != an::InterfaceCallSuccCode) {
      std::cerr << "re-login failed, abort\n";
      return 11;
    }
  }

  // -------------------------------------------------------------------------
  std::cout << "\n--- S2 只读状态面 ---\n";
  ensureConnected(svc, host, port);
  double current_joints[kDof] = {0};
  {
    an::RobotState state{};
    const int rc = svc.robotServiceGetRobotCurrentState(state);
    record("S2", "robot_current_state", rc == an::InterfaceCallSuccCode,
           "rc=" + num(rc) + " state=" + stateName(state));

    an::JointStatus joints[kDof]{};
    const int rc_js = svc.robotServiceGetRobotJointStatus(joints, kDof);
    std::ostringstream js;
    for (int i = 0; i < kDof; ++i) {
      js << "J" << i + 1 << "[pos=" << std::fixed << std::setprecision(4)
         << joints[i].jointPosJ << " cur=" << joints[i].jointCurrentI
         << " temp=" << joints[i].jointCurTemp
         << " err=" << joints[i].jointErrorNum << "] ";
    }
    record("S2", "joint_status x6", rc_js == an::InterfaceCallSuccCode, js.str());

    an::wayPoint_S waypoint{};
    const int rc_wp = svc.robotServiceGetCurrentWaypointInfo(waypoint);
    an::JointParam jp{};
    const int rc_jp = svc.robotServiceGetJointAngleInfo(jp);
    bool consistent = false;
    double wp_jp_err = -1.0;
    if (rc_wp == an::InterfaceCallSuccCode && rc_jp == an::InterfaceCallSuccCode) {
      wp_jp_err = maxJointError(waypoint.jointpos, jp.jointPos);
      consistent = wp_jp_err < 1e-4;
      for (int i = 0; i < kDof; ++i) current_joints[i] = jp.jointPos[i];
    }
    record("S2", "waypoint_vs_jointangle_consistency",
           consistent, "max_diff_rad=" + num(wp_jp_err));

    an::RobotDiagnosis diag{};
    const int rc_diag = svc.robotServiceGetRobotDiagnosisInfo(diag);
    std::ostringstream ds;
    ds << "power=" << diag.armPowerStatus
       << " brake=" << diag.brakeStuats
       << " soft_estop=" << diag.softEmergency
       << " remote_estop=" << diag.remoteEmergency
       << " collision=" << diag.robotCollision
       << " canbus_mask=" << static_cast<int>(diag.armCanbusStatus)
       << " end_speed=" << diag.robotEndSpeed;
    record("S2", "diagnosis_info", rc_diag == an::InterfaceCallSuccCode, ds.str());

    // 控制柜 IO（按类型批量读）
    std::vector<an::RobotIoType> io_types = {
      an::RobotBoardUserDI, an::RobotBoardUserDO,
      an::RobotBoardUserAI, an::RobotBoardUserAO};
    std::vector<an::RobotIoDesc> io_status;
    const int rc_io = svc.robotServiceGetBoardIOStatus(io_types, io_status);
    record("S2", "board_io_read", rc_io == an::InterfaceCallSuccCode,
           "rc=" + num(rc_io) + " entries=" + num(io_status.size()));
    for (const auto &io : io_status) {
      std::cout << "    IO type=" << io.ioType << " addr=" << io.ioAddr
                << " value=" << io.ioValue << '\n';
    }

    // 工具端 IO
    std::vector<an::RobotIoDesc> tool_io;
    const int rc_tool = svc.robotServiceGetAllToolDigitalIOStatus(tool_io);
    record("S2", "tool_io_read", rc_tool == an::InterfaceCallSuccCode,
           "rc=" + num(rc_tool) + " entries=" + num(tool_io.size()));
    for (const auto &io : tool_io) {
      std::cout << "    TOOL_IO addr=" << io.ioAddr << " value=" << io.ioValue << '\n';
    }

    // 运动学回环：当前关节角 -> FK -> IK -> 关节角
    if (rc_wp == an::InterfaceCallSuccCode) {
      an::wayPoint_S fk_out{};
      const int rc_fk = svc.robotServiceRobotFk(current_joints, kDof, fk_out);
      an::wayPoint_S ik_out{};
      const int rc_ik = svc.robotServiceRobotIk(
        current_joints, fk_out.cartPos.position, fk_out.orientation, ik_out);
      double fk_ik_err = -1.0;
      bool fkik_ok = false;
      if (rc_fk == an::InterfaceCallSuccCode && rc_ik == an::InterfaceCallSuccCode) {
        fk_ik_err = maxJointError(ik_out.jointpos, current_joints);
        fkik_ok = fk_ik_err < 1e-3;
      }
      record("S2", "fk_ik_roundtrip", fkik_ok,
             "rc_fk=" + num(rc_fk) + " rc_ik=" + num(rc_ik) +
               " max_err_rad=" + num(fk_ik_err));

      // 四元数 <-> RPY 回环（允许 q 与 -q 等价）
      an::Rpy rpy{};
      const int rc_q2r = svc.quaternionToRPY(waypoint.orientation, rpy);
      an::Ori ori2{};
      const int rc_r2q = svc.RPYToQuaternion(rpy, ori2);
      bool quat_ok = false;
      double quat_err = -1.0;
      if (rc_q2r == an::InterfaceCallSuccCode && rc_r2q == an::InterfaceCallSuccCode) {
        const an::Ori &q = waypoint.orientation;
        const double d1 = std::fabs(ori2.w - q.w) + std::fabs(ori2.x - q.x) +
                          std::fabs(ori2.y - q.y) + std::fabs(ori2.z - q.z);
        const double d2 = std::fabs(ori2.w + q.w) + std::fabs(ori2.x + q.x) +
                          std::fabs(ori2.y + q.y) + std::fabs(ori2.z + q.z);
        quat_err = std::min(d1, d2);
        quat_ok = quat_err < 1e-6;
      }
      record("S2", "quaternion_rpy_roundtrip", quat_ok,
             "rc_q2r=" + num(rc_q2r) + " rc_r2q=" + num(rc_r2q) +
               " err=" + num(quat_err));
    }

    const std::string desc0 =
      svc.getErrDescByCode(static_cast<an::RobotErrorCode>(0));
    record("S2", "getErrDescByCode(0)", !desc0.empty(),
           "desc=\"" + desc0 + "\"");
  }

  // -------------------------------------------------------------------------
  std::cout << "\n--- S3 轮询基准 ---\n";
  ensureConnected(svc, host, port);
  {
    // 10Hz x 300 次
    Stats poll;
    int failures = 0;
    const int samples = 300;
    const auto period = std::chrono::milliseconds(100);
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < samples && !g_stop.load(); ++i) {
      an::JointStatus joints[kDof]{};
      const auto t0 = std::chrono::steady_clock::now();
      const int rc = svc.robotServiceGetRobotJointStatus(joints, kDof);
      poll.add(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count());
      if (rc != an::InterfaceCallSuccCode) ++failures;
      std::this_thread::sleep_until(start + (i + 1) * period);
    }
    record("S3", "poll_10hz_x300", failures == 0 && poll.count >= samples / 2,
           poll.str() + " failures=" + num(failures));

    // 突发：200 次背靠背，测有效吞吐
    const int burst = 200;
    int burst_failures = 0;
    const auto b0 = std::chrono::steady_clock::now();
    for (int i = 0; i < burst && !g_stop.load(); ++i) {
      an::JointStatus joints[kDof]{};
      if (svc.robotServiceGetRobotJointStatus(joints, kDof) !=
          an::InterfaceCallSuccCode) {
        ++burst_failures;
      }
    }
    const double burst_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - b0).count();
    const double burst_hz = burst_s > 0 ? burst / burst_s : 0.0;
    record("S3", "burst_200_effective_rate", burst_failures == 0,
           "effective_hz=" + num(burst_hz) + " failures=" + num(burst_failures));
  }

  // -------------------------------------------------------------------------
  std::cout << "\n--- S4 实时推送（10s）---\n";
  {
    // 强制换新连接，隔离前序阶段（突发轮询）对推送通道的影响
    svc.robotServiceLogout();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const int fresh = svc.robotServiceLogin(host.c_str(), port, "aubo", "123456");
    std::cout << "  [fresh login] rc=" << fresh << '\n';
  }
  {
    g_push_stamps.assign(20000, {});
    g_push_count.store(0);
    const int push_on = svc.robotServiceSetRealTimeJointStatusPush(true);
    const int reg = svc.robotServiceRegisterRealTimeJointStatusCallback(
      &onJointStatusPush, nullptr);
    if (push_on != 0 || reg != 0) {
      record("S4", "push_enable", false,
             "push_on=" + num(push_on) + " reg=" + num(reg));
    } else {
      std::this_thread::sleep_for(std::chrono::seconds(10));
      bool alive_after_10s = false;
      svc.robotServiceGetConnectStatus(alive_after_10s);
      const int push_off = svc.robotServiceSetRealTimeJointStatusPush(false);
      svc.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);

      const std::size_t count =
        std::min(g_push_count.load(), g_push_stamps.size());
      double min_iv = std::numeric_limits<double>::infinity();
      double max_iv = 0.0, total_iv = 0.0;
      for (std::size_t i = 1; i < count; ++i) {
        const double iv = std::chrono::duration<double, std::milli>(
          g_push_stamps[i] - g_push_stamps[i - 1]).count();
        min_iv = std::min(min_iv, iv);
        max_iv = std::max(max_iv, iv);
        total_iv += iv;
      }
      // 关闭后再等 1s，确认推送停止
      const std::size_t frozen = g_push_count.load();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const bool stopped = g_push_count.load() == frozen;

      std::ostringstream oss;
      oss << "count=" << count << " rate_hz=" << std::fixed
          << std::setprecision(1) << (count / 10.0)
          << " interval_ms[min/avg/max]=" << std::setprecision(2)
          << (count > 1 ? min_iv : 0.0) << '/'
          << (count > 1 ? total_iv / (count - 1) : 0.0) << '/'
          << (count > 1 ? max_iv : 0.0)
          << " push_off_rc=" << push_off << " stopped_after_off=" << stopped
          << " alive_after_10s=" << alive_after_10s;
      record("S4", "push_10s",
             count > 100 && push_off == 0 && stopped && alive_after_10s,
             oss.str());
    }
  }

  // -------------------------------------------------------------------------
  std::cout << "\n--- S5 TCP2CAN 写通道（无运动，当前位姿原样回写）---\n";
  ensureConnected(svc, host, port);
  {
    double setpoint[kDof];
    if (!readJoints(svc, setpoint)) {
      record("S5", "read_setpoint", false, "getJointAngleInfo failed");
    } else {
      int rc = svc.robotServiceEnterTcp2CanbusMode();
      if (rc == an::ErrCode_ResponseReturnError) {
        svc.robotServiceLeaveTcp2CanbusMode();
        rc = svc.robotServiceEnterTcp2CanbusMode();
      }
      record("S5", "enter_tcp2can", rc == an::InterfaceCallSuccCode,
             "rc=" + num(rc));

      if (rc == an::InterfaceCallSuccCode) {
        auto makePoints = [&](int n) {
          std::vector<an::wayPoint_S> pts(n, an::wayPoint_S{});
          for (auto &p : pts) {
            for (int i = 0; i < kDof; ++i) p.jointpos[i] = setpoint[i];
          }
          return pts;
        };

        // 批量上限探测：4 / 8 / 16 / 32
        for (const int batch : {4, 8, 16, 32}) {
          auto pts = makePoints(batch);
          const int rc_b = svc.robotServiceSetRobotPosData2Canbus(pts);
          record("S5", ("batch_" + num(batch)).c_str(), true,
                 "rc=" + num(rc_b));  // 实测记录，不预设期望值
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        // 等待 RIB 排空
        an::RobotDiagnosis diag{};
        for (int i = 0; i < 100; ++i) {
          if (svc.robotServiceGetRobotDiagnosisInfo(diag) ==
                an::InterfaceCallSuccCode &&
              diag.macTargetPosDataSize == 0) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // 5ms 周期、RIB 水位门控、batch<=8，持续 20s
        constexpr int kRibHigh = 120;
        constexpr int kBatchMax = 8;
        Stats write_stats, diag_stats;
        int write_failures = 0, diag_failures = 0, cycles = 0;
        long points_written = 0;
        int last_rib = 0, rib_min = std::numeric_limits<int>::max(), rib_max = 0;
        const auto t_end = std::chrono::steady_clock::now() +
                           std::chrono::seconds(20);
        while (!g_stop.load() && std::chrono::steady_clock::now() < t_end) {
          const auto cycle_start = std::chrono::steady_clock::now();
          const auto d0 = std::chrono::steady_clock::now();
          const int rc_d = svc.robotServiceGetRobotDiagnosisInfo(diag);
          diag_stats.add(std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - d0).count());
          if (rc_d != an::InterfaceCallSuccCode) {
            ++diag_failures;
          } else {
            last_rib = diag.macTargetPosDataSize;
            rib_min = std::min(rib_min, last_rib);
            rib_max = std::max(rib_max, last_rib);
          }
          if (last_rib < kRibHigh) {
            const int batch = std::min(kBatchMax, kRibHigh - last_rib);
            auto pts = makePoints(batch);
            const auto w0 = std::chrono::steady_clock::now();
            const int rc_w = svc.robotServiceSetRobotPosData2Canbus(pts);
            write_stats.add(std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - w0).count());
            if (rc_w != an::InterfaceCallSuccCode) {
              ++write_failures;
            } else {
              points_written += batch;
              last_rib += batch;
            }
          }
          ++cycles;
          std::this_thread::sleep_until(
            cycle_start + std::chrono::milliseconds(5));
        }

        // 排空观测
        double drain_s = -1.0;
        int drain_start_rib = 0;
        if (svc.robotServiceGetRobotDiagnosisInfo(diag) ==
            an::InterfaceCallSuccCode) {
          drain_start_rib = diag.macTargetPosDataSize;
          const auto d_start = std::chrono::steady_clock::now();
          while (!g_stop.load() &&
                 std::chrono::steady_clock::now() - d_start <
                   std::chrono::seconds(15)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (svc.robotServiceGetRobotDiagnosisInfo(diag) !=
                an::InterfaceCallSuccCode) {
              continue;
            }
            if (diag.macTargetPosDataSize == 0) {
              drain_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - d_start).count();
              break;
            }
          }
        }

        std::ostringstream oss;
        oss << "cycles=" << cycles << " points_written=" << points_written
            << " write_failures=" << write_failures
            << " diag_failures=" << diag_failures
            << " write[" << write_stats.str() << "]"
            << " rib[min/max]=" << (rib_min == std::numeric_limits<int>::max() ? 0 : rib_min)
            << '/' << rib_max
            << " drain_start_rib=" << drain_start_rib
            << " drain_s=" << drain_s;
        record("S5", "tcp2can_stream_20s",
               write_failures == 0 && diag_failures == 0 && drain_s >= 0.0,
               oss.str());

        svc.rootServiceRobotMoveControl(an::RobotMoveStop);
        const int leave = svc.robotServiceLeaveTcp2CanbusMode();
        record("S5", "leave_tcp2can", leave == an::InterfaceCallSuccCode,
               "rc=" + num(leave));
      }
    }
  }

  // -------------------------------------------------------------------------
  if (!skip_motion) {
    std::cout << "\n--- S6 运动测试（仅 home <-> camera_pose，低速）---\n";
    ensureConnected(svc, host, port);

    // 6.1 高层 MoveJ 接口实测（本固件上被拒绝则记录实测 rc）
    {
      const int rc_init = svc.robotServiceInitGlobalMoveProfile();
      an::JointVelcAccParam max_vel{}, max_acc{};
      for (int i = 0; i < kDof; ++i) {
        max_vel.jointPara[i] = 0.3;   // rad/s
        max_acc.jointPara[i] = 1.0;   // rad/s^2
      }
      const int rc_vel = svc.robotServiceSetGlobalMoveJointMaxVelc(max_vel);
      const int rc_acc = svc.robotServiceSetGlobalMoveJointMaxAcc(max_acc);
      double target[kDof];
      for (int i = 0; i < kDof; ++i) target[i] = kCameraPose[i];
      const int rc_mv = svc.robotServiceJointMove(target, true);
      std::ostringstream oss;
      oss << "rc_init=" << rc_init << " rc_vel=" << rc_vel
          << " rc_acc=" << rc_acc << " jointMove_rc=" << rc_mv
          << " (10023=ErrCode_ResponseReturnError 服务器拒绝)";
      record("S6", "jointMove_interface", rc_mv == an::InterfaceCallSuccCode,
             oss.str());
    }

    // 6.2 TCP2CAN 流式插补运动（端点严格为 home/camera_pose）
    int rc_enter = svc.robotServiceEnterTcp2CanbusMode();
    if (rc_enter == an::ErrCode_ResponseReturnError) {
      svc.robotServiceLeaveTcp2CanbusMode();
      rc_enter = svc.robotServiceEnterTcp2CanbusMode();
    }
    record("S6", "enter_tcp2can", rc_enter == an::InterfaceCallSuccCode,
           "rc=" + num(rc_enter));

    bool motion_ok = rc_enter == an::InterfaceCallSuccCode;
    bool saw_running = false;
    if (motion_ok) {
      for (int round = 0; round < rounds && motion_ok && !g_stop.load();
           ++round) {
        const struct {
          const char *name;
          const double *target;
        } legs[2] = {
          {"camera_pose", kCameraPose},
          {"home", kHomePose},
        };
        for (int leg = 0; leg < 2 && motion_ok && !g_stop.load(); ++leg) {
          const bool observe = (round == 0 && leg == 0);
          const StreamResult r =
            streamMoveTcp2Can(svc, legs[leg].target, observe);
          saw_running = saw_running || r.saw_running;
          const bool ok = !r.aborted && r.err >= 0.0 && r.err < 0.01;
          std::ostringstream oss;
          oss << "round=" << round + 1 << " points=" << r.points
              << " total_s=" << std::fixed << std::setprecision(2) << r.total_s
              << " failures=" << r.failures
              << " max_err_rad=" << std::setprecision(5) << r.err;
          record("S6",
                 (std::string("stream_to_") + legs[leg].name).c_str(), ok,
                 oss.str());
          motion_ok = ok;
        }
      }
      if (saw_running) {
        record("S6", "state_running_observed", true, "Running during stream");
      }

      // 结束确保停在 home
      double actual[kDof] = {0};
      const bool read_ok = readJoints(svc, actual);
      double home_err = read_ok ? maxJointError(actual, kHomePose) : -1.0;
      if (home_err > 0.01 && !g_stop.load()) {
        const StreamResult r = streamMoveTcp2Can(svc, kHomePose, false);
        home_err = r.err;
      }
      record("S6", "final_at_home", home_err >= 0.0 && home_err < 0.01,
             "max_err_rad=" + num(home_err));

      svc.rootServiceRobotMoveControl(an::RobotMoveStop);
      const int leave = svc.robotServiceLeaveTcp2CanbusMode();
      record("S6", "leave_tcp2can", leave == an::InterfaceCallSuccCode,
             "rc=" + num(leave));
    }
  } else {
    std::cout << "\n--- S6 运动测试（--skip-motion，跳过）---\n";
  }

  // -------------------------------------------------------------------------
  if (!skip_coexist) {
    std::cout << "\n--- S7 推送+TCP2CAN 并发（破坏性，最后执行）---\n";
    ensureConnected(svc, host, port);
    double setpoint[kDof];
    if (!readJoints(svc, setpoint)) {
      record("S7", "read_setpoint", false, "getJointAngleInfo failed");
    } else {
      int rc = svc.robotServiceEnterTcp2CanbusMode();
      if (rc == an::ErrCode_ResponseReturnError) {
        svc.robotServiceLeaveTcp2CanbusMode();
        rc = svc.robotServiceEnterTcp2CanbusMode();
      }
      g_push_stamps.assign(60000, {});
      g_push_count.store(0);
      const int push_on = svc.robotServiceSetRealTimeJointStatusPush(true);
      const int reg = svc.robotServiceRegisterRealTimeJointStatusCallback(
        &onJointStatusPush, nullptr);

      int consecutive_failures = 0;
      double time_to_failure_s = -1.0;
      const auto c_start = std::chrono::steady_clock::now();
      const auto c_end = c_start + std::chrono::seconds(30);
      while (!g_stop.load() && std::chrono::steady_clock::now() < c_end) {
        const auto cycle_start = std::chrono::steady_clock::now();
        an::RobotDiagnosis diag{};
        const int rc_d = svc.robotServiceGetRobotDiagnosisInfo(diag);
        int rc_w = an::InterfaceCallSuccCode;
        if (rc_d == an::InterfaceCallSuccCode &&
            diag.macTargetPosDataSize < 120) {
          const int batch = std::min(8, 120 - diag.macTargetPosDataSize);
          std::vector<an::wayPoint_S> pts(batch, an::wayPoint_S{});
          for (auto &p : pts) {
            for (int i = 0; i < kDof; ++i) p.jointpos[i] = setpoint[i];
          }
          rc_w = svc.robotServiceSetRobotPosData2Canbus(pts);
        }
        if (rc_d != an::InterfaceCallSuccCode ||
            rc_w != an::InterfaceCallSuccCode) {
          ++consecutive_failures;
        } else {
          consecutive_failures = 0;
        }
        bool connected = true;
        svc.robotServiceGetConnectStatus(connected);
        if (!connected || consecutive_failures >= 10) {
          time_to_failure_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - c_start).count();
          break;
        }
        std::this_thread::sleep_until(
          cycle_start + std::chrono::milliseconds(5));
      }

      svc.robotServiceSetRealTimeJointStatusPush(false);
      svc.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);
      svc.rootServiceRobotMoveControl(an::RobotMoveStop);
      svc.robotServiceLeaveTcp2CanbusMode();

      std::ostringstream oss;
      oss << "enter_rc=" << rc << " push_on_rc=" << push_on << " reg_rc=" << reg
          << " pushes_received=" << g_push_count.load()
          << " time_to_failure_s=" << time_to_failure_s
          << (time_to_failure_s < 0 ? " (30s 内未断链)" : " (发生断链/连续失败)");
      std::cout << "  " << oss.str() << '\n';

      // 恢复验证：重登录
      svc.robotServiceLogout();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      const int relogin = svc.robotServiceLogin(host.c_str(), port, "aubo", "123456");
      double joints_after[kDof] = {0};
      const bool recovered = relogin == an::InterfaceCallSuccCode &&
                             readJoints(svc, joints_after);
      record("S7", "coexist_and_reconnect", recovered, oss.str());
    }
  } else {
    std::cout << "\n--- S7 推送+TCP2CAN 并发（--skip-coexist，跳过）---\n";
  }

  // -------------------------------------------------------------------------
  std::cout << "\n--- S8 清理与汇总 ---\n";
  const int logout = svc.robotServiceLogout();
  record("S8", "final_logout", logout == an::InterfaceCallSuccCode,
         "rc=" + num(logout));

  int pass = 0, fail = 0;
  std::cout << "\n===== 汇总 =====\n";
  for (const auto &r : g_results) {
    std::cout << (r.passed ? "PASS" : "FAIL") << "  " << r.stage << "  "
              << r.name << '\n';
    r.passed ? ++pass : ++fail;
  }
  std::cout << "total=" << g_results.size() << " pass=" << pass
            << " fail=" << fail << '\n';
  return fail == 0 ? 0 : 1;
}
