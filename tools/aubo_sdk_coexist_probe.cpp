// AUBO SDK 2.5.3 推送+TCP2CAN 并发长时探针（健康通道复测）。
// 单连接上同时开启关节状态推送与 TCP2CAN 回写（当前位姿，无运动），
// 统计推送速率、写失败、断链时间。此前（网卡驱动修复前）同连接
// 22-28s 必断链（2/2 复现）；修复后 full_test 30s 未断，本探针加长验证。
//
// Build:
//   SDK=src/aubo_e5_hardware/vendor/aubo_sdk_2_5_3
//   g++ -std=c++17 -I $SDK/include tools/aubo_sdk_coexist_probe.cpp
//     -o build/diagnostics/coexist_probe -L $SDK/lib -laubo_sdk -Wl,-rpath,$PWD/$SDK/lib
// Usage: coexist_probe [host] [duration_sec]   (default 169.254.10.98, 300)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace an = aubo_robot_namespace;
std::atomic<std::size_t> g_push_count{0};

void onJointStatus(const an::JointStatus *, int, void *)
{
  g_push_count.fetch_add(1, std::memory_order_relaxed);
}

int main(int argc, char **argv)
{
  const char *host = argc > 1 ? argv[1] : "169.254.10.98";
  const int duration_sec = argc > 2 ? std::max(10, std::atoi(argv[2])) : 300;

  ServiceInterface svc;
  const int login = svc.robotServiceLogin(host, 8899, "aubo", "123456");
  std::cout << "login=" << login << '\n';
  if (login != 0) return 10;

  an::JointParam jp{};
  if (svc.robotServiceGetJointAngleInfo(jp) != an::InterfaceCallSuccCode) {
    std::cout << "read setpoint failed\n";
    return 20;
  }

  int rc = svc.robotServiceEnterTcp2CanbusMode();
  if (rc == an::ErrCode_ResponseReturnError) {
    svc.robotServiceLeaveTcp2CanbusMode();
    rc = svc.robotServiceEnterTcp2CanbusMode();
  }
  std::cout << "enter_tcp2can=" << rc << '\n';
  if (rc != an::InterfaceCallSuccCode) return 30;

  const int push_on = svc.robotServiceSetRealTimeJointStatusPush(true);
  const int reg = svc.robotServiceRegisterRealTimeJointStatusCallback(
    &onJointStatus, nullptr);
  std::cout << "push_on=" << push_on << " reg=" << reg << '\n';

  const auto start = std::chrono::steady_clock::now();
  const auto end = start + std::chrono::seconds(duration_sec);
  int consecutive_failures = 0, write_calls = 0, write_failures = 0;
  long points_written = 0;
  double time_to_failure_s = -1.0;
  std::size_t last_push_report = 0;

  while (std::chrono::steady_clock::now() < end) {
    const auto cycle = std::chrono::steady_clock::now();
    an::RobotDiagnosis diag{};
    const int rc_d = svc.robotServiceGetRobotDiagnosisInfo(diag);
    int rc_w = an::InterfaceCallSuccCode;
    if (rc_d == an::InterfaceCallSuccCode && diag.macTargetPosDataSize < 120) {
      const int batch = std::min(8, 120 - diag.macTargetPosDataSize);
      std::vector<an::wayPoint_S> pts(batch, an::wayPoint_S{});
      for (auto &p : pts) {
        for (int i = 0; i < 6; ++i) p.jointpos[i] = jp.jointPos[i];
      }
      ++write_calls;
      rc_w = svc.robotServiceSetRobotPosData2Canbus(pts);
      if (rc_w == an::InterfaceCallSuccCode) points_written += batch;
    }
    if (rc_d != an::InterfaceCallSuccCode ||
        rc_w != an::InterfaceCallSuccCode) {
      ++consecutive_failures;
      if (rc_w != an::InterfaceCallSuccCode) ++write_failures;
    } else {
      consecutive_failures = 0;
    }
    bool connected = true;
    svc.robotServiceGetConnectStatus(connected);
    if (!connected || consecutive_failures >= 10) {
      time_to_failure_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      std::cout << "FAILURE at t=" << time_to_failure_s
                << "s connected=" << connected
                << " consecutive_failures=" << consecutive_failures << '\n';
      break;
    }
    // 每 30s 打印一次进度
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
      cycle - start).count();
    if (elapsed > 0 && elapsed % 30 == 0 &&
        g_push_count.load() != last_push_report) {
      last_push_report = g_push_count.load();
      std::cout << "t=" << elapsed << "s pushes=" << g_push_count.load()
                << " rate_hz=" << std::fixed << std::setprecision(1)
                << (static_cast<double>(g_push_count.load()) / elapsed)
                << " points_written=" << points_written << '\n';
    }
    std::this_thread::sleep_until(cycle + std::chrono::milliseconds(5));
  }

  const double total_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();
  svc.robotServiceSetRealTimeJointStatusPush(false);
  svc.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);
  svc.rootServiceRobotMoveControl(an::RobotMoveStop);
  svc.robotServiceLeaveTcp2CanbusMode();
  svc.robotServiceLogout();

  std::cout << std::fixed << std::setprecision(2)
            << "duration_s=" << total_s
            << " pushes=" << g_push_count.load()
            << " push_rate_hz=" << (g_push_count.load() / total_s)
            << " write_calls=" << write_calls
            << " write_failures=" << write_failures
            << " points_written=" << points_written
            << " time_to_failure_s=" << time_to_failure_s
            << (time_to_failure_s < 0 ? " (全程未断链)" : " (断链/连续失败)")
            << '\n';
  return time_to_failure_s < 0 ? 0 : 40;
}
