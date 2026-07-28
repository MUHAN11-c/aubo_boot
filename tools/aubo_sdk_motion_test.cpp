// AUBO SDK 2.5.3 TCP2CAN 流式插补运动测试：仅 home <-> camera_pose。
//
// 背景：robotServiceJointMove 三种重载均被服务端拒绝（rc=10023），
// 因此运动改走 TCP2CAN 通道（robotServiceSetRobotPosData2Canbus）——
// 与 ros2_control 硬件插件实际使用的运动通道相同。
// 轨迹为两个允许位姿之间的关节空间余弦插值，端点严格等于 home / camera_pose，
// 不使用任何其他位姿。
//
// Build:
//   SDK=src/aubo_e5_hardware/vendor/aubo_sdk_2_5_3
//   g++ -std=c++17 -I $SDK/include tools/aubo_sdk_motion_test.cpp
//     -o build/diagnostics/motion_test -L $SDK/lib -laubo_sdk -Wl,-rpath,$PWD/$SDK/lib
// Usage: motion_test [host] [rounds] [batch_max]

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace an = aubo_robot_namespace;
constexpr int kDof = an::ARM_DOF;

// SRDF group_state，SDK 关节序 shoulder, upperArm, foreArm, wrist1, wrist2, wrist3
const double kHomePose[kDof] = {0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0};
const double kCameraPose[kDof] = {
  -0.27411168813705444, 0.4963911175727844, 1.7700852155685425,
  -0.2978658676147461, 1.571584939956665, -0.2750104069709778};

constexpr int kRibHigh = 120;   // RIB 水位门控上限
constexpr double kPointDt = 0.010;    // 路径点间隔 10ms（100 pts/s）
constexpr double kMaxSpeed = 0.25;    // 关节平均速度上限 rad/s

bool readJoints(ServiceInterface &svc, double out[kDof])
{
  an::JointParam jp{};
  if (svc.robotServiceGetJointAngleInfo(jp) != an::InterfaceCallSuccCode) {
    return false;
  }
  for (int i = 0; i < kDof; ++i) out[i] = jp.jointPos[i];
  return true;
}

double verify(ServiceInterface &svc, const double *target)
{
  double actual[kDof];
  if (!readJoints(svc, actual)) return -1.0;
  double err = 0.0;
  for (int i = 0; i < kDof; ++i) {
    err = std::max(err, std::fabs(actual[i] - target[i]));
  }
  return err;
}

// 流式插补运动：当前位姿 -> target，返回 (到位误差, 耗时, 失败标志)
double streamMove(ServiceInterface &svc, const double *target, const char *name,
                  bool &failed, int batch_max)
{
  failed = false;
  double start[kDof];
  if (!readJoints(svc, start)) {
    std::cout << "  " << name << ": read start joints failed\n";
    failed = true;
    return -1.0;
  }

  double delta[kDof], max_delta = 0.0;
  for (int i = 0; i < kDof; ++i) {
    delta[i] = target[i] - start[i];
    max_delta = std::max(max_delta, std::fabs(delta[i]));
  }
  if (max_delta < 1e-6) {
    std::cout << "  " << name << ": already at target\n";
    return verify(svc, target);
  }

  const double duration = std::max(2.0, max_delta / kMaxSpeed);
  const int n = static_cast<int>(duration / kPointDt) + 1;
  std::vector<std::array<double, kDof>> path(n);
  for (int k = 0; k < n; ++k) {
    const double tau = static_cast<double>(k) / (n - 1);
    const double s = 0.5 * (1.0 - std::cos(M_PI * tau));  // 余弦缓动，端点速度为 0
    for (int i = 0; i < kDof; ++i) {
      path[k][i] = start[i] + s * delta[i];
    }
  }

  std::cout << "  " << name << ": max_delta=" << std::fixed
            << std::setprecision(3) << max_delta << " rad, points=" << n
            << ", planned_s=" << std::setprecision(2) << duration << '\n';

  const auto t0 = std::chrono::steady_clock::now();
  std::size_t idx = 0;
  int last_rib = 0, failures = 0;
  an::RobotDiagnosis diag{};
  while (idx < path.size() && failures < 10) {
    const auto cycle = std::chrono::steady_clock::now();
    if (svc.robotServiceGetRobotDiagnosisInfo(diag) ==
        an::InterfaceCallSuccCode) {
      last_rib = diag.macTargetPosDataSize;
    } else {
      ++failures;
    }
    if (last_rib < kRibHigh) {
      const int batch = std::min(
        {batch_max, kRibHigh - last_rib,
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
        ++failures;
      }
    }
    std::this_thread::sleep_until(cycle + std::chrono::milliseconds(5));
  }
  if (failures >= 10) {
    std::cout << "  " << name << ": abort, failures=" << failures << '\n';
    svc.rootServiceRobotMoveControl(an::RobotMoveStop);
    failed = true;
    return -1.0;
  }

  // 等 RIB 排空（机械臂走完缓存点）
  double drain_s = -1.0;
  const auto d0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - d0 < std::chrono::seconds(20)) {
    if (svc.robotServiceGetRobotDiagnosisInfo(diag) ==
          an::InterfaceCallSuccCode &&
        diag.macTargetPosDataSize == 0) {
      drain_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - d0).count();
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const double total_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t0).count();
  std::cout << "  " << name << ": streamed " << idx << " pts, total_s="
            << std::setprecision(2) << total_s << " drain_s=" << drain_s
            << " failures=" << failures << '\n';
  if (drain_s < 0) {
    failed = true;
    return -1.0;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // 稳定后读数
  return verify(svc, target);
}

int main(int argc, char **argv)
{
  const std::string host = argc > 1 ? argv[1] : "169.254.10.98";
  const int rounds = argc > 2 ? std::max(1, std::atoi(argv[2])) : 1;
  const int batch_max = argc > 3 ? std::clamp(std::atoi(argv[3]), 1, 32) : 8;
  std::cout << "host=" << host << " rounds=" << rounds
            << " batch_max=" << batch_max << '\n';

  ServiceInterface svc;
  const int login = svc.robotServiceLogin(host.c_str(), 8899, "aubo", "123456");
  std::cout << "login=" << login << '\n';
  if (login != 0) return 10;

  an::ToolDynamicsParam tdp{};
  an::ROBOT_SERVICE_STATE state;
  std::cout << "startup="
            << svc.rootServiceRobotStartup(tdp, 6, true, true, 1000, state)
            << " state=" << static_cast<int>(state) << '\n';

  int rc = svc.robotServiceEnterTcp2CanbusMode();
  if (rc == an::ErrCode_ResponseReturnError) {
    svc.robotServiceLeaveTcp2CanbusMode();
    rc = svc.robotServiceEnterTcp2CanbusMode();
  }
  std::cout << "enter_tcp2can=" << rc << '\n';
  if (rc != an::InterfaceCallSuccCode) {
    svc.robotServiceLogout();
    return 20;
  }

  // 回放周期实验（argv[4] > 0 时设置，单位秒）：验证 SetTrackPlaybackCycle
  // 是否控制 TCP2CAN RIB 的消费节奏。默认约 1.4s/腿（消费≈供给 ~150 pts/s），
  // 若 10ms 生效应变为 ~2.1s/腿（100 pts/s = 内容时长）。
  if (argc > 4 && std::atof(argv[4]) > 0.0) {
    const double cycle = std::atof(argv[4]);
    std::cout << "set_playback_cycle(" << cycle << ")="
              << svc.robotServiceSetTrackPlaybackCycle(cycle)
              << " get=" << svc.robotServiceGetTrackPlaybackCycle() << '\n';
  } else {
    std::cout << "playback_cycle(default)=" << svc.robotServiceGetTrackPlaybackCycle() << '\n';
  }

  bool ok = true;
  for (int r = 0; r < rounds && ok; ++r) {
    bool failed = false;
    const double err1 = streamMove(svc, kCameraPose, "camera_pose", failed, batch_max);
    std::cout << "round " << r + 1 << " camera_pose err="
              << std::setprecision(5) << err1 << '\n';
    ok = !failed && err1 >= 0.0 && err1 < 0.01;
    if (!ok) break;
    const double err2 = streamMove(svc, kHomePose, "home", failed, batch_max);
    std::cout << "round " << r + 1 << " home err=" << err2 << '\n';
    ok = !failed && err2 >= 0.0 && err2 < 0.01;
  }

  std::cout << "move_stop="
            << svc.rootServiceRobotMoveControl(an::RobotMoveStop)
            << " leave=" << svc.robotServiceLeaveTcp2CanbusMode()
            << " logout=" << svc.robotServiceLogout() << '\n';
  return ok ? 0 : 1;
}
