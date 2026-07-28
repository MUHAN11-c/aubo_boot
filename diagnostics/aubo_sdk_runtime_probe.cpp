// AUBO SDK 状态轮询探针（零运动）：按固定周期调用
// robotServiceGetRobotJointStatus，测量单次调用延迟、失败率与阻塞事件。
// 逐样本数据写入 CSV（默认 results/runtime_probe_latest.csv，每次运行覆盖），
// 供 plot_results.py 绘制延迟曲线与频率曲线。
//
// 用法: aubo_sdk_runtime_probe [host] [samples] [period_ms] [csv_path]

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

int main(int argc, char **argv)
{
  const char *host = argc > 1 ? argv[1] : "169.254.10.98";
  const int samples = argc > 2 ? std::max(1, std::atoi(argv[2])) : 300;
  const int period_ms = argc > 3 ? std::max(1, std::atoi(argv[3])) : 100;
  const char *csv_path =
    argc > 4 ? argv[4] : "results/runtime_probe_latest.csv";

  ServiceInterface service;
  aubo_robot_namespace::RobotType robot_type{};
  aubo_robot_namespace::RobotDhPara dh{};
  const int login =
    service.robotServiceLogin(host, 8899, "aubo", "123456", robot_type, dh);
  std::cout << "login=" << login
            << " robot_type=" << static_cast<int>(robot_type)
            << " dh=" << dh.A3 << ',' << dh.A4 << ',' << dh.D1 << ','
            << dh.D2 << ',' << dh.D5 << ',' << dh.D6 << '\n';
  if (login != 0) {
    return 10;
  }

  const int initial_handshake = service.robotServiceRobotHandShake(true);
  std::cout << "initial_handshake=" << initial_handshake << '\n';
  if (initial_handshake != 0) {
    service.robotServiceLogout();
    return 20;
  }

  std::array<aubo_robot_namespace::JointStatus, 6> joints{};
  std::array<double, 6> previous{};
  bool have_previous = false;
  int read_failures = 0;
  int handshake_failures = 0;
  int invalid_samples = 0;
  double largest_jump = 0.0;
  double minimum_latency_ms = std::numeric_limits<double>::infinity();
  double maximum_latency_ms = 0.0;
  double total_latency_ms = 0.0;
  // 阻塞事件阈值：正常调用 ~2-4ms，超过 100ms 视为一次阻塞
  constexpr double kBlockThresholdMs = 100.0;
  int block_events = 0;

  std::ofstream csv(csv_path, std::ios::trunc);
  if (csv) {
    csv << "sample,t_sec,latency_ms,rc\n";
  } else {
    std::cout << "warn: cannot open csv_path=" << csv_path << '\n';
  }

  const auto test_start = std::chrono::steady_clock::now();
  for (int sample = 0; sample < samples; ++sample) {
    if (sample != 0 && sample % 50 == 0) {
      const int result = service.robotServiceRobotHandShake(true);
      handshake_failures += result != 0;
    }

    const auto call_start = std::chrono::steady_clock::now();
    const int result =
      service.robotServiceGetRobotJointStatus(joints.data(), joints.size());
    const double latency_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - call_start).count();
    minimum_latency_ms = std::min(minimum_latency_ms, latency_ms);
    maximum_latency_ms = std::max(maximum_latency_ms, latency_ms);
    total_latency_ms += latency_ms;
    if (latency_ms > kBlockThresholdMs) {
      ++block_events;
    }
    if (csv) {
      const double t_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - test_start).count();
      csv << sample << ',' << t_sec << ',' << latency_ms << ',' << result
          << '\n';
    }

    if (result != 0) {
      ++read_failures;
    } else {
      for (std::size_t joint = 0; joint < joints.size(); ++joint) {
        const double position = joints[joint].jointPosJ;
        if (!std::isfinite(position) || std::abs(position) > 10.0) {
          ++invalid_samples;
        }
        if (have_previous) {
          largest_jump = std::max(largest_jump, std::abs(position - previous[joint]));
        }
        previous[joint] = position;
      }
      have_previous = true;
    }

    const auto deadline =
      test_start + std::chrono::milliseconds((sample + 1) * period_ms);
    std::this_thread::sleep_until(deadline);
  }

  std::cout << std::fixed << std::setprecision(3)
            << "samples=" << samples
            << " period_ms=" << period_ms
            << " read_failures=" << read_failures
            << " handshake_failures=" << handshake_failures
            << " invalid_values=" << invalid_samples
            << " latency_ms[min/avg/max]=" << minimum_latency_ms << '/'
            << total_latency_ms / samples << '/' << maximum_latency_ms
            << " block_events(gt" << static_cast<int>(kBlockThresholdMs)
            << "ms)=" << block_events
            << " largest_jump_rad=" << largest_jump << '\n'
            << "last_positions_rad=";
  for (std::size_t joint = 0; joint < previous.size(); ++joint) {
    std::cout << (joint == 0 ? "" : ",") << previous[joint];
  }
  std::cout << '\n';

  const int logout = service.robotServiceLogout();
  std::cout << "logout=" << logout << '\n';
  return read_failures == 0 && handshake_failures == 0 &&
      invalid_samples == 0 && logout == 0 ? 0 : 30;
}
