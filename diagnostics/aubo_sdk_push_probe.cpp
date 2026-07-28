// AUBO SDK 实时关节状态推送探针（零运动，回调内不调 SDK）。测量服务器推送
// 频率、间隔抖动、推送空洞（服务器断连）与数据是否真实刷新。
// 逐次推送的时间戳写入 CSV（默认 results/push_probe_latest.csv，每次运行
// 覆盖），供 plot_results.py 绘制频率/间隔曲线。
//
// 用法: aubo_sdk_push_probe [host] [duration_sec] [csv_path]

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace {

std::atomic<std::size_t> g_count{0};
std::array<double, 6> g_last_position{};
std::mutex g_position_mutex;

void onJointStatus(const aubo_robot_namespace::JointStatus *status, int size,
                   void *arg)
{
  auto *stamps = static_cast<std::vector<std::chrono::steady_clock::time_point> *>(arg);
  const std::size_t index = g_count.fetch_add(1, std::memory_order_relaxed);
  if (index < stamps->size()) {
    (*stamps)[index] = std::chrono::steady_clock::now();
  }
  if (status != nullptr && size >= 6) {
    std::lock_guard<std::mutex> lock(g_position_mutex);
    for (int joint = 0; joint < 6; ++joint) {
      g_last_position[joint] = status[joint].jointPosJ;
    }
  }
}

}  // namespace

int main(int argc, char **argv)
{
  const char *host = argc > 1 ? argv[1] : "169.254.10.98";
  const int duration_sec = argc > 2 ? std::max(5, std::atoi(argv[2])) : 60;
  const char *csv_path =
    argc > 3 ? argv[3] : "results/push_probe_latest.csv";

  ServiceInterface service;
  const int login = service.robotServiceLogin(host, 8899, "aubo", "123456");
  std::cout << "login=" << login << '\n';
  if (login != 0) {
    return 10;
  }

  // Generous headroom: even 1 kHz for the whole run fits.
  std::vector<std::chrono::steady_clock::time_point> stamps(
    static_cast<std::size_t>(duration_sec) * 2000);

  const int push_on = service.robotServiceSetRealTimeJointStatusPush(true);
  const int reg = service.robotServiceRegisterRealTimeJointStatusCallback(
    &onJointStatus, &stamps);
  std::cout << "push_enable=" << push_on << " register=" << reg << '\n';
  if (push_on != 0 || reg != 0) {
    service.robotServiceLogout();
    return 20;
  }

  const auto start = std::chrono::steady_clock::now();
  std::array<double, 6> first_position{};
  std::array<double, 6> mid_position{};
  double max_deviation = 0.0;
  std::size_t previous_count = 0;
  for (int sec = 0; sec < duration_sec; ++sec) {
    std::this_thread::sleep_until(start + std::chrono::seconds(sec + 1));
    const std::size_t now_count = g_count.load(std::memory_order_relaxed);
    std::cout << "sec=" << (sec + 1) << " pushes_this_sec="
              << (now_count - previous_count) << " total=" << now_count << '\n';
    previous_count = now_count;
    std::lock_guard<std::mutex> lock(g_position_mutex);
    if (sec == 0) {
      first_position = g_last_position;
    }
    if (sec == duration_sec / 2) {
      mid_position = g_last_position;
    }
    for (int joint = 0; joint < 6; ++joint) {
      max_deviation = std::max(
        max_deviation, std::abs(g_last_position[joint] - first_position[joint]));
    }
  }

  const int push_off = service.robotServiceSetRealTimeJointStatusPush(false);
  const int unreg =
    service.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);
  const int logout = service.robotServiceLogout();

  const std::size_t count =
    std::min(g_count.load(std::memory_order_relaxed), stamps.size());
  double min_interval_ms = std::numeric_limits<double>::infinity();
  double max_interval_ms = 0.0;
  double total_interval_ms = 0.0;
  double variance_acc = 0.0;
  // 推送空洞阈值：正常间隔 ~30ms，超过 100ms 视为一次推送空洞
  constexpr double kHoleThresholdMs = 100.0;
  int hole_events = 0;
  if (count > 1) {
    for (std::size_t i = 1; i < count; ++i) {
      const double interval_ms = std::chrono::duration<double, std::milli>(
        stamps[i] - stamps[i - 1]).count();
      min_interval_ms = std::min(min_interval_ms, interval_ms);
      max_interval_ms = std::max(max_interval_ms, interval_ms);
      total_interval_ms += interval_ms;
      if (interval_ms > kHoleThresholdMs) {
        ++hole_events;
      }
    }
    const double mean = total_interval_ms / (count - 1);
    for (std::size_t i = 1; i < count; ++i) {
      const double interval_ms = std::chrono::duration<double, std::milli>(
        stamps[i] - stamps[i - 1]).count();
      const double d = interval_ms - mean;
      variance_acc += d * d;
    }
    variance_acc /= (count - 1);
  }

  // 逐次推送数据落盘（每次运行覆盖最新）
  {
    std::ofstream csv(csv_path, std::ios::trunc);
    if (csv) {
      csv << "index,t_sec,interval_ms\n";
      for (std::size_t i = 0; i < count; ++i) {
        const double t_sec = std::chrono::duration<double>(
          stamps[i] - stamps[0]).count();
        const double interval_ms =
          i == 0 ? 0.0
                 : std::chrono::duration<double, std::milli>(
                     stamps[i] - stamps[i - 1]).count();
        csv << i << ',' << t_sec << ',' << interval_ms << '\n';
      }
    } else {
      std::cout << "warn: cannot open csv_path=" << csv_path << '\n';
    }
  }

  std::cout << std::fixed << std::setprecision(3)
            << "duration_sec=" << duration_sec
            << " pushes=" << count
            << " rate_hz=" << (count > 0 ? static_cast<double>(count) / duration_sec : 0.0)
            << " interval_ms[min/avg/max]="
            << (count > 1 ? min_interval_ms : 0.0) << '/'
            << (count > 1 ? total_interval_ms / (count - 1) : 0.0) << '/'
            << (count > 1 ? max_interval_ms : 0.0)
            << " jitter_std_ms=" << std::sqrt(variance_acc)
            << " hole_events(gt" << static_cast<int>(kHoleThresholdMs)
            << "ms)=" << hole_events
            << " max_position_deviation_rad=" << max_deviation << '\n'
            << "first_positions_rad=";
  for (int joint = 0; joint < 6; ++joint) {
    std::cout << (joint == 0 ? "" : ",") << first_position[joint];
  }
  std::cout << "\nmid_positions_rad=";
  for (int joint = 0; joint < 6; ++joint) {
    std::cout << (joint == 0 ? "" : ",") << mid_position[joint];
  }
  std::cout << "\npush_off=" << push_off << " unregister=" << unreg
            << " logout=" << logout << '\n';
  return count > 0 && push_off == 0 && logout == 0 ? 0 : 30;
}
