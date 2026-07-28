// AUBO SDK TCP2CAN 写入路径探针（零运动）：读取一次当前关节位置作为固定设定点
// 原样回写，机械臂保持不动，测量指令通道的写延迟、RIB 水位与吞吐。
//
// 模拟硬件插件发送线程：5ms 周期、RIB 水位门控批量写（每次
// robotServiceSetRobotPosData2Canbus ≤ batch_max 个 wayPoint_S）、周期性
// robotServiceGetRobotDiagnosisInfo 读取 macTargetPosDataSize（RIB 水位）。
// 逐周期数据（RIB 水位、写/诊断延迟）写入 CSV（默认
// results/tcp2can_probe_latest.csv，每次运行覆盖），供 plot_results.py 绘图。
//
// 用法: aubo_sdk_tcp2can_probe [host] [duration_sec] [batch_max]
//       [diag_interval_cycles] [csv_path]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop.store(true); }

struct Stats {
  double min_ms = std::numeric_limits<double>::infinity();
  double max_ms = 0.0;
  double total_ms = 0.0;
  std::size_t count = 0;
  std::vector<double> samples;  // for percentiles

  void add(double ms) {
    min_ms = std::min(min_ms, ms);
    max_ms = std::max(max_ms, ms);
    total_ms += ms;
    ++count;
    samples.push_back(ms);
  }

  double avg() const { return count ? total_ms / count : 0.0; }

  double percentile(double p) {
    if (samples.empty()) return 0.0;
    std::sort(samples.begin(), samples.end());
    const std::size_t index = std::min(
      samples.size() - 1,
      static_cast<std::size_t>(p / 100.0 * samples.size()));
    return samples[index];
  }
};

}  // namespace

int main(int argc, char **argv)
{
  const char *host = argc > 1 ? argv[1] : "169.254.10.98";
  const int duration_sec = argc > 2 ? std::max(5, std::atoi(argv[2])) : 30;
  const int batch_arg = argc > 3 ? std::max(1, std::atoi(argv[3])) : 8;
  const int diag_interval = argc > 4 ? std::max(1, std::atoi(argv[4])) : 1;
  const char *csv_path =
    argc > 5 ? argv[5] : "results/tcp2can_probe_latest.csv";

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  ServiceInterface service;
  const int login = service.robotServiceLogin(host, 8899, "aubo", "123456");
  std::cout << "login=" << login << '\n';
  if (login != 0) {
    return 10;
  }

  // Fixed setpoint = current arm pose (no motion).
  aubo_robot_namespace::JointStatus joints[6]{};
  int ret = service.robotServiceGetRobotJointStatus(joints, 6);
  if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
    std::cout << "initial_joint_status_failed=" << ret << '\n';
    service.robotServiceLogout();
    return 20;
  }
  double setpoint[6];
  std::cout << "setpoint_rad=";
  for (int i = 0; i < 6; ++i) {
    setpoint[i] = joints[i].jointPosJ;
    std::cout << (i == 0 ? "" : ",") << setpoint[i];
  }
  std::cout << '\n';

  ret = service.robotServiceEnterTcp2CanbusMode();
  if (ret == aubo_robot_namespace::ErrCode_ResponseReturnError) {
    service.robotServiceLeaveTcp2CanbusMode();
    ret = service.robotServiceEnterTcp2CanbusMode();
  }
  std::cout << "enter_tcp2can=" << ret << '\n';
  if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
    service.robotServiceLogout();
    return 30;
  }

  // Target band follows AuboRos2System's sender loop.
  constexpr int kRibHigh = 120;
  const int kBatchMax = batch_arg;

  Stats write_stats;
  Stats diag_stats;
  int rib_min = std::numeric_limits<int>::max();
  int rib_max = 0;
  long rib_total = 0;
  std::size_t rib_samples = 0;
  int diag_failures = 0;
  int write_failures = 0;
  int cycles = 0;
  long points_written = 0;
  int last_rib = 0;
  // 阻塞事件阈值：正常写/诊断 ~2-5ms，超过 100ms 视为一次阻塞
  constexpr double kBlockThresholdMs = 100.0;
  int write_block_events = 0;
  int diag_block_events = 0;

  std::ofstream csv(csv_path, std::ios::trunc);
  if (csv) {
    csv << "phase,t_sec,rib,write_ms,write_rc,batch,diag_ms,diag_rc\n";
  } else {
    std::cout << "warn: cannot open csv_path=" << csv_path << '\n';
  }

  const auto start = std::chrono::steady_clock::now();
  const auto end = start + std::chrono::seconds(duration_sec);
  while (!g_stop.load() && std::chrono::steady_clock::now() < end) {
    const auto cycle_start = std::chrono::steady_clock::now();
    double diag_ms = -1.0;
    int diag_rc = -1;
    double write_ms = -1.0;
    int write_rc = -1;
    int batch_sent = 0;

    if (cycles % diag_interval == 0) {
      aubo_robot_namespace::RobotDiagnosis diag{};
      auto call_start = std::chrono::steady_clock::now();
      ret = service.robotServiceGetRobotDiagnosisInfo(diag);
      diag_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - call_start).count();
      diag_rc = ret;
      diag_stats.add(diag_ms);
      if (diag_ms > kBlockThresholdMs) {
        ++diag_block_events;
      }
      if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        ++diag_failures;
      } else {
        last_rib = diag.macTargetPosDataSize;
        rib_min = std::min(rib_min, last_rib);
        rib_max = std::max(rib_max, last_rib);
        rib_total += last_rib;
        ++rib_samples;
      }
    }

    if (last_rib < kRibHigh) {
      const int batch = std::min(kBatchMax, kRibHigh - last_rib);
      std::vector<aubo_robot_namespace::wayPoint_S> points(
        batch, aubo_robot_namespace::wayPoint_S{});
      for (auto &point : points) {
        for (int i = 0; i < 6; ++i) {
          point.jointpos[i] = setpoint[i];
        }
      }
      auto call_start = std::chrono::steady_clock::now();
      ret = service.robotServiceSetRobotPosData2Canbus(points);
      write_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - call_start).count();
      write_rc = ret;
      batch_sent = batch;
      write_stats.add(write_ms);
      if (write_ms > kBlockThresholdMs) {
        ++write_block_events;
      }
      if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        ++write_failures;
      } else {
        points_written += batch;
        last_rib += batch;  // optimistic estimate between diag queries
      }
    }

    if (csv) {
      const double t_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      csv << "stream," << t_sec << ',' << last_rib << ',';
      if (write_ms >= 0.0) {
        csv << write_ms << ',' << write_rc << ',' << batch_sent;
      } else {
        csv << ",,";  // 无写调用：write_ms/write_rc/batch 三个字段留空
      }
      csv << ',';
      if (diag_ms >= 0.0) {
        csv << diag_ms << ',' << diag_rc;
      } else {
        csv << ',';  // 无诊断调用：diag_ms/diag_rc 留空
      }
      csv << '\n';
    }

    ++cycles;
    std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(5));
  }

  // Drain measurement: stop writing, watch RIB empty -> consumption rate.
  int drain_start_rib = 0;
  double drain_seconds = -1.0;
  {
    aubo_robot_namespace::RobotDiagnosis diag{};
    if (service.robotServiceGetRobotDiagnosisInfo(diag) ==
        aubo_robot_namespace::InterfaceCallSuccCode) {
      drain_start_rib = diag.macTargetPosDataSize;
      const auto drain_start = std::chrono::steady_clock::now();
      while (!g_stop.load() &&
             std::chrono::steady_clock::now() - drain_start <
               std::chrono::seconds(15)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 5ms 轮询
        // （原 50ms：48 点排空 <55ms 时只有 1 个采样点，无法测斜率）
        if (service.robotServiceGetRobotDiagnosisInfo(diag) !=
            aubo_robot_namespace::InterfaceCallSuccCode) {
          continue;
        }
        const double drain_t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - drain_start).count();
        std::cout << "drain_t=" << drain_t
                  << " rib=" << diag.macTargetPosDataSize << '\n';
        if (csv) {
          const double t_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
          csv << "drain," << t_sec << ',' << diag.macTargetPosDataSize
              << ",,,,,\n";
        }
        if (diag.macTargetPosDataSize == 0) {
          drain_seconds = drain_t;
          break;
        }
      }
    }
  }

  // Clean shutdown: stop -> leave -> logout.
  const int stop = service.rootServiceRobotMoveControl(
    aubo_robot_namespace::RobotMoveStop);
  const int leave = service.robotServiceLeaveTcp2CanbusMode();
  const int logout = service.robotServiceLogout();

  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();
  std::cout << std::fixed << std::setprecision(3)
            << "cycles=" << cycles
            << " batch_max=" << kBatchMax
            << " diag_interval=" << diag_interval
            << " points_written=" << points_written
            << " production_pts_per_s="
            << (elapsed > 0 ? points_written / elapsed : 0.0)
            << " write_calls=" << write_stats.count
            << " write_failures=" << write_failures
            << " write_ms[min/avg/p99/max]=" << write_stats.min_ms << '/'
            << write_stats.avg() << '/' << write_stats.percentile(99.0) << '/'
            << write_stats.max_ms
            << " write_block_events(gt" << static_cast<int>(kBlockThresholdMs)
            << "ms)=" << write_block_events << '\n'
            << "diag_calls=" << diag_stats.count
            << " diag_failures=" << diag_failures
            << " diag_ms[min/avg/p99/max]=" << diag_stats.min_ms << '/'
            << diag_stats.avg() << '/' << diag_stats.percentile(99.0) << '/'
            << diag_stats.max_ms
            << " diag_block_events(gt" << static_cast<int>(kBlockThresholdMs)
            << "ms)=" << diag_block_events << '\n'
            << "rib[min/avg/max]="
            << (rib_samples ? rib_min : 0) << '/'
            << (rib_samples ? static_cast<double>(rib_total) / rib_samples : 0.0)
            << '/' << rib_max
            << " drain_start_rib=" << drain_start_rib
            << " drain_seconds=" << drain_seconds
            << " consumption_pts_per_s="
            << (drain_seconds > 0 ? drain_start_rib / drain_seconds : 0.0)
            << " stop=" << stop << " leave=" << leave << " logout=" << logout
            << '\n';
  return write_failures == 0 && logout == 0 ? 0 : 40;
}
