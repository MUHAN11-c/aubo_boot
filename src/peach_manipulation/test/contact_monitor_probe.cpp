// 零 ROS 纯核探针：合成电流序列驱动 ContactMonitor。由
// test_contact_monitor.py 编译运行，不进 gtest / DDS。
#include "peach_manipulation/contact_monitor.hpp"

#include <cstdio>
#include <vector>

using peach_manipulation::ContactDetectConfig;
using peach_manipulation::ContactMonitor;
using peach_manipulation::ContactVerdict;
using peach_manipulation::CurrentSample;

namespace
{

CurrentSample sample(double t, double wrist3)
{
  CurrentSample out;
  out.t = t;
  out.current = {1.0, 1.0, 1.0, 0.5, 0.5, wrist3};
  return out;
}

int fail(const char * name)
{
  std::fprintf(stderr, "FAIL %s\n", name);
  return 1;
}

}  // namespace

int main()
{
  int failures = 0;

  {
    ContactDetectConfig cfg;
    ContactMonitor monitor(cfg);
    monitor.start({sample(0.0, 0.4), sample(0.1, 0.4)});
    const auto report = monitor.update(sample(0.2, 9.0));
    if (report.verdict != ContactVerdict::INSUFFICIENT) {
      failures += fail("disabled");
    }
  }

  {
    ContactDetectConfig cfg;
    cfg.enabled = true;
    cfg.spike_threshold = 2.0;
    ContactMonitor monitor(cfg);
    monitor.start({sample(0.0, 0.4)});
    const auto report = monitor.update(sample(0.2, 9.0));
    if (report.verdict != ContactVerdict::INSUFFICIENT) {
      failures += fail("short_baseline");
    }
  }

  {
    ContactDetectConfig cfg;
    cfg.enabled = true;
    cfg.slope_threshold = 20.0;
    cfg.spike_threshold = 5.0;
    ContactMonitor monitor(cfg);
    std::vector<CurrentSample> baseline;
    for (int i = 0; i < 5; ++i) {
      baseline.push_back(sample(0.05 * i, 0.40));
    }
    monitor.start(baseline);
    bool hard = false;
    for (int i = 0; i < 20; ++i) {
      const double wrist = 0.40 + 0.01 * (i + 1);
      const auto report = monitor.update(sample(0.30 + 0.05 * i, wrist));
      if (report.verdict == ContactVerdict::SUSPECTED_HARD) {
        hard = true;
      }
    }
    if (hard) {
      failures += fail("friction_ramp");
    }
  }

  {
    ContactDetectConfig cfg;
    cfg.enabled = true;
    cfg.slope_threshold = 50.0;
    cfg.spike_threshold = 2.0;
    ContactMonitor monitor(cfg);
    monitor.start({sample(0.0, 0.40), sample(0.1, 0.41), sample(0.2, 0.40)});
    const auto report = monitor.update(sample(0.25, 3.50));
    if (report.verdict != ContactVerdict::SUSPECTED_HARD) {
      failures += fail("spike");
    }
  }

  {
    ContactDetectConfig cfg;
    cfg.enabled = true;
    cfg.slope_threshold = 8.0;
    cfg.spike_threshold = 50.0;
    ContactMonitor monitor(cfg);
    monitor.start({sample(0.0, 0.40), sample(0.1, 0.40), sample(0.2, 0.40)});
    monitor.update(sample(0.25, 0.41));
    const auto report = monitor.update(sample(0.30, 1.20));
    if (report.verdict != ContactVerdict::SUSPECTED_HARD) {
      failures += fail("slope");
    }
  }

  if (failures == 0) {
    std::printf("PASS\n");
  }
  return failures == 0 ? 0 : 1;
}
