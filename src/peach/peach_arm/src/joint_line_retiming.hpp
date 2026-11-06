#ifndef PEACH_ARM__JOINT_LINE_RETIMING_HPP_
#define PEACH_ARM__JOINT_LINE_RETIMING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <ruckig/ruckig.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace peach_arm
{

struct JointLineLimit
{
  double lower, upper, velocity, acceleration, jerk;
};

struct JointLineRetimeReport
{
  std::string reason;
  double before_s{0.0}, after_s{0.0};
  double velocity_ratio{0.0}, acceleration_ratio{0.0}, jerk_ratio{0.0};
  std::vector<JointLineLimit> validated_limits;
};

inline int64_t jointLineTimeNs(const builtin_interfaces::msg::Duration & time)
{
  return static_cast<int64_t>(time.sec) * 1000000000LL + time.nanosec;
}

inline long double binomial(unsigned n, unsigned k)
{
  long double value = 1.0L;
  for (unsigned i = 1; i <= k; ++i) {
    value *= (n + 1 - i) / static_cast<long double>(i);
  }
  return value;
}

// 多项式在[0,1]上位于Bernstein控制值的凸包内；这是整个区间的保守界，
// 不是采样点极值。纳秒时间、q/v/a均来自最终待发送消息，与JTC quintic一致。
template<std::size_t N>
inline std::array<long double, 2> polynomialEnvelope(const std::array<long double, N> & power)
{
  std::array<long double, 2> range{
    std::numeric_limits<long double>::infinity(),
    -std::numeric_limits<long double>::infinity()};
  for (unsigned k = 0; k < N; ++k) {
    long double b = 0.0L;
    for (unsigned i = 0; i <= k; ++i) {
      b += power[i] * binomial(k, i) / binomial(N - 1, i);
    }
    range[0] = std::min(range[0], b);
    range[1] = std::max(range[1], b);
  }
  return range;
}

inline bool jointLineQuinticWithinLimits(
  const trajectory_msgs::msg::JointTrajectory & path,
  const std::vector<JointLineLimit> & limits,
  JointLineRetimeReport & report)
{
  if (path.points.size() < 2 || limits.size() != path.joint_names.size()) {return false;}
  for (std::size_t k = 1; k < path.points.size(); ++k) {
    const auto & l = path.points[k - 1];
    const auto & r = path.points[k];
    const long double dt = (jointLineTimeNs(r.time_from_start) -
      jointLineTimeNs(l.time_from_start)) * 1.0e-9L;
    if (dt <= 0.0L || l.positions.size() != limits.size() ||
      l.velocities.size() != limits.size() || l.accelerations.size() != limits.size() ||
      r.positions.size() != limits.size() || r.velocities.size() != limits.size() ||
      r.accelerations.size() != limits.size()) {return false;}
    for (std::size_t j = 0; j < limits.size(); ++j) {
      const long double c1 = l.velocities[j] * dt;
      const long double c2 = l.accelerations[j] * dt * dt / 2.0L;
      const long double x = static_cast<long double>(r.positions[j]) - l.positions[j] - c1 - c2;
      const long double v = r.velocities[j] * dt - c1 - 2.0L * c2;
      const long double a = r.accelerations[j] * dt * dt - 2.0L * c2;
      const long double c3 = 10.0L * x - 4.0L * v + a / 2.0L;
      const long double c4 = -15.0L * x + 7.0L * v - a;
      const long double c5 = 6.0L * x - 3.0L * v + a / 2.0L;
      const auto qr = polynomialEnvelope<6>({l.positions[j], c1, c2, c3, c4, c5});
      const auto vr = polynomialEnvelope<5>({c1, 2 * c2, 3 * c3, 4 * c4, 5 * c5});
      const auto ar = polynomialEnvelope<4>({2 * c2, 6 * c3, 12 * c4, 20 * c5});
      const auto jr = polynomialEnvelope<3>({6 * c3, 24 * c4, 60 * c5});
      // 为JTC双精度系数计算留出保守舍入界；短到数值病态的区间回退原轨迹。
      const long double error = 128.0L * std::numeric_limits<double>::epsilon() *
        (1.0L + std::abs(l.positions[j]) + std::abs(r.positions[j]) +
        std::abs(c1) + std::abs(c2) + std::abs(x) + std::abs(v) + std::abs(a));
      const double vmax = static_cast<double>((std::max(std::abs(vr[0]), std::abs(vr[1])) +
        20.0L * error) / dt);
      const double amax = static_cast<double>((std::max(std::abs(ar[0]), std::abs(ar[1])) +
        80.0L * error) / (dt * dt));
      const double jmax = static_cast<double>((std::max(std::abs(jr[0]), std::abs(jr[1])) +
        240.0L * error) / (dt * dt * dt));
      if (!std::isfinite(vmax) || !std::isfinite(amax) || !std::isfinite(jmax) ||
        !std::isfinite(qr[0]) || !std::isfinite(qr[1]) ||
        qr[0] - error<limits[j].lower || qr[1] + error> limits[j].upper ||
        vmax > limits[j].velocity || amax > limits[j].acceleration || jmax > limits[j].jerk)
      {
        return false;
      }
      report.velocity_ratio = std::max(report.velocity_ratio, vmax / limits[j].velocity);
      report.acceleration_ratio = std::max(report.acceleration_ratio,
          amax / limits[j].acceleration);
      report.jerk_ratio = std::max(report.jerk_ratio, jmax / limits[j].jerk);
    }
  }
  return true;
}

// 只接受有限、同关节直线、单调、静止到静止的PTP。调用方核实际模型/规划器。
// 成功前不修改输入；保留每一个原q，另插入全部Ruckig相位边界，不删原采样。
inline bool retimeJointLine(
  trajectory_msgs::msg::JointTrajectory & path,
  const std::vector<JointLineLimit> & model_limits, double velocity_scaling,
  double acceleration_scaling, JointLineRetimeReport & report)
{
  report = {};
  const auto reject = [&report](const char * reason) {report.reason = reason; return false;};
  const std::size_t n = path.joint_names.size();
  if (n == 0 || model_limits.size() != n || path.points.size() < 2 ||
    path.points.size() > 4096 || !std::isfinite(velocity_scaling) || velocity_scaling <= 0.0 ||
    velocity_scaling > 1.0 || !std::isfinite(acceleration_scaling) ||
    acceleration_scaling <= 0.0 || acceleration_scaling > 1.0)
  {
    return reject("unknown limits or shape");
  }
  for (std::size_t j = 0; j < n; ++j) {
    const auto & limit = model_limits[j];
    if (path.joint_names[j].empty() ||
      std::find(path.joint_names.begin(), path.joint_names.begin() + j, path.joint_names[j]) !=
      path.joint_names.begin() + j ||
      !std::isfinite(limit.lower) || !std::isfinite(limit.upper) || limit.lower >= limit.upper ||
      !std::isfinite(limit.velocity) || limit.velocity <= 0.0 ||
      !std::isfinite(limit.acceleration) || limit.acceleration <= 0.0 ||
      !std::isfinite(limit.jerk) || limit.jerk <= 0.0)
    {
      return reject("unknown joint model limits");
    }
  }
  int64_t previous_ns = -1;
  for (const auto & point : path.points) {
    const int64_t time_ns = jointLineTimeNs(point.time_from_start);
    if (point.time_from_start.nanosec >= 1000000000U || time_ns <= previous_ns ||
      point.positions.size() != n || point.velocities.size() != n ||
      point.accelerations.size() != n || !point.effort.empty())
    {
      return reject("invalid waypoint shape or time");
    }
    previous_ns = time_ns;
    for (std::size_t j = 0; j < n; ++j) {
      if (!std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j]) ||
        !std::isfinite(point.accelerations[j]) || point.positions[j] < model_limits[j].lower ||
        point.positions[j] > model_limits[j].upper)
      {
        return reject("nonfinite or out of bounds waypoint");
      }
    }
  }
  if (jointLineTimeNs(path.points.front().time_from_start) != 0) {
    return reject("nonzero start time");
  }
  std::vector<double> delta(n);
  std::size_t dominant = 0;
  for (std::size_t j = 0; j < n; ++j) {
    if (path.points.front().velocities[j] != 0.0 || path.points.back().velocities[j] != 0.0 ||
      path.points.front().accelerations[j] != 0.0 || path.points.back().accelerations[j] != 0.0)
    {
      return reject("nonzero endpoint derivatives");
    }
    delta[j] = path.points.back().positions[j] - path.points.front().positions[j];
    if (std::abs(delta[j]) > std::abs(delta[dominant])) {dominant = j;}
  }
  if (std::abs(delta[dominant]) < 1.0e-6) {return reject("negligible line");}
  std::vector<double> progress;
  double previous_s = -1.0;
  for (const auto & point : path.points) {
    const double s = (point.positions[dominant] - path.points.front().positions[dominant]) /
      delta[dominant];
    if (s < 0.0 || s > 1.0 || s <= previous_s) {return reject("nonmonotonic line");}
    for (std::size_t j = 0; j < n; ++j) {
      if (std::abs(point.positions[j] - path.points.front().positions[j] - s * delta[j]) >
        1.0e-10 ||
        std::abs(point.velocities[j] - point.velocities[dominant] * delta[j] / delta[dominant]) >
        1.0e-10 ||
        std::abs(point.accelerations[j] - point.accelerations[dominant] * delta[j] /
          delta[dominant]) >
        1.0e-10)
      {
        return reject("not a synchronized joint line");
      }
    }
    progress.push_back(s);
    previous_s = s;
  }
  double sv = std::numeric_limits<double>::infinity();
  double sa = sv, sj = sv;
  auto limits = model_limits;
  for (std::size_t j = 0; j < n; ++j) {
    limits[j].velocity *= velocity_scaling;
    limits[j].acceleration *= acceleration_scaling;
    if (std::abs(delta[j]) > 1.0e-12) {
      sv = std::min(sv, limits[j].velocity / std::abs(delta[j]));
      sa = std::min(sa, limits[j].acceleration / std::abs(delta[j]));
      sj = std::min(sj, limits[j].jerk / std::abs(delta[j]));
    }
  }
  // 至少0.1s爬升，不采用模型允许的极限jerk档。v/a留0.1%、jerk留1%
  // 余量：保留原q后，纳秒量化误差的三阶导数会被短区间放大。
  sj = std::min(sj, sa / 0.1);
  for (std::size_t j = 0; j < n; ++j) {
    if (std::abs(delta[j]) > 1.0e-12) {
      limits[j].jerk = std::min(limits[j].jerk, sj * std::abs(delta[j]));
    }
  }
  ruckig::InputParameter<1> input;
  input.current_position = {0.0}; input.current_velocity = {0.0};
  input.current_acceleration = {0.0};
  input.target_position = {1.0}; input.target_velocity = {0.0}; input.target_acceleration = {0.0};
  input.max_velocity = {sv * 0.999}; input.max_acceleration = {sa * 0.999};
  input.max_jerk = {sj * 0.99};
  ruckig::Ruckig<1> solver;
  ruckig::Trajectory<1> trajectory;
  if (solver.calculate(input, trajectory) < 0) {return reject("ruckig failed");}
  const double duration = trajectory.get_duration();
  report.before_s = previous_ns * 1.0e-9;
  report.after_s = duration;
  if (!std::isfinite(duration) || duration <= 0.0 || duration + 0.001 >= report.before_s) {
    return reject("no timing benefit");
  }
  const auto ns = [](double t) {return static_cast<int64_t>(std::llround(t * 1.0e9));};
  struct Sample {int64_t time_ns; int original;};
  std::vector<Sample> samples;
  for (std::size_t i = 0; i < progress.size(); ++i) {
    double lo = 0.0, hi = duration;
    for (unsigned k = 0; k < 64; ++k) {
      const double mid = (lo + hi) / 2.0;
      std::array<double, 1> p, v, a;
      trajectory.at_time(mid, p, v, a);
      if (p[0] < progress[i]) {lo = mid;} else {hi = mid;}
    }
    samples.push_back({ns(i == 0 ? 0.0 : (i + 1 == progress.size() ? duration : (lo + hi) / 2.0)),
        static_cast<int>(i)});
  }
  const auto profiles = trajectory.get_profiles();
  for (double t : profiles[0][0].t_sum) {
    samples.push_back({ns(t), -1});
  }
  std::sort(samples.begin(), samples.end(), [](const auto & a, const auto & b) {
      return a.time_ns < b.time_ns;
    });
  std::vector<Sample> distinct;
  for (const auto & sample : samples) {
    if (!distinct.empty() && sample.time_ns == distinct.back().time_ns) {
      if (sample.original >= 0) {
        if (distinct.back().original >= 0) {return reject("original timestamps collapse");}
        distinct.back() = sample;
      }
    } else {distinct.push_back(sample);}
  }
  auto candidate = path;
  candidate.points.clear();
  for (std::size_t k = 0; k < distinct.size(); ++k) {
    const auto & sample = distinct[k];
    std::array<double, 1> p, v, a;
    trajectory.at_time(sample.time_ns * 1.0e-9, p, v, a);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(n); point.velocities.resize(n); point.accelerations.resize(n);
    point.time_from_start.sec = static_cast<int32_t>(sample.time_ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(sample.time_ns % 1000000000LL);
    for (std::size_t j = 0; j < n; ++j) {
      point.positions[j] = sample.original >= 0 ? path.points[sample.original].positions[j] :
        path.points.front().positions[j] + delta[j] * p[0];
      point.velocities[j] = k == 0 || k + 1 == distinct.size() ? 0.0 : delta[j] * v[0];
      point.accelerations[j] = k == 0 || k + 1 == distinct.size() ? 0.0 : delta[j] * a[0];
    }
    candidate.points.push_back(std::move(point));
  }
  if (!jointLineQuinticWithinLimits(candidate, limits, report)) {
    return reject("quintic whole-interval bound failed");
  }
  path = std::move(candidate);
  report.validated_limits = std::move(limits);
  report.reason = "same joint line, original waypoints and phase boundaries retained";
  return true;
}

}  // namespace peach_arm

#endif  // PEACH_ARM__JOINT_LINE_RETIMING_HPP_
