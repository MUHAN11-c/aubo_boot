#include "aubo_ros2_trajectory_action.h"

using namespace aubo_ros2_trajectory_action;

UniformSampleFilter::UniformSampleFilter()
{
  sample_duration_ = DEFAULT_SAMPLE_DURATION;
  std::cout << "UniformSampleFilter Ready" << std::endl;
}

double UniformSampleFilter::toSec(const builtin_interfaces::msg::Duration &duration)
{
  return (double)duration.sec + 1e-9 * (double)duration.nanosec;
}

builtin_interfaces::msg::Duration UniformSampleFilter::toDuration(double time_in_seconds)
{
  builtin_interfaces::msg::Duration duration;

  duration.sec = static_cast<int32_t>(time_in_seconds);
  duration.nanosec = static_cast<uint32_t>((time_in_seconds - duration.sec) * 1e9);

  return duration;
}

void UniformSampleFilter::configure(const double &sample_duration)
{
  sample_duration_ = sample_duration;
}

bool UniformSampleFilter::update(const trajectory_msgs::msg::JointTrajectory &in, trajectory_msgs::msg::JointTrajectory &out)
{
  bool success = false;
  size_t size_in = in.points.size();
  if (size_in < 2)
  {
    out = in;
    return true;
  }
  
  double duration_in = toSec(in.points.back().time_from_start);
  double interpolated_time = 0.0;
  size_t index_in = 0;

  trajectory_msgs::msg::JointTrajectoryPoint p1, p2, interp_pt;

  out = in;
  out.points.clear();

  // 始终保留起点
  out.points.push_back(in.points[0]);
  
  // 从第一个间隔开始插值
  interpolated_time = sample_duration_;

  while (interpolated_time < duration_in)
  {
    // 找到包含 interpolated_time 的两个原始点
    while (index_in + 1 < size_in && 
           interpolated_time > toSec(in.points[index_in + 1].time_from_start))
    {
      index_in++;
    }

    if (index_in + 1 >= size_in)
    {
      break;
    }

    p1 = in.points[index_in];
    p2 = in.points[index_in + 1];

    if (!interpolatePt(p1, p2, interpolated_time, interp_pt))
    {
      return false;
    }
    out.points.push_back(interp_pt);
    interpolated_time += sample_duration_;
  }

  // 保留终点
  out.points.push_back(in.points.back());

  success = true;
  return success;
}

bool UniformSampleFilter::interpolatePt(trajectory_msgs::msg::JointTrajectoryPoint &p1, trajectory_msgs::msg::JointTrajectoryPoint &p2,
                                        double time_from_start, trajectory_msgs::msg::JointTrajectoryPoint &interp_pt)
{
  bool ret = false;
  double p1_time_from_start = toSec(p1.time_from_start);
  double p2_time_from_start = toSec(p2.time_from_start);

  if (time_from_start >= p1_time_from_start && time_from_start <= p2_time_from_start)
  {
    if (p1.positions.size() == p1.velocities.size() && p1.positions.size() == p1.accelerations.size())
    {
      if (p1.positions.size() == p2.positions.size() && p1.velocities.size() == p2.velocities.size() && p1.accelerations.size() == p2.accelerations.size())
      {
        interp_pt = p1;
        KDL::VelocityProfile_Spline spline_calc;

        for (size_t i = 0; i < p1.positions.size(); ++i)
        {
          double time_from_p1 = time_from_start - toSec(p1.time_from_start);
          double time_from_p1_to_p2 = p2_time_from_start - p1_time_from_start;

          spline_calc.SetProfileDuration(p1.positions[i], p1.velocities[i], p1.accelerations[i], p2.positions[i],
                                         p2.velocities[i], p2.accelerations[i], time_from_p1_to_p2);

          builtin_interfaces::msg::Duration time_from_start_dur = toDuration(time_from_start);

          interp_pt.time_from_start = time_from_start_dur;
          interp_pt.positions[i] = spline_calc.Pos(time_from_p1);
          interp_pt.velocities[i] = spline_calc.Vel(time_from_p1);
          interp_pt.accelerations[i] = spline_calc.Acc(time_from_p1);
        }
        ret = true;
      }
      else
      {
        ret = false;
      }
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }

  return ret;
}
