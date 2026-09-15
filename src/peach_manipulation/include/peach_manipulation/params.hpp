#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

// Jazzy 的 rclcpp 已移除 ParameterDescriptor 别名（只在 rcl_interfaces 存在），
// 本文件 declare_one 用到它，这里按旧名补回。
namespace rclcpp
{
using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
}

namespace peach_manipulation_node
{

/// 手写参数快照：成员名=参数键（组为嵌套结构体）；默认值与 config/peach_manipulation.yaml
/// 保持同步（yaml 为部署事实源，此为兜底默认）；键名冻结（决策 0017）。
struct Params
{
  uint64_t __stamp = 0;  ///< 变更戳（ParamListener::is_old 用）
  std::string photo_pose_named_target = "global_photo_pose";
  double photo_pose_joint_tolerance_rad = 0.05;
  double photo_pose_max_joint_vel_rad_s = 0.05;
  std::string harvest_stow_named_target = "harvest_stow";
  struct Frames
  {
    std::string base = "base_link";
    std::string tip = "tcp";
    std::string camera = "camera_depth_optical_frame";
    std::string tool = "tcp";
  };
  Frames frames;
  struct Moveit
  {
    std::string planning_group = "manipulator_e5";
    double planning_time_s = 1.5;
    int64_t planning_attempts = 1L;
    double velocity_scaling = 0.1;
    double acceleration_scaling = 0.1;
    double transit_velocity_scaling = 0.1;
    double transit_acceleration_scaling = 0.1;
    std::string pilz_pipeline = "pilz_industrial_motion_planner";
    std::string fallback_pipeline = "ompl";
    std::string mtc_free_space_pipeline = "pilz_industrial_motion_planner";
    std::string mtc_free_space_planner = "LIN";
    double mtc_cartesian_step_m = 0.005;
    double mtc_cartesian_min_fraction = 0.95;
    double mtc_cartesian_precision_m = 0.001;
    int64_t mtc_max_solutions = 5L;
    double mtc_approach_max_duration_s = 0.0;
    double mtc_approach_max_total_joint_travel_rad = 12.0;
    double mtc_approach_max_single_joint_travel_rad = 6.1;
    double mtc_approach_max_detour_ratio = 1.8;
    double mtc_approach_max_chord_deviation_m = 0.25;
    double mtc_approach_max_recede_m = 0.08;
    double mtc_approach_transit_max_detour_ratio = 1.8;
    double mtc_approach_transit_max_chord_deviation_m = 0.25;
    double mtc_approach_transit_max_recede_m = 0.08;
    double mtc_approach_max_tcp_rotation_deg = 110.0;
    double mtc_approach_tcp_rotation_slack_deg = 20.0;
    double mtc_approach_keepout_radius_m = 0.12;
    double mtc_approach_keepout_axial_m = 0.12;
    double mtc_approach_cartesian_max_distance_m = 0.8;
    double mtc_approach_along_axis_m = 0.0;
    double approach_staging_standoff_m = 0.1;
    double approach_near_velocity_scaling = 0.05;
    double mtc_approach_max_lateral_m = 0.05;
    double mtc_approach_max_align_deg = 20.0;
    double observe_planning_time_s = 1.0;
    int64_t observe_planning_attempts = 1L;
    double observe_max_duration_s = 0.0;
    double observe_max_total_joint_travel_rad = 4.0;
    double observe_max_single_joint_travel_rad = 1.5;
    double photo_planning_time_s = 3.0;
    double photo_ptp_planning_time_s = 0.5;
    double transit_max_duration_s = 0.0;
    double transit_max_total_joint_travel_rad = 6.0;
    double transit_max_single_joint_travel_rad = 2.5;
  };
  Moveit moveit;
  struct Scan
  {
    double observation_radius_m = 0.4;
    double minimum_radius_m = 0.32;
    double azimuth_step_deg = 12.0;
    double azimuth_limit_deg = 16.0;
    double elevation_step_deg = 8.0;
    double elevation_limit_deg = 0.0;
    double preferred_baseline_deg = 12.0;
    double radial_step_m = 0.015;
    int64_t candidate_layers = 1L;
    int64_t views_to_minimum_radius = 5L;
    double max_camera_step_m = 0.15;
    double workspace_max_reach_m = 0.78;
    int64_t maximum_moves = 2L;
    int64_t min_effective_views = 1L;
    double time_budget_s = 15.0;
    double assumed_frame_interval_s = 0.4;
    double min_camera_height_m = 0.06;
    std::vector<double> protected_zones = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double frame_wait_s = 4.0;
  };
  Scan scan;
  struct Quality
  {
    int64_t minimum_views = 2L;
    double minimum_baseline_deg = 8.0;
    double minimum_mean_nearest_baseline_deg = 6.0;
    double minimum_mean_depth_ratio = 0.4;
    double maximum_data_age_s = 3.0;
    double maximum_axis_angle_deg = 35.0;
  };
  Quality quality;
  struct Execution
  {
    bool enabled = false;
    bool require_robot_status = true;
    double robot_status_max_age_s = 1.0;
    double target_observation_max_age_s = 3.0;
  };
  Execution execution;
  struct Grasp
  {
    bool enabled = false;
    double neck_margin_m = 0.015;
    double minimum_travel_m = 0.02;
    double maximum_travel_m = 0.2;
    double reconfirm_wait_s = 6.0;
    double reconfirm_tolerance_m = 0.03;
    int64_t reconfirm_max_attempts = 3L;
    bool allow_stale_anchor = false;
    // ①层果实胶囊：感知直径 + 本膨胀（米）。感知直径无效时回退
    // moveit.mtc_approach_keepout_radius_m 作 fruitRadiusM 的 base。
    double fruit_inflation_m = 0.01;
    // ④层接触止损（默认关）：关节电流特征判别，阈值须真机受控试验
    // 标定后启用（SDK 原始单位）；<=0 不判该项。
    struct ContactDetect
    {
      bool enabled = false;
      double baseline_s = 0.2;
      double slope_threshold = 0.0;
      double spike_threshold = 0.0;
    };
    ContactDetect contact_detect;
  };
  Grasp grasp;
  struct Tool
  {
    bool enabled = false;
    int64_t io_fun = 3L;
    int64_t io_pin = 0L;
    double close_state = 1.0;
    // 当前末端工具档案标签（消息 tool_profile_id；基础值=固定圆柱，
    // 整栈由 launch tool_profile 档案注入覆盖）
    std::string profile_id = "hollow_cylinder_v1";
  };
  Tool tool;
  struct Timeouts
  {
    double service_s = 3.0;
    double refined_s = 30.0;
  };
  Timeouts timeouts;
};

/// 手写参数监听器：构造即声明+启动校验（非法覆盖值抛异常终止启动），on-set
/// 校验拒绝非法值并原子刷新快照；接口与原 GPL 生成物一致（get_params/is_old）。
class ParamListener
{
public:
  ParamListener(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
    rclcpp::Logger logger)
  : parameters_interface_(std::move(parameters_interface)), logger_(logger)
  {
    declare_and_validate();
    handle_ = parameters_interface_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_set(params);
      });
  }

  Params get_params() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Params out = params_;
    out.__stamp = stamp_;
    return out;
  }

  bool is_old(const Params & other) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return stamp_ != other.__stamp;
  }

private:
  void declare_and_validate()
  {
    declare_one<std::string>("frames.base", "base_link");
    declare_one<std::string>("frames.tip", "tcp");
    declare_one<std::string>("frames.camera", "camera_depth_optical_frame");
    declare_one<std::string>("frames.tool", "tcp");
    declare_one<std::string>("moveit.planning_group", "manipulator_e5");
    declare_one<double>("moveit.planning_time_s", 1.5);
    declare_one<int64_t>("moveit.planning_attempts", 1L);
    declare_one<double>("moveit.velocity_scaling", 0.1);
    declare_one<double>("moveit.acceleration_scaling", 0.1);
    declare_one<double>("moveit.transit_velocity_scaling", 0.1);
    declare_one<double>("moveit.transit_acceleration_scaling", 0.1);
    declare_one<std::string>("moveit.pilz_pipeline", "pilz_industrial_motion_planner");
    declare_one<std::string>("moveit.fallback_pipeline", "ompl");
    declare_one<std::string>("moveit.mtc_free_space_pipeline", "pilz_industrial_motion_planner");
    declare_one<std::string>("moveit.mtc_free_space_planner", "LIN");
    declare_one<double>("moveit.mtc_cartesian_step_m", 0.005);
    declare_one<double>("moveit.mtc_cartesian_min_fraction", 0.95);
    declare_one<double>("moveit.mtc_cartesian_precision_m", 0.001);
    declare_one<int64_t>("moveit.mtc_max_solutions", 5L);
    declare_one<double>("moveit.mtc_approach_max_duration_s", 0.0);
    declare_one<double>("moveit.mtc_approach_max_total_joint_travel_rad", 12.0);
    declare_one<double>("moveit.mtc_approach_max_single_joint_travel_rad", 6.1);
    declare_one<double>("moveit.mtc_approach_max_detour_ratio", 1.8);
    declare_one<double>("moveit.mtc_approach_max_chord_deviation_m", 0.25);
    declare_one<double>("moveit.mtc_approach_max_recede_m", 0.08);
    declare_one<double>("moveit.mtc_approach_transit_max_detour_ratio", 1.8);
    declare_one<double>(
      "moveit.mtc_approach_transit_max_chord_deviation_m", 0.25);
    declare_one<double>("moveit.mtc_approach_transit_max_recede_m", 0.08);
    declare_one<double>("moveit.mtc_approach_max_tcp_rotation_deg", 110.0);
    declare_one<double>("moveit.mtc_approach_tcp_rotation_slack_deg", 20.0);
    declare_one<double>("moveit.mtc_approach_keepout_radius_m", 0.12);
    declare_one<double>("moveit.mtc_approach_keepout_axial_m", 0.12);
    declare_one<double>("moveit.mtc_approach_cartesian_max_distance_m", 0.8);
    declare_one<double>("moveit.mtc_approach_along_axis_m", 0.0);
    declare_one<double>("moveit.approach_staging_standoff_m", 0.1);
    declare_one<double>("moveit.approach_near_velocity_scaling", 0.05);
    declare_one<double>("moveit.mtc_approach_max_lateral_m", 0.05);
    declare_one<double>("moveit.mtc_approach_max_align_deg", 20.0);
    declare_one<double>("moveit.observe_planning_time_s", 1.0);
    declare_one<int64_t>("moveit.observe_planning_attempts", 1L);
    declare_one<double>("moveit.observe_max_duration_s", 0.0);
    declare_one<double>("moveit.observe_max_total_joint_travel_rad", 4.0);
    declare_one<double>("moveit.observe_max_single_joint_travel_rad", 1.5);
    declare_one<double>("moveit.photo_planning_time_s", 3.0);
    declare_one<double>("moveit.photo_ptp_planning_time_s", 0.5);
    declare_one<double>("moveit.transit_max_duration_s", 0.0);
    declare_one<double>("moveit.transit_max_total_joint_travel_rad", 6.0);
    declare_one<double>("moveit.transit_max_single_joint_travel_rad", 2.5);
    declare_one<std::string>("photo_pose_named_target", "global_photo_pose");
    declare_one<double>("photo_pose_joint_tolerance_rad", 0.05);
    declare_one<double>("photo_pose_max_joint_vel_rad_s", 0.05);
    declare_one<std::string>("harvest_stow_named_target", "harvest_stow");
    declare_one<double>("scan.observation_radius_m", 0.4);
    declare_one<double>("scan.minimum_radius_m", 0.32);
    declare_one<double>("scan.azimuth_step_deg", 12.0);
    declare_one<double>("scan.azimuth_limit_deg", 16.0);
    declare_one<double>("scan.elevation_step_deg", 8.0);
    declare_one<double>("scan.elevation_limit_deg", 0.0);
    declare_one<double>("scan.preferred_baseline_deg", 12.0);
    declare_one<double>("scan.radial_step_m", 0.015);
    declare_one<int64_t>("scan.candidate_layers", 1L);
    declare_one<int64_t>("scan.views_to_minimum_radius", 5L);
    declare_one<double>("scan.max_camera_step_m", 0.15);
    declare_one<double>("scan.workspace_max_reach_m", 0.78);
    declare_one<int64_t>("scan.maximum_moves", 2L);
    declare_one<int64_t>("scan.min_effective_views", 1L);
    declare_one<double>("scan.time_budget_s", 15.0);
    declare_one<double>("scan.assumed_frame_interval_s", 0.4);
    declare_one<double>("scan.min_camera_height_m", 0.06);
    declare_one<std::vector<double>>("scan.protected_zones", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_one<double>("scan.frame_wait_s", 4.0);
    declare_one<int64_t>("quality.minimum_views", 2L);
    declare_one<double>("quality.minimum_baseline_deg", 8.0);
    declare_one<double>("quality.minimum_mean_nearest_baseline_deg", 6.0);
    declare_one<double>("quality.minimum_mean_depth_ratio", 0.4);
    declare_one<double>("quality.maximum_data_age_s", 3.0);
    declare_one<double>("quality.maximum_axis_angle_deg", 35.0);
    declare_one<bool>("execution.enabled", false);
    declare_one<bool>("execution.require_robot_status", true);
    declare_one<double>("execution.robot_status_max_age_s", 1.0);
    declare_one<double>("execution.target_observation_max_age_s", 3.0);
    declare_one<bool>("grasp.enabled", false);
    declare_one<double>("grasp.neck_margin_m", 0.015);
    declare_one<double>("grasp.minimum_travel_m", 0.02);
    declare_one<double>("grasp.maximum_travel_m", 0.2);
    declare_one<double>("grasp.reconfirm_wait_s", 6.0);
    declare_one<double>("grasp.reconfirm_tolerance_m", 0.03);
    declare_one<int64_t>("grasp.reconfirm_max_attempts", 3L);
    declare_one<bool>("grasp.allow_stale_anchor", false);
    declare_one<double>("grasp.fruit_inflation_m", 0.01);
    declare_one<bool>("grasp.contact_detect.enabled", false);
    declare_one<double>("grasp.contact_detect.baseline_s", 0.2);
    declare_one<double>("grasp.contact_detect.slope_threshold", 0.0);
    declare_one<double>("grasp.contact_detect.spike_threshold", 0.0);
    declare_one<bool>("tool.enabled", false);
    declare_one<int64_t>("tool.io_fun", 3L);
    declare_one<int64_t>("tool.io_pin", 0L);
    declare_one<double>("tool.close_state", 1.0);
    declare_one<std::string>("tool.profile_id", "hollow_cylinder_v1");
    declare_one<double>("timeouts.service_s", 3.0);
    declare_one<double>("timeouts.refined_s", 30.0);
  }

  template<typename T>
  void declare_one(const std::string & name, const T & value)
  {
    parameters_interface_->declare_parameter(
      name, rclcpp::ParameterValue(value), rclcpp::ParameterDescriptor(), false);
    rclcpp::Parameter declared;
    parameters_interface_->get_parameter(name, declared);
    std::string reason;
    if (!validate_one(name, declared, reason)) {
      throw rclcpp::exceptions::InvalidParameterValueException(reason);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    assign(name, declared);
  }

  bool validate_one(
    const std::string & name, const rclcpp::Parameter & param,
    std::string & reason) const
  {
    (void)logger_;
    if (name == "moveit.planning_time_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.planning_time_s: 须 > 0.0"; return false;}
    } else if (name == "moveit.planning_attempts") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "moveit.planning_attempts: 须 >= 1.0"; return false;}
    } else if (name == "moveit.velocity_scaling") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.velocity_scaling: 须 > 0.0"; return false;}
      if (!(v <= 1.0)) {reason = "moveit.velocity_scaling: 须 <= 1.0"; return false;}
    } else if (name == "moveit.acceleration_scaling") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.acceleration_scaling: 须 > 0.0"; return false;}
      if (!(v <= 1.0)) {reason = "moveit.acceleration_scaling: 须 <= 1.0"; return false;}
    } else if (name == "moveit.transit_velocity_scaling") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.transit_velocity_scaling: 须 > 0.0"; return false;}
      if (!(v <= 1.0)) {reason = "moveit.transit_velocity_scaling: 须 <= 1.0"; return false;}
    } else if (name == "moveit.transit_acceleration_scaling") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.transit_acceleration_scaling: 须 > 0.0"; return false;}
      if (!(v <= 1.0)) {reason = "moveit.transit_acceleration_scaling: 须 <= 1.0"; return false;}
    } else if (name == "moveit.mtc_cartesian_step_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.mtc_cartesian_step_m: 须 > 0.0"; return false;}
    } else if (name == "moveit.mtc_cartesian_min_fraction") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.mtc_cartesian_min_fraction: 须 > 0.0"; return false;}
      if (!(v <= 1.0)) {reason = "moveit.mtc_cartesian_min_fraction: 须 <= 1.0"; return false;}
    } else if (name == "moveit.mtc_cartesian_precision_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.mtc_cartesian_precision_m: 须 > 0.0"; return false;}
    } else if (name == "moveit.mtc_max_solutions") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "moveit.mtc_max_solutions: 须 >= 1.0"; return false;}
    } else if (name == "moveit.mtc_approach_max_duration_s") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "moveit.mtc_approach_max_duration_s: 须 >= 0.0"; return false;}
    } else if (name == "moveit.mtc_approach_max_total_joint_travel_rad") {
      const double v = param.as_double();
      if (!(v > 0.0)) {
        reason = "moveit.mtc_approach_max_total_joint_travel_rad: 须 > 0.0"; return false;
      }
    } else if (name == "moveit.mtc_approach_max_single_joint_travel_rad") {
      const double v = param.as_double();
      if (!(v > 0.0)) {
        reason = "moveit.mtc_approach_max_single_joint_travel_rad: 须 > 0.0"; return false;
      }
    } else if (name == "moveit.mtc_approach_max_detour_ratio") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "moveit.mtc_approach_max_detour_ratio: 须 >= 0.0"; return false;}
    } else if (name == "moveit.mtc_approach_max_chord_deviation_m") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {
        reason = "moveit.mtc_approach_max_chord_deviation_m: 须 >= 0.0"; return false;
      }
    } else if (name == "moveit.mtc_approach_max_recede_m") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "moveit.mtc_approach_max_recede_m: 须 >= 0.0"; return false;}
    } else if (name == "moveit.mtc_approach_max_tcp_rotation_deg") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {
        reason = "moveit.mtc_approach_max_tcp_rotation_deg: 须 >= 0.0"; return false;
      }
    } else if (name == "moveit.mtc_approach_tcp_rotation_slack_deg") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {
        reason = "moveit.mtc_approach_tcp_rotation_slack_deg: 须 >= 0.0"; return false;
      }
    } else if (name == "moveit.mtc_approach_keepout_radius_m") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {
        reason = "moveit.mtc_approach_keepout_radius_m: 须 >= 0.0"; return false;
      }
    } else if (name == "moveit.mtc_approach_keepout_axial_m") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {
        reason = "moveit.mtc_approach_keepout_axial_m: 须 >= 0.0"; return false;
      }
    } else if (name == "moveit.mtc_approach_cartesian_max_distance_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {
        reason = "moveit.mtc_approach_cartesian_max_distance_m: 须 > 0.0"; return false;
      }
    } else if (name == "moveit.mtc_approach_along_axis_m") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "moveit.mtc_approach_along_axis_m: 须 >= 0.0"; return false;}
    } else if (name == "moveit.approach_staging_standoff_m") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "moveit.approach_staging_standoff_m: 须 >= 0.0"; return false;}
    } else if (name == "moveit.approach_near_velocity_scaling") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.approach_near_velocity_scaling: 须 > 0.0"; return false;}
      if (!(v <= 1.0)) {reason = "moveit.approach_near_velocity_scaling: 须 <= 1.0"; return false;}
    } else if (name == "moveit.mtc_approach_max_lateral_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.mtc_approach_max_lateral_m: 须 > 0.0"; return false;}
    } else if (name == "moveit.mtc_approach_max_align_deg") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.mtc_approach_max_align_deg: 须 > 0.0"; return false;}
    } else if (name == "moveit.observe_planning_time_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.observe_planning_time_s: 须 > 0.0"; return false;}
    } else if (name == "moveit.observe_planning_attempts") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "moveit.observe_planning_attempts: 须 >= 1.0"; return false;}
    } else if (name == "moveit.observe_max_duration_s") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "moveit.observe_max_duration_s: 须 >= 0.0"; return false;}
    } else if (name == "moveit.observe_max_total_joint_travel_rad") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.observe_max_total_joint_travel_rad: 须 > 0.0"; return false;}
    } else if (name == "moveit.observe_max_single_joint_travel_rad") {
      const double v = param.as_double();
      if (!(v > 0.0)) {
        reason = "moveit.observe_max_single_joint_travel_rad: 须 > 0.0"; return false;
      }
    } else if (name == "moveit.photo_planning_time_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.photo_planning_time_s: 须 > 0.0"; return false;}
    } else if (name == "moveit.photo_ptp_planning_time_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.photo_ptp_planning_time_s: 须 > 0.0"; return false;}
    } else if (name == "moveit.transit_max_duration_s") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "moveit.transit_max_duration_s: 须 >= 0.0"; return false;}
    } else if (name == "moveit.transit_max_total_joint_travel_rad") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "moveit.transit_max_total_joint_travel_rad: 须 > 0.0"; return false;}
    } else if (name == "moveit.transit_max_single_joint_travel_rad") {
      const double v = param.as_double();
      if (!(v > 0.0)) {
        reason = "moveit.transit_max_single_joint_travel_rad: 须 > 0.0"; return false;
      }
    } else if (name == "photo_pose_joint_tolerance_rad") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "photo_pose_joint_tolerance_rad: 须 > 0.0"; return false;}
    } else if (name == "photo_pose_max_joint_vel_rad_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "photo_pose_max_joint_vel_rad_s: 须 > 0.0"; return false;}
    } else if (name == "scan.observation_radius_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.observation_radius_m: 须 > 0.0"; return false;}
    } else if (name == "scan.minimum_radius_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.minimum_radius_m: 须 > 0.0"; return false;}
    } else if (name == "scan.azimuth_step_deg") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.azimuth_step_deg: 须 > 0.0"; return false;}
    } else if (name == "scan.azimuth_limit_deg") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "scan.azimuth_limit_deg: 须 >= 0.0"; return false;}
    } else if (name == "scan.elevation_step_deg") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.elevation_step_deg: 须 > 0.0"; return false;}
    } else if (name == "scan.elevation_limit_deg") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "scan.elevation_limit_deg: 须 >= 0.0"; return false;}
    } else if (name == "scan.preferred_baseline_deg") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "scan.preferred_baseline_deg: 须 >= 0.0"; return false;}
    } else if (name == "scan.radial_step_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.radial_step_m: 须 > 0.0"; return false;}
    } else if (name == "scan.candidate_layers") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "scan.candidate_layers: 须 >= 1.0"; return false;}
    } else if (name == "scan.views_to_minimum_radius") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "scan.views_to_minimum_radius: 须 >= 1.0"; return false;}
    } else if (name == "scan.max_camera_step_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.max_camera_step_m: 须 > 0.0"; return false;}
    } else if (name == "scan.workspace_max_reach_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.workspace_max_reach_m: 须 > 0.0"; return false;}
    } else if (name == "scan.maximum_moves") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "scan.maximum_moves: 须 >= 1.0"; return false;}
    } else if (name == "scan.min_effective_views") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "scan.min_effective_views: 须 >= 1.0"; return false;}
    } else if (name == "scan.time_budget_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.time_budget_s: 须 > 0.0"; return false;}
    } else if (name == "scan.assumed_frame_interval_s") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "scan.assumed_frame_interval_s: 须 >= 0.0"; return false;}
    } else if (name == "scan.min_camera_height_m") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "scan.min_camera_height_m: 须 >= 0.0"; return false;}
    } else if (name == "scan.frame_wait_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "scan.frame_wait_s: 须 > 0.0"; return false;}
    } else if (name == "quality.minimum_views") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "quality.minimum_views: 须 >= 1.0"; return false;}
    } else if (name == "quality.minimum_baseline_deg") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "quality.minimum_baseline_deg: 须 >= 0.0"; return false;}
    } else if (name == "quality.minimum_mean_nearest_baseline_deg") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {
        reason = "quality.minimum_mean_nearest_baseline_deg: 须 >= 0.0"; return false;
      }
    } else if (name == "quality.minimum_mean_depth_ratio") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "quality.minimum_mean_depth_ratio: 须 >= 0.0"; return false;}
      if (!(v <= 1.0)) {reason = "quality.minimum_mean_depth_ratio: 须 <= 1.0"; return false;}
    } else if (name == "quality.maximum_data_age_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "quality.maximum_data_age_s: 须 > 0.0"; return false;}
    } else if (name == "quality.maximum_axis_angle_deg") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "quality.maximum_axis_angle_deg: 须 > 0.0"; return false;}
      if (!(v <= 180.0)) {reason = "quality.maximum_axis_angle_deg: 须 <= 180.0"; return false;}
    } else if (name == "execution.robot_status_max_age_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "execution.robot_status_max_age_s: 须 > 0.0"; return false;}
    } else if (name == "execution.target_observation_max_age_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "execution.target_observation_max_age_s: 须 > 0.0"; return false;}
    } else if (name == "grasp.fruit_inflation_m" ||
      name == "grasp.contact_detect.baseline_s" ||
      name == "grasp.contact_detect.slope_threshold" ||
      name == "grasp.contact_detect.spike_threshold")
    {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "grasp 果实/接触检测: 须 >= 0.0"; return false;}
    } else if (name == "grasp.neck_margin_m") {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "grasp.neck_margin_m: 须 >= 0.0"; return false;}
    } else if (name == "grasp.minimum_travel_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "grasp.minimum_travel_m: 须 > 0.0"; return false;}
    } else if (name == "grasp.maximum_travel_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "grasp.maximum_travel_m: 须 > 0.0"; return false;}
    } else if (name == "grasp.reconfirm_wait_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "grasp.reconfirm_wait_s: 须 > 0.0"; return false;}
    } else if (name == "grasp.reconfirm_tolerance_m") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "grasp.reconfirm_tolerance_m: 须 > 0.0"; return false;}
    } else if (name == "grasp.reconfirm_max_attempts") {
      const int64_t v = param.as_int();
      if (!(v >= 1.0)) {reason = "grasp.reconfirm_max_attempts: 须 >= 1.0"; return false;}
    } else if (name == "tool.io_fun") {
      const int64_t v = param.as_int();
      if (!(v >= 0.0)) {reason = "tool.io_fun: 须 >= 0.0"; return false;}
    } else if (name == "tool.io_pin") {
      const int64_t v = param.as_int();
      if (!(v >= 0.0)) {reason = "tool.io_pin: 须 >= 0.0"; return false;}
    } else if (name == "timeouts.service_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "timeouts.service_s: 须 > 0.0"; return false;}
    } else if (name == "timeouts.refined_s") {
      const double v = param.as_double();
      if (!(v > 0.0)) {reason = "timeouts.refined_s: 须 > 0.0"; return false;}
    } else if (name == "moveit.mtc_approach_max_recede_m" ||
      name == "moveit.mtc_approach_transit_max_detour_ratio" ||
      name == "moveit.mtc_approach_transit_max_chord_deviation_m" ||
      name == "moveit.mtc_approach_transit_max_recede_m" ||
      name == "moveit.mtc_approach_max_tcp_rotation_deg" ||
      name == "moveit.mtc_approach_tcp_rotation_slack_deg")
    {
      const double v = param.as_double();
      if (!(v >= 0.0)) {reason = "staging 护栏: 须 >= 0.0"; return false;}
    }
    return true;
  }

  void assign(const std::string & name, const rclcpp::Parameter & param)
  {
    if (name == "frames.base") {
      params_.frames.base = param.as_string();
    } else if (name == "frames.tip") {
      params_.frames.tip = param.as_string();
    } else if (name == "frames.camera") {
      params_.frames.camera = param.as_string();
    } else if (name == "frames.tool") {
      params_.frames.tool = param.as_string();
    } else if (name == "moveit.planning_group") {
      params_.moveit.planning_group = param.as_string();
    } else if (name == "moveit.planning_time_s") {
      params_.moveit.planning_time_s = param.as_double();
    } else if (name == "moveit.planning_attempts") {
      params_.moveit.planning_attempts = param.as_int();
    } else if (name == "moveit.velocity_scaling") {
      params_.moveit.velocity_scaling = param.as_double();
    } else if (name == "moveit.acceleration_scaling") {
      params_.moveit.acceleration_scaling = param.as_double();
    } else if (name == "moveit.transit_velocity_scaling") {
      params_.moveit.transit_velocity_scaling = param.as_double();
    } else if (name == "moveit.transit_acceleration_scaling") {
      params_.moveit.transit_acceleration_scaling = param.as_double();
    } else if (name == "moveit.pilz_pipeline") {
      params_.moveit.pilz_pipeline = param.as_string();
    } else if (name == "moveit.fallback_pipeline") {
      params_.moveit.fallback_pipeline = param.as_string();
    } else if (name == "moveit.mtc_free_space_pipeline") {
      params_.moveit.mtc_free_space_pipeline = param.as_string();
    } else if (name == "moveit.mtc_free_space_planner") {
      params_.moveit.mtc_free_space_planner = param.as_string();
    } else if (name == "moveit.mtc_cartesian_step_m") {
      params_.moveit.mtc_cartesian_step_m = param.as_double();
    } else if (name == "moveit.mtc_cartesian_min_fraction") {
      params_.moveit.mtc_cartesian_min_fraction = param.as_double();
    } else if (name == "moveit.mtc_cartesian_precision_m") {
      params_.moveit.mtc_cartesian_precision_m = param.as_double();
    } else if (name == "moveit.mtc_max_solutions") {
      params_.moveit.mtc_max_solutions = param.as_int();
    } else if (name == "moveit.mtc_approach_max_duration_s") {
      params_.moveit.mtc_approach_max_duration_s = param.as_double();
    } else if (name == "moveit.mtc_approach_max_total_joint_travel_rad") {
      params_.moveit.mtc_approach_max_total_joint_travel_rad = param.as_double();
    } else if (name == "moveit.mtc_approach_max_single_joint_travel_rad") {
      params_.moveit.mtc_approach_max_single_joint_travel_rad = param.as_double();
    } else if (name == "moveit.mtc_approach_max_detour_ratio") {
      params_.moveit.mtc_approach_max_detour_ratio = param.as_double();
    } else if (name == "moveit.mtc_approach_max_chord_deviation_m") {
      params_.moveit.mtc_approach_max_chord_deviation_m = param.as_double();
    } else if (name == "moveit.mtc_approach_max_recede_m") {
      params_.moveit.mtc_approach_max_recede_m = param.as_double();
    } else if (name == "moveit.mtc_approach_transit_max_detour_ratio") {
      params_.moveit.mtc_approach_transit_max_detour_ratio = param.as_double();
    } else if (name == "moveit.mtc_approach_transit_max_chord_deviation_m") {
      params_.moveit.mtc_approach_transit_max_chord_deviation_m =
        param.as_double();
    } else if (name == "moveit.mtc_approach_transit_max_recede_m") {
      params_.moveit.mtc_approach_transit_max_recede_m = param.as_double();
    } else if (name == "moveit.mtc_approach_max_tcp_rotation_deg") {
      params_.moveit.mtc_approach_max_tcp_rotation_deg = param.as_double();
    } else if (name == "moveit.mtc_approach_tcp_rotation_slack_deg") {
      params_.moveit.mtc_approach_tcp_rotation_slack_deg = param.as_double();
    } else if (name == "moveit.mtc_approach_keepout_radius_m") {
      params_.moveit.mtc_approach_keepout_radius_m = param.as_double();
    } else if (name == "moveit.mtc_approach_keepout_axial_m") {
      params_.moveit.mtc_approach_keepout_axial_m = param.as_double();
    } else if (name == "moveit.mtc_approach_cartesian_max_distance_m") {
      params_.moveit.mtc_approach_cartesian_max_distance_m = param.as_double();
    } else if (name == "moveit.mtc_approach_along_axis_m") {
      params_.moveit.mtc_approach_along_axis_m = param.as_double();
    } else if (name == "moveit.approach_staging_standoff_m") {
      params_.moveit.approach_staging_standoff_m = param.as_double();
    } else if (name == "moveit.approach_near_velocity_scaling") {
      params_.moveit.approach_near_velocity_scaling = param.as_double();
    } else if (name == "moveit.mtc_approach_max_lateral_m") {
      params_.moveit.mtc_approach_max_lateral_m = param.as_double();
    } else if (name == "moveit.mtc_approach_max_align_deg") {
      params_.moveit.mtc_approach_max_align_deg = param.as_double();
    } else if (name == "moveit.observe_planning_time_s") {
      params_.moveit.observe_planning_time_s = param.as_double();
    } else if (name == "moveit.observe_planning_attempts") {
      params_.moveit.observe_planning_attempts = param.as_int();
    } else if (name == "moveit.observe_max_duration_s") {
      params_.moveit.observe_max_duration_s = param.as_double();
    } else if (name == "moveit.observe_max_total_joint_travel_rad") {
      params_.moveit.observe_max_total_joint_travel_rad = param.as_double();
    } else if (name == "moveit.observe_max_single_joint_travel_rad") {
      params_.moveit.observe_max_single_joint_travel_rad = param.as_double();
    } else if (name == "moveit.photo_planning_time_s") {
      params_.moveit.photo_planning_time_s = param.as_double();
    } else if (name == "moveit.photo_ptp_planning_time_s") {
      params_.moveit.photo_ptp_planning_time_s = param.as_double();
    } else if (name == "moveit.transit_max_duration_s") {
      params_.moveit.transit_max_duration_s = param.as_double();
    } else if (name == "moveit.transit_max_total_joint_travel_rad") {
      params_.moveit.transit_max_total_joint_travel_rad = param.as_double();
    } else if (name == "moveit.transit_max_single_joint_travel_rad") {
      params_.moveit.transit_max_single_joint_travel_rad = param.as_double();
    } else if (name == "photo_pose_named_target") {
      params_.photo_pose_named_target = param.as_string();
    } else if (name == "photo_pose_joint_tolerance_rad") {
      params_.photo_pose_joint_tolerance_rad = param.as_double();
    } else if (name == "photo_pose_max_joint_vel_rad_s") {
      params_.photo_pose_max_joint_vel_rad_s = param.as_double();
    } else if (name == "harvest_stow_named_target") {
      params_.harvest_stow_named_target = param.as_string();
    } else if (name == "scan.observation_radius_m") {
      params_.scan.observation_radius_m = param.as_double();
    } else if (name == "scan.minimum_radius_m") {
      params_.scan.minimum_radius_m = param.as_double();
    } else if (name == "scan.azimuth_step_deg") {
      params_.scan.azimuth_step_deg = param.as_double();
    } else if (name == "scan.azimuth_limit_deg") {
      params_.scan.azimuth_limit_deg = param.as_double();
    } else if (name == "scan.elevation_step_deg") {
      params_.scan.elevation_step_deg = param.as_double();
    } else if (name == "scan.elevation_limit_deg") {
      params_.scan.elevation_limit_deg = param.as_double();
    } else if (name == "scan.preferred_baseline_deg") {
      params_.scan.preferred_baseline_deg = param.as_double();
    } else if (name == "scan.radial_step_m") {
      params_.scan.radial_step_m = param.as_double();
    } else if (name == "scan.candidate_layers") {
      params_.scan.candidate_layers = param.as_int();
    } else if (name == "scan.views_to_minimum_radius") {
      params_.scan.views_to_minimum_radius = param.as_int();
    } else if (name == "scan.max_camera_step_m") {
      params_.scan.max_camera_step_m = param.as_double();
    } else if (name == "scan.workspace_max_reach_m") {
      params_.scan.workspace_max_reach_m = param.as_double();
    } else if (name == "scan.maximum_moves") {
      params_.scan.maximum_moves = param.as_int();
    } else if (name == "scan.min_effective_views") {
      params_.scan.min_effective_views = param.as_int();
    } else if (name == "scan.time_budget_s") {
      params_.scan.time_budget_s = param.as_double();
    } else if (name == "scan.assumed_frame_interval_s") {
      params_.scan.assumed_frame_interval_s = param.as_double();
    } else if (name == "scan.min_camera_height_m") {
      params_.scan.min_camera_height_m = param.as_double();
    } else if (name == "scan.protected_zones") {
      params_.scan.protected_zones = param.as_double_array();
    } else if (name == "scan.frame_wait_s") {
      params_.scan.frame_wait_s = param.as_double();
    } else if (name == "quality.minimum_views") {
      params_.quality.minimum_views = param.as_int();
    } else if (name == "quality.minimum_baseline_deg") {
      params_.quality.minimum_baseline_deg = param.as_double();
    } else if (name == "quality.minimum_mean_nearest_baseline_deg") {
      params_.quality.minimum_mean_nearest_baseline_deg = param.as_double();
    } else if (name == "quality.minimum_mean_depth_ratio") {
      params_.quality.minimum_mean_depth_ratio = param.as_double();
    } else if (name == "quality.maximum_data_age_s") {
      params_.quality.maximum_data_age_s = param.as_double();
    } else if (name == "quality.maximum_axis_angle_deg") {
      params_.quality.maximum_axis_angle_deg = param.as_double();
    } else if (name == "execution.enabled") {
      params_.execution.enabled = param.as_bool();
    } else if (name == "execution.require_robot_status") {
      params_.execution.require_robot_status = param.as_bool();
    } else if (name == "execution.robot_status_max_age_s") {
      params_.execution.robot_status_max_age_s = param.as_double();
    } else if (name == "execution.target_observation_max_age_s") {
      params_.execution.target_observation_max_age_s = param.as_double();
    } else if (name == "grasp.enabled") {
      params_.grasp.enabled = param.as_bool();
    } else if (name == "grasp.neck_margin_m") {
      params_.grasp.neck_margin_m = param.as_double();
    } else if (name == "grasp.minimum_travel_m") {
      params_.grasp.minimum_travel_m = param.as_double();
    } else if (name == "grasp.maximum_travel_m") {
      params_.grasp.maximum_travel_m = param.as_double();
    } else if (name == "grasp.reconfirm_wait_s") {
      params_.grasp.reconfirm_wait_s = param.as_double();
    } else if (name == "grasp.reconfirm_tolerance_m") {
      params_.grasp.reconfirm_tolerance_m = param.as_double();
    } else if (name == "grasp.reconfirm_max_attempts") {
      params_.grasp.reconfirm_max_attempts = param.as_int();
    } else if (name == "grasp.allow_stale_anchor") {
      params_.grasp.allow_stale_anchor = param.as_bool();
    } else if (name == "grasp.fruit_inflation_m") {
      params_.grasp.fruit_inflation_m = param.as_double();
    } else if (name == "grasp.contact_detect.enabled") {
      params_.grasp.contact_detect.enabled = param.as_bool();
    } else if (name == "grasp.contact_detect.baseline_s") {
      params_.grasp.contact_detect.baseline_s = param.as_double();
    } else if (name == "grasp.contact_detect.slope_threshold") {
      params_.grasp.contact_detect.slope_threshold = param.as_double();
    } else if (name == "grasp.contact_detect.spike_threshold") {
      params_.grasp.contact_detect.spike_threshold = param.as_double();
    } else if (name == "tool.enabled") {
      params_.tool.enabled = param.as_bool();
    } else if (name == "tool.io_fun") {
      params_.tool.io_fun = param.as_int();
    } else if (name == "tool.io_pin") {
      params_.tool.io_pin = param.as_int();
    } else if (name == "tool.close_state") {
      params_.tool.close_state = param.as_double();
    } else if (name == "tool.profile_id") {
      params_.tool.profile_id = param.as_string();
    } else if (name == "timeouts.service_s") {
      params_.timeouts.service_s = param.as_double();
    } else if (name == "timeouts.refined_s") {
      params_.timeouts.refined_s = param.as_double();
    }
  }

  rcl_interfaces::msg::SetParametersResult on_set(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    for (const auto & p : params) {
      std::string reason;
      if (!validate_one(p.get_name(), p, reason)) {
        result.successful = false;
        result.reason = reason;
        return result;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & p : params) {
      assign(p.get_name(), p);
    }
    ++stamp_;
    result.successful = true;
    return result;
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_;
  rclcpp::Logger logger_;
  mutable std::mutex mutex_;
  Params params_;
  uint64_t stamp_ = 1;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr handle_;
};

}  // namespace peach_manipulation_node
