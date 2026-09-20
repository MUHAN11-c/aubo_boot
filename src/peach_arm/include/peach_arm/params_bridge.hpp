// 功能：GPL Params 快照 → 运动/接触 Config 的单点转换（W5-1）。逐键转抄
// 自原 rebuildMotionInterface / rebuildGraspTask 的 Config 组装段；节点侧
// 只保留 safety/execution 门回调与 staging IK 回调装配。UR Driver GPL 单源
// 同法：yaml → Params → 运行配置只经一道转写。
#ifndef PEACH_MANIPULATION__PARAMS_BRIDGE_HPP_
#define PEACH_MANIPULATION__PARAMS_BRIDGE_HPP_

#include <string>

#include <peach_arm/arm_parameters.hpp>

#include "peach_arm/contact_monitor.hpp"
#include "peach_arm/frame_timeouts.hpp"
#include "peach_arm/grasp_task.hpp"
#include "peach_arm/motion.hpp"
#include "peach_arm/quality_gate.hpp"
#include "peach_arm/safety_gate.hpp"
#include "peach_arm/staging_selector.hpp"
#include "peach_arm/view_planner.hpp"

namespace peach_arm
{

/// Params → MoveItMotionConfig（值字段全量；回调由节点装配）。
inline MoveItMotionConfig toMotionConfig(const Params & params)
{
  MoveItMotionConfig config;
  config.base_frame = params.frames.base;
  config.tip_frame = params.frames.tip;
  config.camera_frame = params.frames.camera;
  config.tool_frame = params.frames.tool;
  const auto & moveit = params.moveit;
  config.pilz_pipeline = moveit.pilz_pipeline;
  config.fallback_pipeline = moveit.fallback_pipeline;
  config.transit_velocity_scaling = moveit.transit_velocity_scaling;
  config.transit_acceleration_scaling = moveit.transit_acceleration_scaling;
  config.transit_max_duration_s = moveit.transit_max_duration_s;
  config.transit_max_total_joint_travel_rad =
    moveit.transit_max_total_joint_travel_rad;
  config.transit_max_single_joint_travel_rad =
    moveit.transit_max_single_joint_travel_rad;
  config.observe_planning_time_s = moveit.observe_planning_time_s;
  config.observe_planning_attempts =
    static_cast<int>(moveit.observe_planning_attempts);
  config.observe_max_duration_s = moveit.observe_max_duration_s;
  config.observe_max_total_joint_travel_rad =
    moveit.observe_max_total_joint_travel_rad;
  config.observe_max_single_joint_travel_rad =
    moveit.observe_max_single_joint_travel_rad;
  config.photo_planning_time_s = moveit.photo_planning_time_s;
  config.photo_ptp_planning_time_s = moveit.photo_ptp_planning_time_s;
  config.default_planning_time_s = moveit.planning_time_s;
  config.default_planning_attempts = static_cast<int>(moveit.planning_attempts);
  config.photo_pose_joint_tolerance_rad = params.photo_pose_joint_tolerance_rad;
  config.photo_pose_max_joint_vel_rad_s = params.photo_pose_max_joint_vel_rad_s;
  return config;
}

/// Params → GraspTaskConfig 值字段（select_goal_joints / lookup_current_tip /
/// 执行门 / protected_zones 由节点在 toGraspTaskConfig 之后装配）。
inline GraspTaskConfig toGraspTaskConfig(const Params & params)
{
  GraspTaskConfig config;
  const auto & moveit = params.moveit;
  config.planning_group = moveit.planning_group;
  config.tip_frame = params.frames.tip;
  config.base_frame = params.frames.base;
  config.free_space_pipeline = moveit.mtc_free_space_pipeline;
  config.free_space_planner = moveit.mtc_free_space_planner;
  config.planning_time_s = moveit.planning_time_s;
  config.velocity_scaling = moveit.velocity_scaling;
  config.acceleration_scaling = moveit.acceleration_scaling;
  config.cartesian_step_m = moveit.mtc_cartesian_step_m;
  config.cartesian_min_fraction = moveit.mtc_cartesian_min_fraction;
  config.cartesian_precision_m = moveit.mtc_cartesian_precision_m;
  config.max_solutions = static_cast<std::size_t>(moveit.mtc_max_solutions);
  config.approach_max_duration_s = moveit.mtc_approach_max_duration_s;
  config.approach_max_total_joint_travel_rad =
    moveit.mtc_approach_max_total_joint_travel_rad;
  config.approach_max_single_joint_travel_rad =
    moveit.mtc_approach_max_single_joint_travel_rad;
  config.approach_max_detour_ratio = moveit.mtc_approach_max_detour_ratio;
  config.approach_max_chord_deviation_m =
    moveit.mtc_approach_max_chord_deviation_m;
  config.approach_max_recede_m = moveit.mtc_approach_max_recede_m;
  config.staging_max_detour_ratio = moveit.mtc_approach_transit_max_detour_ratio;
  config.staging_max_chord_deviation_m =
    moveit.mtc_approach_transit_max_chord_deviation_m;
  config.staging_max_recede_m = moveit.mtc_approach_transit_max_recede_m;
  config.approach_max_tcp_rotation_deg = moveit.mtc_approach_max_tcp_rotation_deg;
  config.approach_tcp_rotation_slack_deg =
    moveit.mtc_approach_tcp_rotation_slack_deg;
  config.approach_keepout_radius_m = moveit.mtc_approach_keepout_radius_m;
  config.approach_keepout_axial_m = moveit.mtc_approach_keepout_axial_m;
  config.approach_cartesian_max_distance_m =
    moveit.mtc_approach_cartesian_max_distance_m;
  config.approach_along_axis_m = moveit.mtc_approach_along_axis_m;
  config.approach_staging_standoff_m = moveit.approach_staging_standoff_m;
  config.approach_near_velocity_scaling = moveit.approach_near_velocity_scaling;
  config.approach_max_lateral_m = moveit.mtc_approach_max_lateral_m;
  config.approach_max_align_deg = moveit.mtc_approach_max_align_deg;
  config.fruit_inflation_m = params.grasp.fruit_inflation_m;
  config.tool_links = params.tool.links;
  config.contact_tool_links = params.tool.contact_links;
  config.tool_body_length_m = params.tool.body_length_m;
  config.tool_body_radius_m = params.tool.body_radius_m;
  return config;
}

/// Params → staging 候选选择配置（W5-2；默认值=原节点硬编码）。
inline StagingSelectorConfig toStagingSelectorConfig(const Params & params)
{
  StagingSelectorConfig config;
  config.seeds = static_cast<int>(params.staging.seeds);
  config.wrist_weight = params.staging.wrist_weight;
  config.roll_penalty = params.staging.roll_penalty;
  config.top_n = static_cast<int>(params.staging.top_n);
  return config;
}

/// Params → 帧率自适应超时族配置（W5-3）。
inline FrameRateTimeoutConfig toFrameRateTimeoutConfig(const Params & params)
{
  FrameRateTimeoutConfig config;
  config.assumed_frame_interval_s = params.scan.assumed_frame_interval_s;
  config.frame_wait_s = params.scan.frame_wait_s;
  config.target_observation_max_age_config_s =
    params.execution.target_observation_max_age_s;
  config.reconfirm_wait_s = params.grasp.reconfirm_wait_s;
  config.refined_timeout_s = params.timeouts.refined_s;
  return config;
}

/// Params → 视点规划配置值字段（W13-B）。protected_zones 需逐盒 WARN 的
/// 解析（parseProtectedZones），留在节点装配（view_config.protected_zones）。
inline ViewPlannerConfig toViewPlannerConfig(const Params & params)
{
  ViewPlannerConfig config;
  config.observation_radius_m = params.scan.observation_radius_m;
  config.minimum_radius_m = params.scan.minimum_radius_m;
  config.azimuth_step_deg = params.scan.azimuth_step_deg;
  config.elevation_step_deg = params.scan.elevation_step_deg;
  config.elevation_limit_deg = params.scan.elevation_limit_deg;
  config.preferred_baseline_deg = params.scan.preferred_baseline_deg;
  config.radial_step_m = params.scan.radial_step_m;
  config.max_camera_step_m = params.scan.max_camera_step_m;
  config.workspace_max_reach_m = params.scan.workspace_max_reach_m;
  config.min_camera_height_m = params.scan.min_camera_height_m;
  return config;
}

/// Params → 质量门配置值字段（W13-B）。
inline QualityGateConfig toQualityGateConfig(const Params & params)
{
  QualityGateConfig config;
  config.minimum_views = static_cast<std::size_t>(params.quality.minimum_views);
  config.minimum_baseline_deg = params.quality.minimum_baseline_deg;
  config.minimum_mean_nearest_baseline_deg =
    params.quality.minimum_mean_nearest_baseline_deg;
  config.minimum_mean_depth_ratio = params.quality.minimum_mean_depth_ratio;
  config.maximum_data_age_s = params.quality.maximum_data_age_s;
  config.maximum_axis_angle_deg = params.quality.maximum_axis_angle_deg;
  return config;
}

/// Params → 安全门配置值字段（W13-B）。运行期帧率自适应的新鲜度上限经
/// SafetyGate::set_target_observation_max_age_s 注入，不走本桥。
inline SafetyGateConfig toSafetyGateConfig(const Params & params)
{
  SafetyGateConfig config;
  config.require_robot_status = params.execution.require_robot_status;
  config.robot_status_max_age_s = params.execution.robot_status_max_age_s;
  config.target_observation_max_age_s =
    params.execution.target_observation_max_age_s;
  return config;
}

/// Params → 接触止损配置（grasp.contact_detect.*，W13-B；阈值须真机标定）。
inline ContactDetectConfig toContactDetectConfig(const Params & params)
{
  ContactDetectConfig config;
  config.enabled = params.grasp.contact_detect.enabled;
  config.baseline_s = params.grasp.contact_detect.baseline_s;
  config.slope_threshold = params.grasp.contact_detect.slope_threshold;
  config.spike_threshold = params.grasp.contact_detect.spike_threshold;
  return config;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__PARAMS_BRIDGE_HPP_
