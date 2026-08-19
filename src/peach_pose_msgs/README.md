# peach_pose_msgs — PeachPose 感知→规划边界消息包

## 简介

桃子位姿估计（PeachPose）链路的自定义消息包：把感知结果封装成三类冻结契约
——3D 抓取候选、2D 像素参考、拟合诊断——供下游抓取规划消费。契约源自
peach_canopy 项目的 interfaces（见 msg 头注释），**只承载感知参考，不含
运动指令**（package.xml 描述："感知→规划边界"）。纯 rosidl 接口包
（ament_cmake），不含节点；目前唯一生产者是 peach_pose_ros2 包。

## 使用方法

构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_pose_msgs
source install/setup.bash
```

在其他包中依赖：

- package.xml：`<depend>peach_pose_msgs</depend>`
- C++（ament_cmake）：`find_package(peach_pose_msgs REQUIRED)` +
  `ament_target_dependencies(<目标> peach_pose_msgs)`，代码内 include 用
  snake_case 头，如 `#include "peach_pose_msgs/msg/bag_grasp_candidate.hpp"`
- Python（rclpy）：`from peach_pose_msgs.msg import BagGraspCandidateArray`

查看接口定义：

```bash
ros2 interface show peach_pose_msgs/msg/BagGraspCandidate
ros2 interface show peach_pose_msgs/msg/BagFitting
```

本包消息不带运动语义，本身无真机安全约束；消费其输出的下游运动规划
须遵守 AGENTS.md 第 10 节（速度/加速度缩放先压 0.1）的安全约定。

## 执行逻辑

生产者只有一个：peach_pose_ros2 的 `peach_pose_node`。一帧 RGB-D 处理完成后
同帧发布到规范组话题（A5 起旧 `~/` 组已删除，发布面单套化）：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/peach/perception/initial_pose` | `BagGraspCandidateArray` | 3D 抓取候选，坐标系=header.frame_id（output_frame，默认 base_link；TF 失败退回相机系并告警） |
| `/peach/perception/diagnostics` | `BagFittingArray` | 拟合诊断（TargetPoseResult.metrics 的平坦化） |

2D 像素参考不再单独成话题，随 `/peach/perception/target_observations` 的
`candidate_2d` 字段与 3D 按 `target_id` 对齐下发。下游按 `target_id` 关联
数据流。消息的 `status` 枚举一致
（ACCEPT=0 / REOBSERVE=1 / REJECT=2），由节点的刀具几何门控判定；
SAM 掩膜缺失时显式 REOBSERVE + `mask_unavailable`，不做静默深度回退
（管线细节见 `src/peach_pose_ros2/README.md`）。

## 软件框架

6 个 msg，依赖仅 std_msgs、geometry_msgs；`member_of_group
rosidl_interface_packages`；经 ament_lint_auto 接入 lint 测试。

```text
msg/
  BagGraspCandidate.msg       3D 抓取候选：header（frame_id=输出系）、
                              target_id、entry_pose（Pose）、bag_bottom /
                              bag_neck（Point）、translation_direction
                              （Vector3）、bag_diameter_upper_m、
                              suggested_travel_m、confidence、status 三态 +
                              diagnostic_flags；strategy_id / model_version /
                              calibration_version / tool_version 四个追溯标识
  BagGrasp2D.msg              2D 参考：检测框 bbox_x/y/w/h（像素）、
                              bottom / neck / grasp / travel_end 四个像素点
                              （Point，z 恒 0，各配 has_* 有效位）、
                              confidence、status、diagnostic_flags
  BagFitting.msg              拟合诊断：target_kind / mask_source /
                              axis_source、axis_confidence 与
                              axis_disagreement_deg、theta_err_deg、
                              error_budget_mm / radial_clearance_mm、
                              valid_depth / foreground / boundary_touch
                              三比率、bag_length_m / bag_diameter_upper_m /
                              travel_m；袋线圆柱拟合（cylinder_rms_m /
                              inlier_ratio）、果线球拟合与梗洼
                              （fruit_radius_m / sphere_* / cavity_dip_mm）、
                              axis_polarity_corrected、status、
                              diagnostic_flags；无效标量填 -1
  BagGraspCandidateArray.msg  header + BagGraspCandidate[] candidates
  BagFittingArray.msg         header + BagFitting[] fittings
```
