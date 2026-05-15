/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef TOOL_CHANGER_SCENE_ATTACH_WORKER_H_
#define TOOL_CHANGER_SCENE_ATTACH_WORKER_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/string.hpp>
#include <ivg_interfaces/msg/tool_changer_status.hpp>
#include <ivg_interfaces/srv/change_tool.hpp>

namespace tool_changer
{

/**
 * @brief 订阅 /tool_changer_status，通过 AttachedCollisionObject 让 move_group 感知工具碰撞。
 *
 * 碰撞由两层协同提供：
 *   1. URDF <collision> — RViz 视觉渲染 + robot_state_publisher TF（由 updateRobotDescription 管理）
 *   2. AttachedCollisionObject (/planning_scene diff) — move_group 感知碰撞、用于规划避障
 *
 * 工具切换时：
 *   附着：发送 AttachedCollisionObject ADD（网格附着到 kuaihuan_Link）+ 更新 URDF
 *   脱离：发送 AttachedCollisionObject REMOVE + 更新空工具 URDF
 *
 * 不再使用 world dock 碰撞对象，只关注已附着的工具。
 */
class SceneAttachWorker : public rclcpp::Node
{
public:
  explicit SceneAttachWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SceneAttachWorker() override = default;

private:
  struct ToolGeometry {
    shape_msgs::msg::Mesh mesh_collision;
    geometry_msgs::msg::Pose attach_offset;  // 工具相对 kuaihuan_Link 的偏移
    std::vector<std::string> touch_links;    // 附着时豁免碰撞的 link 列表
  };

  // 配置加载
  void loadToolConfig();
  shape_msgs::msg::Mesh loadMesh(const std::string& resource_path);

  // /tool_changer_status 回调
  void onToolStatus(const ivg_interfaces::msg::ToolChangerStatus& msg);

  // robot_description 更新（URDF 缓存 → topic + robot_state_publisher 参数）
  void updateRobotDescription(const std::string& tool_id);

  // AttachedCollisionObject 管理（/planning_scene diff）
  void attachToolToScene(const std::string& tool_id);
  void detachToolFromScene(const std::string& tool_id);

  // 服务回调
  void onSceneAttach(const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
                     std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp);
  void onSceneDetach(const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
                     std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp);

  // ── 成员 ──

  std::map<std::string, ToolGeometry> tool_geometries_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::Subscription<ivg_interfaces::msg::ToolChangerStatus>::SharedPtr tool_status_sub_;

  rclcpp::Service<ivg_interfaces::srv::ChangeTool>::SharedPtr scene_attach_srv_;
  rclcpp::Service<ivg_interfaces::srv::ChangeTool>::SharedPtr scene_detach_srv_;

  // URDF 缓存 & 发布
  std::map<std::string, std::string> urdf_cache_;
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_pub_;

  std::string current_attached_tool_;
};

}  // namespace tool_changer

#endif  // TOOL_CHANGER_SCENE_ATTACH_WORKER_H_
