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
#include <set>
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
 * @brief 订阅 /tool_changer_status，管理 PlanningScene world dock 碰撞对象 + 动态 URDF 切换。
 *
 * 碰撞几何由 URDF <collision> 负责。工具切换时发布新 URDF 到 /robot_description
 * 并设置 robot_state_publisher 参数触发 TF 树重建。
 * World dock 碰撞对象用于防止机械臂与停靠工具碰撞。
 */
class SceneAttachWorker : public rclcpp::Node
{
public:
  explicit SceneAttachWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SceneAttachWorker() override = default;

private:
  struct ToolGeometry
  {
    shape_msgs::msg::Mesh mesh_collision;
    geometry_msgs::msg::Pose dock_pose;
  };

  // 配置 & 初始化
  void loadToolConfig();
  shape_msgs::msg::Mesh loadMesh(const std::string& resource_path);
  void addAllToolsToWorld();

  // World dock 操作
  void addToolToWorldDock(const std::string& tool_id);
  void removeToolFromWorld(const std::string& tool_id);

  // /tool_changer_status 回调
  void onToolStatus(const ivg_interfaces::msg::ToolChangerStatus& msg);

  // robot_description 更新（URDF 缓存 → /robot_description topic + robot_state_publisher 参数）
  void updateRobotDescription(const std::string& tool_id);

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
  std::set<std::string> tools_in_world_;  // 当前在 world dock 中的工具 ID 集合
};

}  // namespace tool_changer

#endif  // TOOL_CHANGER_SCENE_ATTACH_WORKER_H_
