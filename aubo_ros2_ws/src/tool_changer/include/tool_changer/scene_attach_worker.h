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
#include <tool_changer_interface/msg/tool_changer_status.hpp>
#include <tool_changer_interface/srv/change_tool.hpp>

namespace tool_changer
{

/**
 * @brief PlanningScene 附着 Worker
 *
 * 订阅 /tool_changer_status，根据当前工具自动更新 PlanningScene：
 *   - 工具 attached → REMOVE from world + ADD AttachedCollisionObject on kuaihuan_Link
 *   - 工具 detached → REMOVE from robot + ADD CollisionObject to world dock
 *
 * 同时提供 /scene_attach / /scene_detach 服务用于手动触发。
 */
class SceneAttachWorker : public rclcpp::Node
{
public:
  explicit SceneAttachWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SceneAttachWorker() override = default;

private:
  struct ToolGeometry {
    shape_msgs::msg::Mesh mesh_collision;
    std::string mesh_visual;  // MESH_RESOURCE marker 路径 (用于 RViz2 平滑渲染)
    geometry_msgs::msg::Pose dock_pose;
    geometry_msgs::msg::Pose attach_offset;
    std::vector<std::string> touch_links;
  };

  // 加载 tools.yaml
  void loadToolConfig();
  shape_msgs::msg::Mesh loadMesh(const std::string& resource_path);

  // 初始化：所有工具添加到 world dock
  void addAllToolsToWorld();

  // 场景操作
  bool attachTool(const std::string& tool_id);
  bool detachTool(const std::string& tool_id);

  // World dock 显示辅助方法（只改 world，不碰 AttachedCollisionObject）
  void addToolToWorldDock(const std::string& tool_id);
  void removeToolFromWorld(const std::string& tool_id);

  // /tool_changer_status 订阅回调
  void onToolStatus(const tool_changer_interface::msg::ToolChangerStatus& msg);

  // robot_description 参数更新
  void updateRobotDescription(const std::string& tool_id);

  // 服务回调
  void onSceneAttach(const std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> req,
                     std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> resp);
  void onSceneDetach(const std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> req,
                     std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> resp);

  // 成员
  std::map<std::string, ToolGeometry> tool_geometries_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::Subscription<tool_changer_interface::msg::ToolChangerStatus>::SharedPtr tool_status_sub_;

  rclcpp::Service<tool_changer_interface::srv::ChangeTool>::SharedPtr scene_attach_srv_;
  rclcpp::Service<tool_changer_interface::srv::ChangeTool>::SharedPtr scene_detach_srv_;

  // robot_description 参数更新
  std::map<std::string, std::string> urdf_cache_;
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_pub_;

  std::string current_attached_tool_;
};

}  // namespace tool_changer

#endif  // TOOL_CHANGER_SCENE_ATTACH_WORKER_H_
