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
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/string.hpp>
#include <ivg_interfaces/msg/tool_changer_status.hpp>
#include <ivg_interfaces/srv/change_tool.hpp>

namespace tool_changer
{

/**
 * @brief 订阅 /tool_changer_status（及 /scene_attach、/scene_detach），同步 MoveIt 中「已连接工具」的碰撞几何喵~
 *
 * - **`/attached_collision_object`**：发布 ADD/REMOVE `AttachedCollisionObject`（附着到 `kuaihuan_Link`）喵~
 * - **`/planning_scene`（is_diff）**：仅推送 `world.collision_objects` 中对 `attached_tool_<id>` 的 REMOVE，
 *   清除 detach 后可能残留在 world 中的同名对象，避免误判碰撞喵~
 * - **发布 `/robot_description`**：工具切换时推送对应 URDF（TRANSIENT_LOCAL），前端 3D 视图自动重载喵~
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

  // robot_description 参数更新（前端 3D URDF 重载）喵~
  // sync=true: 同步等待 set_parameters 结果并验证（/set_display_tool 使用）
  // sync=false: 异步 set_parameters + 回调日志（/scene_attach 等内部路径，避免阻塞）喵~
  void updateRobotDescription(const std::string& tool_id, bool sync = false);

  // MoveIt：ACO（/attached_collision_object）+ world REMOVE（/planning_scene）
  void attachToolToScene(const std::string& tool_id);
  void detachToolFromScene(const std::string& tool_id);
  /** detach/attach 后清除 world 中残留的 attached_tool_<tool_id>（PlanningScene diff）喵~ */
  void removeWorldToolObject(const std::string& tool_id);

  // 服务回调
  void onSceneAttach(const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
                     std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp);
  void onSceneDetach(const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
                     std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp);
  void onSetDisplayTool(const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
                        std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp);

  // ── 成员 ──

  std::map<std::string, ToolGeometry> tool_geometries_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attached_object_pub_;
  rclcpp::Subscription<ivg_interfaces::msg::ToolChangerStatus>::SharedPtr tool_status_sub_;

  rclcpp::Service<ivg_interfaces::srv::ChangeTool>::SharedPtr scene_attach_srv_;
  rclcpp::Service<ivg_interfaces::srv::ChangeTool>::SharedPtr scene_detach_srv_;
  rclcpp::Service<ivg_interfaces::srv::ChangeTool>::SharedPtr display_tool_srv_;

  // robot_description 动态发布（前端 3D + RViz2 URDF 重载）
  std::map<std::string, std::string> urdf_cache_;
  rclcpp::CallbackGroup::SharedPtr param_cb_group_;         // 独立回调组，避免 set_parameters 死锁
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_pub_;

  std::string current_attached_tool_;   // 物理快换当前工具（/tool_changer_status 驱动）
  std::string current_display_tool_;    // 前端 URDF 显示夹爪选择（/set_display_tool 驱动）
};

}  // namespace tool_changer

#endif  // TOOL_CHANGER_SCENE_ATTACH_WORKER_H_
