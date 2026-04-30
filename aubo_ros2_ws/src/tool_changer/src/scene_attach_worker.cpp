/**
 * @file scene_attach_worker.cpp
 * @brief 独立节点：订阅 /tool_changer_status，更新 PlanningScene 工具附着显示
 */

#include "tool_changer/scene_attach_worker.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <yaml-cpp/yaml.h>

namespace tool_changer
{

SceneAttachWorker::SceneAttachWorker(const rclcpp::NodeOptions& options)
  : rclcpp::Node("scene_attach_worker", options)
{
  planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(10).transient_local());

  tool_status_sub_ = create_subscription<tool_changer_interface::msg::ToolChangerStatus>(
      "/tool_changer_status", 10,
      [this](const tool_changer_interface::msg::ToolChangerStatus& msg) { onToolStatus(msg); });

  scene_attach_srv_ = create_service<tool_changer_interface::srv::ChangeTool>(
      "/scene_attach",
      std::bind(&SceneAttachWorker::onSceneAttach, this, std::placeholders::_1, std::placeholders::_2));

  scene_detach_srv_ = create_service<tool_changer_interface::srv::ChangeTool>(
      "/scene_detach",
      std::bind(&SceneAttachWorker::onSceneDetach, this, std::placeholders::_1, std::placeholders::_2));

  loadToolConfig();
  addAllToolsToWorld();

  RCLCPP_INFO(get_logger(), "就绪 | 监听 /tool_changer_status | /scene_attach /scene_detach | %zu 工具",
              tool_geometries_.size());
}

// ═══════════════════════════════════════════════════════════════════
// 配置加载
// ═══════════════════════════════════════════════════════════════════

void SceneAttachWorker::loadToolConfig()
{
  std::string config_path;
  try {
    config_path = ament_index_cpp::get_package_share_directory("tool_changer") + "/config/tools.yaml";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "tool_changer 包路径: %s", e.what());
    return;
  }

  YAML::Node config;
  try { config = YAML::LoadFile(config_path); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "YAML 加载失败 %s: %s", config_path.c_str(), e.what());
    return;
  }

  auto tools = config["tools"];
  if (!tools) { RCLCPP_ERROR(get_logger(), "缺少 'tools' 节点"); return; }

  for (const auto& kv : tools)
  {
    std::string tid = kv.first.as<std::string>();
    const auto& t = kv.second;
    ToolGeometry geom;

    geom.mesh_collision = loadMesh(t["mesh_collision"].as<std::string>());

    const auto& dp = t["dock_pose"];
    geom.dock_pose.position.x = dp["position"]["x"].as<double>();
    geom.dock_pose.position.y = dp["position"]["y"].as<double>();
    geom.dock_pose.position.z = dp["position"]["z"].as<double>();
    geom.dock_pose.orientation.w = 1.0;

    const auto& ao = t["attach_offset"];
    geom.attach_offset.position.x = ao["position"]["x"].as<double>();
    geom.attach_offset.position.y = ao["position"]["y"].as<double>();
    geom.attach_offset.position.z = ao["position"]["z"].as<double>();
    geom.attach_offset.orientation.w = 1.0;

    if (t["touch_links"])
      for (const auto& tl : t["touch_links"])
        geom.touch_links.push_back(tl.as<std::string>());

    tool_geometries_[tid] = geom;
    RCLCPP_INFO(get_logger(), "加载: %s (%zu△)", tid.c_str(), geom.mesh_collision.triangles.size());
  }
}

shape_msgs::msg::Mesh SceneAttachWorker::loadMesh(const std::string& resource_path)
{
  shape_msgs::msg::Mesh msg;
  try {
    auto shape = shapes::createMeshFromResource(resource_path);
    if (!shape) return msg;
    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(shape, shape_msg)) return msg;
    if (shape_msg.type() == typeid(shape_msgs::msg::Mesh))
      msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "loadMesh: %s — %s", resource_path.c_str(), e.what());
  }
  return msg;
}

// ═══════════════════════════════════════════════════════════════════
// 初始化 & 场景操作
// ═══════════════════════════════════════════════════════════════════

void SceneAttachWorker::addAllToolsToWorld()
{
  for (const auto& [tid, geom] : tool_geometries_)
  {
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;

    moveit_msgs::msg::CollisionObject co;
    co.id = "tool_" + tid;
    co.header.frame_id = "base_link";
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    co.pose = geom.dock_pose;
    co.meshes.push_back(geom.mesh_collision);
    co.mesh_poses.push_back(geometry_msgs::msg::Pose{});
    co.mesh_poses.back().orientation.w = 1.0;
    scene.world.collision_objects.push_back(co);

    planning_scene_pub_->publish(scene);
  }
  RCLCPP_INFO(get_logger(), "全部 %zu 工具已添加到 world dock", tool_geometries_.size());
}

bool SceneAttachWorker::attachTool(const std::string& tool_id)
{
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) { RCLCPP_ERROR(get_logger(), "未知: %s", tool_id.c_str()); return false; }

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;

  // REMOVE from world
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = "tool_" + tool_id;
    co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.world.collision_objects.push_back(co);
  }

  // ADD AttachedCollisionObject
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = "kuaihuan_Link";
    aco.touch_links = it->second.touch_links;
    aco.object.id = "tool_" + tool_id;
    aco.object.header.frame_id = "kuaihuan_Link";
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    aco.object.pose = it->second.attach_offset;
    aco.object.meshes.push_back(it->second.mesh_collision);
    aco.object.mesh_poses.push_back(geometry_msgs::msg::Pose{});
    aco.object.mesh_poses.back().orientation.w = 1.0;
    scene.robot_state.attached_collision_objects.push_back(aco);
  }

  planning_scene_pub_->publish(scene);
  RCLCPP_INFO(get_logger(), "附着: %s → kuaihuan_Link", tool_id.c_str());
  return true;
}

bool SceneAttachWorker::detachTool(const std::string& tool_id)
{
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) { RCLCPP_ERROR(get_logger(), "未知: %s", tool_id.c_str()); return false; }

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;

  // REMOVE AttachedCollisionObject
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = "kuaihuan_Link";
    aco.object.id = "tool_" + tool_id;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.robot_state.attached_collision_objects.push_back(aco);
  }

  // ADD back to world dock
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = "tool_" + tool_id;
    co.header.frame_id = "base_link";
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    co.pose = it->second.dock_pose;
    co.meshes.push_back(it->second.mesh_collision);
    co.mesh_poses.push_back(geometry_msgs::msg::Pose{});
    co.mesh_poses.back().orientation.w = 1.0;
    scene.world.collision_objects.push_back(co);
  }

  planning_scene_pub_->publish(scene);
  RCLCPP_INFO(get_logger(), "脱离: %s → dock", tool_id.c_str());
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// /tool_changer_status 订阅
// ═══════════════════════════════════════════════════════════════════

void SceneAttachWorker::onToolStatus(const tool_changer_interface::msg::ToolChangerStatus& msg)
{
  const std::string new_tool = msg.is_connected ? msg.tool_id : "";
  if (new_tool == current_attached_tool_)
    return;

  RCLCPP_INFO(get_logger(), "状态变更: %s → %s (connected=%s)",
              current_attached_tool_.c_str(), new_tool.c_str(),
              msg.is_connected ? "true" : "false");

  // 先脱离旧工具，再附着新工具
  if (!current_attached_tool_.empty())
    detachTool(current_attached_tool_);

  if (!new_tool.empty())
    attachTool(new_tool);

  current_attached_tool_ = new_tool;
}

// ═══════════════════════════════════════════════════════════════════
// 服务回调
// ═══════════════════════════════════════════════════════════════════

void SceneAttachWorker::onSceneAttach(
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> req,
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> resp)
{
  resp->success = attachTool(req->tool_id);
  if (resp->success) current_attached_tool_ = req->tool_id;
  resp->message = resp->success ? ("附着: " + req->tool_id) : ("失败: " + req->tool_id);
}

void SceneAttachWorker::onSceneDetach(
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> req,
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> resp)
{
  resp->success = detachTool(req->tool_id);
  if (resp->success && current_attached_tool_ == req->tool_id)
    current_attached_tool_.clear();
  resp->message = resp->success ? ("脱离: " + req->tool_id) : ("失败: " + req->tool_id);
}

}  // namespace tool_changer

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tool_changer::SceneAttachWorker>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
