/**
 * @file scene_attach_worker.cpp
 * @brief 订阅 /tool_changer_status，管理 PlanningScene world dock 碰撞对象 + 动态 URDF 切换。
 *
 * 碰撞几何由 URDF <collision> 负责。工具切换时发布新 URDF 到 /robot_description
 * 并设置 robot_state_publisher 参数触发 TF 树重建。
 * World dock 碰撞对象用于防止机械臂与停靠工具碰撞。
 */

#include "tool_changer/scene_attach_worker.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <resource_retriever/retriever.hpp>

#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

namespace tool_changer
{

// ═══════════════════════════════════════════════════════════════════════
// 工具函数
// ═══════════════════════════════════════════════════════════════════════

namespace
{

/// 调用 xacro 生成 URDF 字符串；失败返回空
std::string runXacro(const std::string& xacro_path, const std::string& extra_args)
{
  std::ostringstream cmd;
  cmd << "xacro " << xacro_path << " " << extra_args << " 2>/dev/null";
  FILE* pipe = popen(cmd.str().c_str(), "r");
  if (!pipe) return {};
  std::string out;
  char buf[4096];
  while (fgets(buf, sizeof(buf), pipe)) out += buf;
  pclose(pipe);
  return out;
}

/// 构造 CollisionObject（不设 frame_id，由调用方设置）
moveit_msgs::msg::CollisionObject makeCollisionObject(
    const std::string& tool_id, const geometry_msgs::msg::Pose& pose,
    const shape_msgs::msg::Mesh& mesh, int8_t operation)
{
  moveit_msgs::msg::CollisionObject co;
  co.id = "tool_" + tool_id;
  co.operation = operation;
  co.pose = pose;
  co.meshes.push_back(mesh);
  co.mesh_poses.push_back(geometry_msgs::msg::Pose{});
  co.mesh_poses.back().orientation.w = 1.0;
  return co;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// 构造函数
// ═══════════════════════════════════════════════════════════════════════

SceneAttachWorker::SceneAttachWorker(const rclcpp::NodeOptions& options)
  : rclcpp::Node("scene_attach_worker", options)
{
  planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(10).transient_local());
  robot_description_pub_ = create_publisher<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local());

  tool_status_sub_ = create_subscription<ivg_interfaces::msg::ToolChangerStatus>(
      "/tool_changer_status", 10,
      [this](const ivg_interfaces::msg::ToolChangerStatus& msg) { onToolStatus(msg); });

  using namespace std::placeholders;
  scene_attach_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
      "/scene_attach", std::bind(&SceneAttachWorker::onSceneAttach, this, _1, _2));
  scene_detach_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
      "/scene_detach", std::bind(&SceneAttachWorker::onSceneDetach, this, _1, _2));

  loadToolConfig();
  addAllToolsToWorld();

  RCLCPP_INFO(get_logger(), "就绪 | %zu 工具 | 监听 /tool_changer_status", tool_geometries_.size());

  // ── 预生成 URDF 缓存 ──
  const std::string xacro_path =
    ament_index_cpp::get_package_share_directory("aubo_moveit_config") +
    "/config/aubo_e5.urdf.xacro";

  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "robot_state_publisher");

  for (const auto& [tid, _] : tool_geometries_)
  {
    std::string urdf = runXacro(xacro_path, "gripper:=" + tid + " use_fake_hardware:=true");
    if (!urdf.empty()) {
      urdf_cache_[tid] = urdf;
      RCLCPP_INFO(get_logger(), "URDF 缓存: %s (%zu bytes)", tid.c_str(), urdf.size());
    }
  }
  // 无工具 URDF
  std::string empty_urdf = runXacro(xacro_path, "use_fake_hardware:=true");
  if (!empty_urdf.empty()) urdf_cache_[""] = empty_urdf;
}

// ═══════════════════════════════════════════════════════════════════════
// 配置加载
// ═══════════════════════════════════════════════════════════════════════

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
  if (!tools) {
    RCLCPP_ERROR(get_logger(), "缺少 'tools' 节点");
    return;
  }

  for (const auto& kv : tools)
  {
    const std::string tid = kv.first.as<std::string>();
    const auto& t = kv.second;

    ToolGeometry geom;
    geom.mesh_collision = loadMesh(t["mesh_collision"].as<std::string>());

    const auto& dp = t["dock_pose"];
    geom.dock_pose.position.x    = dp["position"]["x"].as<double>();
    geom.dock_pose.position.y    = dp["position"]["y"].as<double>();
    geom.dock_pose.position.z    = dp["position"]["z"].as<double>();
    geom.dock_pose.orientation.x = dp["orientation"]["x"].as<double>();
    geom.dock_pose.orientation.y = dp["orientation"]["y"].as<double>();
    geom.dock_pose.orientation.z = dp["orientation"]["z"].as<double>();
    geom.dock_pose.orientation.w = dp["orientation"]["w"].as<double>();

    tool_geometries_[tid] = geom;
    RCLCPP_INFO(get_logger(), "加载: %s (%zu△)", tid.c_str(), geom.mesh_collision.triangles.size());
  }
}

// ═══════════════════════════════════════════════════════════════════════
// STL 加载
// ═══════════════════════════════════════════════════════════════════════

shape_msgs::msg::Mesh SceneAttachWorker::loadMesh(const std::string& resource_path)
{
  shape_msgs::msg::Mesh msg;
  try {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res = retriever.get(resource_path);
    if (res.size < 84) {
      RCLCPP_ERROR(get_logger(), "loadMesh: 文件太小 (%zu bytes): %s", res.size, resource_path.c_str());
      return msg;
    }

    const uint8_t* data = res.data.get();

    if (res.size >= 5 && std::memcmp(data, "solid", 5) == 0) {
      RCLCPP_WARN(get_logger(), "loadMesh: ASCII STL 不支持: %s", resource_path.c_str());
      return msg;
    }

    uint32_t num_triangles = *reinterpret_cast<const uint32_t*>(data + 80);
    size_t expected_size = 84 + static_cast<size_t>(num_triangles) * 50;
    if (res.size < expected_size) {
      RCLCPP_ERROR(get_logger(), "loadMesh: 大小不匹配 (expected %zu, got %zu): %s",
                   expected_size, res.size, resource_path.c_str());
      return msg;
    }

    using VertexKey = std::tuple<double, double, double>;
    std::map<VertexKey, uint32_t> vertex_map;
    std::vector<geometry_msgs::msg::Point> unique_verts;
    std::vector<shape_msgs::msg::MeshTriangle> triangles;
    triangles.reserve(num_triangles);

    for (uint32_t t = 0; t < num_triangles; ++t)
    {
      const uint8_t* tb = data + 84 + static_cast<size_t>(t) * 50;
      shape_msgs::msg::MeshTriangle tri;
      for (int v = 0; v < 3; ++v)
      {
        const uint8_t* vb = tb + 12 + static_cast<size_t>(v) * 12;
        float fx, fy, fz;
        std::memcpy(&fx, vb, 4);
        std::memcpy(&fy, vb + 4, 4);
        std::memcpy(&fz, vb + 8, 4);

        VertexKey key(static_cast<double>(fx), static_cast<double>(fy), static_cast<double>(fz));
        auto it = vertex_map.find(key);
        if (it != vertex_map.end()) {
          tri.vertex_indices[v] = it->second;
        } else {
          uint32_t idx = static_cast<uint32_t>(unique_verts.size());
          vertex_map[key] = idx;
          tri.vertex_indices[v] = idx;
          geometry_msgs::msg::Point pt;
          pt.x = fx; pt.y = fy; pt.z = fz;
          unique_verts.push_back(pt);
        }
      }
      triangles.push_back(tri);
    }

    msg.vertices = std::move(unique_verts);
    msg.triangles = std::move(triangles);

    if (!msg.vertices.empty()) {
      RCLCPP_INFO(get_logger(), "loadMesh: %s (%u△→%zu△, %zu verts)",
                  resource_path.c_str(), num_triangles,
                  msg.triangles.size(), msg.vertices.size());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "loadMesh: %s — %s", resource_path.c_str(), e.what());
  }
  return msg;
}

// ═══════════════════════════════════════════════════════════════════════
// 初始化：所有工具添加到 world dock
// ═══════════════════════════════════════════════════════════════════════

void SceneAttachWorker::addAllToolsToWorld()
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  for (const auto& [tid, geom] : tool_geometries_)
  {
    auto co = makeCollisionObject(tid, geom.dock_pose, geom.mesh_collision,
                                  moveit_msgs::msg::CollisionObject::ADD);
    co.header.frame_id = "base_link";
    scene.world.collision_objects.push_back(co);
    tools_in_world_.insert(tid);
  }
  planning_scene_pub_->publish(scene);
  RCLCPP_INFO(get_logger(), "全部 %zu 工具已添加到 world dock", tool_geometries_.size());
}

// ═══════════════════════════════════════════════════════════════════════
// /tool_changer_status 回调
// ═══════════════════════════════════════════════════════════════════════

void SceneAttachWorker::onToolStatus(const ivg_interfaces::msg::ToolChangerStatus& msg)
{
  const std::string new_tool = msg.is_connected ? msg.tool_id : "";

  if (new_tool == current_attached_tool_)
    return;

  RCLCPP_INFO(get_logger(), "状态变更: %s → %s (connected=%s)",
              current_attached_tool_.c_str(), new_tool.c_str(),
              msg.is_connected ? "true" : "false");

  // 旧工具放回 world dock
  if (!current_attached_tool_.empty())
    addToolToWorldDock(current_attached_tool_);

  // 新工具从 world dock 移除
  if (!new_tool.empty())
    removeToolFromWorld(new_tool);

  // 仅在 attach 新工具时更新 URDF
  // detach 的碰撞模型清理已由 addToolToWorldDock (PlanningScene diff) 处理
  if (!new_tool.empty())
    updateRobotDescription(new_tool);

  current_attached_tool_ = new_tool;
}

// ═══════════════════════════════════════════════════════════════════════
// World dock 操作
// ═══════════════════════════════════════════════════════════════════════

void SceneAttachWorker::addToolToWorldDock(const std::string& tool_id)
{
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) return;

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  auto co = makeCollisionObject(tool_id, it->second.dock_pose, it->second.mesh_collision,
                                moveit_msgs::msg::CollisionObject::ADD);
  co.header.frame_id = "base_link";
  scene.world.collision_objects.push_back(co);
  planning_scene_pub_->publish(scene);
  tools_in_world_.insert(tool_id);
}

void SceneAttachWorker::removeToolFromWorld(const std::string& tool_id)
{
  if (tool_geometries_.find(tool_id) == tool_geometries_.end()) return;

  if (tools_in_world_.find(tool_id) == tools_in_world_.end()) {
    RCLCPP_DEBUG(get_logger(), "跳过 REMOVE: tool_%s 不在 world 中", tool_id.c_str());
    return;
  }

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  auto co = makeCollisionObject(tool_id, geometry_msgs::msg::Pose{}, shape_msgs::msg::Mesh{},
                                moveit_msgs::msg::CollisionObject::REMOVE);
  scene.world.collision_objects.push_back(co);
  planning_scene_pub_->publish(scene);
  tools_in_world_.erase(tool_id);
}

// ═══════════════════════════════════════════════════════════════════════
// 服务回调
// ═══════════════════════════════════════════════════════════════════════

void SceneAttachWorker::onSceneAttach(
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp)
{
  bool found = tool_geometries_.find(req->tool_id) != tool_geometries_.end();
  if (found) {
    removeToolFromWorld(req->tool_id);
    current_attached_tool_ = req->tool_id;
    updateRobotDescription(req->tool_id);
  }
  resp->success = found;
  resp->message = found ? ("附着: " + req->tool_id) : ("未知工具: " + req->tool_id);
}

void SceneAttachWorker::onSceneDetach(
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp)
{
  bool found = tool_geometries_.find(req->tool_id) != tool_geometries_.end();
  if (found && current_attached_tool_ == req->tool_id) {
    addToolToWorldDock(req->tool_id);
    current_attached_tool_.clear();
    updateRobotDescription("");
  }
  resp->success = found;
  resp->message = found ? ("脱离: " + req->tool_id) : ("未知工具: " + req->tool_id);
}

// ═══════════════════════════════════════════════════════════════════════
// robot_description 更新
// ═══════════════════════════════════════════════════════════════════════

void SceneAttachWorker::updateRobotDescription(const std::string& tool_id)
{
  auto it = urdf_cache_.find(tool_id);
  if (it == urdf_cache_.end()) {
    RCLCPP_WARN(get_logger(), "URDF 缓存未命中: %s", tool_id.c_str());
    return;
  }
  const auto& urdf = it->second;
  if (urdf.empty()) return;

  // 发布到 /robot_description → RViz2 / move_group 重载
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = urdf;
  robot_description_pub_->publish(std::move(msg));

  // 设置 robot_state_publisher 参数 → TF 树重建
  if (!param_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "robot_state_publisher 不可达");
    return;
  }
  param_client_->set_parameters(
    {rclcpp::Parameter("robot_description", urdf)},
    [this, tool_id](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      try {
        auto results = future.get();
        bool ok = !results.empty() && results[0].successful;
        RCLCPP_INFO(get_logger(), "robot_description 更新: %s → %s",
                    ok ? "OK" : "FAIL", tool_id.c_str());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "robot_description 更新异常");
      }
    });
}

}  // namespace tool_changer

// ═══════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tool_changer::SceneAttachWorker>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
