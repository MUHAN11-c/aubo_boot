// 场景附着 Worker 实现 — 源自 aubo_boot tool_changer（2026-09-30 移植）。
// 发布两类增量（QoS depth=10 + transient_local）：
//   /attached_collision_object — ADD/REMOVE AttachedCollisionObject
//   /planning_scene（is_diff）— world REMOVE attached_tool_<id> 清残留
#include "tool_changer/scene_attach_worker.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <resource_retriever/retriever.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <tuple>

namespace tool_changer
{

SceneAttachWorker::SceneAttachWorker(const rclcpp::NodeOptions & options)
: rclcpp::Node("scene_attach_worker", options)
{
  attach_frame_ = declare_parameter<std::string>("attach_frame", "kuaihuan_Link");
  modeled_tool_ = declare_parameter<std::string>("modeled_tool", "none");

  planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
    "/planning_scene", rclcpp::QoS(10).transient_local());
  attached_object_pub_ = create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
    "/attached_collision_object", rclcpp::QoS(10).transient_local());

  tool_status_sub_ = create_subscription<ivg_interfaces::msg::ToolChangerStatus>(
    "/tool_changer_status", 10,
    [this](const ivg_interfaces::msg::ToolChangerStatus & msg) {onToolStatus(msg);});

  scene_attach_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
    "/scene_attach",
    [this](const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp) {
      onSceneAttach(req, resp);
    });
  scene_detach_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
    "/scene_detach",
    [this](const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp) {
      onSceneDetach(req, resp);
    });
  display_tool_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
    "/set_display_tool",
    [this](const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp) {
      onSetDisplayTool(req, resp);
    });

  loadToolConfig();


  RCLCPP_INFO(get_logger(), "就绪 | attach_frame=%s | modeled_tool=%s | ACO only",
    attach_frame_.c_str(), modeled_tool_.c_str());
}

// ═════════════════════════════════ 配置加载 ═════════════════════════════════

void SceneAttachWorker::loadToolConfig()
{
  std::string config_path;
  try {
    config_path =
      ament_index_cpp::get_package_share_directory("ivg_description") + "/config/tools.yaml";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "tool_changer 包路径: %s", e.what());
    return;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "YAML 加载失败 %s: %s", config_path.c_str(), e.what());
    return;
  }

  auto tools = config["tools"];
  if (!tools) {
    RCLCPP_ERROR(get_logger(), "缺少 'tools' 节点");
    return;
  }

  for (const auto & kv : tools) {
    std::string tid = kv.first.as<std::string>();
    const auto & t = kv.second;
    ToolGeometry geom;

    geom.mesh_collision = loadMesh(t["mesh_collision"].as<std::string>());

    const auto & ao = t["attach_offset"];
    geom.attach_offset.position.x = ao["position"]["x"].as<double>();
    geom.attach_offset.position.y = ao["position"]["y"].as<double>();
    geom.attach_offset.position.z = ao["position"]["z"].as<double>();
    geom.attach_offset.orientation.x = ao["orientation"]["x"].as<double>();
    geom.attach_offset.orientation.y = ao["orientation"]["y"].as<double>();
    geom.attach_offset.orientation.z = ao["orientation"]["z"].as<double>();
    geom.attach_offset.orientation.w = ao["orientation"]["w"].as<double>();

    if (t["touch_links"]) {
      for (const auto & link : t["touch_links"]) {
        geom.touch_links.push_back(link.as<std::string>());
      }
    }

    tool_geometries_[tid] = geom;
  }
  RCLCPP_INFO(get_logger(), "已加载 %zu 个工具配置", tool_geometries_.size());
}

// ═════════════════════════════════ STL 解析 ═════════════════════════════════

shape_msgs::msg::Mesh SceneAttachWorker::loadMesh(const std::string & resource_path)
{
  shape_msgs::msg::Mesh msg;
  try {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res = retriever.get(resource_path);

    if (res.size < 84) {
      RCLCPP_ERROR(
        get_logger(), "loadMesh: 文件太小 (%zu bytes): %s",
        res.size, resource_path.c_str());
      return msg;
    }

    const uint8_t * data = res.data.get();

    if (res.size >= 5 && std::memcmp(data, "solid", 5) == 0) {
      RCLCPP_WARN(get_logger(), "loadMesh: ASCII STL 不支持: %s", resource_path.c_str());
      return msg;
    }

    uint32_t num_triangles = *reinterpret_cast<const uint32_t *>(data + 80);
    size_t expected_size = 84 + static_cast<size_t>(num_triangles) * 50;
    if (res.size < expected_size) {
      RCLCPP_ERROR(
        get_logger(), "loadMesh: STL 大小不匹配 (expected %zu, got %zu): %s",
        expected_size, res.size, resource_path.c_str());
      return msg;
    }

    // 顶点去重
    using VertexKey = std::tuple<double, double, double>;
    std::map<VertexKey, uint32_t> vertex_map;
    std::vector<geometry_msgs::msg::Point> unique_verts;
    std::vector<shape_msgs::msg::MeshTriangle> triangles;
    triangles.reserve(num_triangles);

    const uint8_t * tri_base = data + 84;

    for (uint32_t t = 0; t < num_triangles; ++t) {
      shape_msgs::msg::MeshTriangle tri;
      const uint8_t * tb = tri_base + static_cast<size_t>(t) * 50;

      for (int v = 0; v < 3; ++v) {
        const uint8_t * vb = tb + 12 + static_cast<size_t>(v) * 12;
        float fx, fy, fz;
        std::memcpy(&fx, vb, 4);
        std::memcpy(&fy, vb + 4, 4);
        std::memcpy(&fz, vb + 8, 4);

        VertexKey key(static_cast<double>(fx), static_cast<double>(fy),
          static_cast<double>(fz));

        auto it = vertex_map.find(key);
        if (it != vertex_map.end()) {
          tri.vertex_indices[v] = it->second;
        } else {
          uint32_t new_idx = static_cast<uint32_t>(unique_verts.size());
          vertex_map[key] = new_idx;
          tri.vertex_indices[v] = new_idx;

          geometry_msgs::msg::Point pt;
          pt.x = fx;
          pt.y = fy;
          pt.z = fz;
          unique_verts.push_back(pt);
        }
      }
      triangles.push_back(tri);
    }

    msg.vertices = std::move(unique_verts);
    msg.triangles = std::move(triangles);
    RCLCPP_INFO(
      get_logger(), "loadMesh: %s (%zu 顶点, %zu 三角形)",
      resource_path.c_str(), msg.vertices.size(), msg.triangles.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "loadMesh: %s — %s", resource_path.c_str(), e.what());
  }
  return msg;
}

// ═════════════════════════════════ 状态同步 ═════════════════════════════════

void SceneAttachWorker::onToolStatus(const ivg_interfaces::msg::ToolChangerStatus & msg)
{
  if (modeled_tool_ != "none") {return;}
  const std::string new_tool = msg.is_connected ? msg.tool_id : "";

  if (new_tool == current_attached_tool_) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "状态变更: %s → %s (connected=%s)",
    current_attached_tool_.c_str(), new_tool.c_str(),
    msg.is_connected ? "true" : "false");

  if (!current_attached_tool_.empty()) {
    detachToolFromScene(current_attached_tool_);
  }
  if (!new_tool.empty()) {
    attachToolToScene(new_tool);
  }
  current_attached_tool_ = new_tool;
}

void SceneAttachWorker::attachToolToScene(const std::string & tool_id)
{
  if (modeled_tool_ != "none") {
    RCLCPP_WARN(get_logger(), "固定模型 %s 不接受运行时工具附着；请以 gripper:=none 启动快换栈",
      modeled_tool_.c_str());
    return;
  }

  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) {
    RCLCPP_WARN(get_logger(), "attachTool: 未知工具 %s", tool_id.c_str());
    return;
  }
  removeWorldToolObject(tool_id);

  moveit_msgs::msg::AttachedCollisionObject att;
  att.object.id = "attached_tool_" + tool_id;
  att.object.header.frame_id = attach_frame_;
  att.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  att.object.pose = it->second.attach_offset;
  att.link_name = attach_frame_;
  att.touch_links = it->second.touch_links;

  att.object.meshes.push_back(it->second.mesh_collision);
  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.orientation.w = 1.0;
  att.object.mesh_poses.push_back(mesh_pose);

  attached_object_pub_->publish(att);
  removeWorldToolObject(tool_id);
  RCLCPP_INFO(
    get_logger(), "AttachedCollisionObject ADD: %s → %s | offset xyz=(%.4f, %.4f, %.4f)",
    tool_id.c_str(), attach_frame_.c_str(),
    it->second.attach_offset.position.x,
    it->second.attach_offset.position.y,
    it->second.attach_offset.position.z);
}

void SceneAttachWorker::detachToolFromScene(const std::string & tool_id)
{
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) {return;}

  moveit_msgs::msg::AttachedCollisionObject att;
  att.object.id = "attached_tool_" + tool_id;
  att.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  att.link_name = attach_frame_;

  attached_object_pub_->publish(att);
  removeWorldToolObject(tool_id);
  RCLCPP_INFO(get_logger(), "AttachedCollisionObject REMOVE: %s", tool_id.c_str());
}

void SceneAttachWorker::removeWorldToolObject(const std::string & tool_id)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  moveit_msgs::msg::CollisionObject obj;
  obj.id = "attached_tool_" + tool_id;
  obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene.world.collision_objects.push_back(obj);

  planning_scene_pub_->publish(scene);
}

// ═════════════════════════════════ 服务回调 ═════════════════════════════════

void SceneAttachWorker::onSceneAttach(
  const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
  std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp)
{
  bool found = tool_geometries_.find(req->tool_id) != tool_geometries_.end();
  if (modeled_tool_ != "none") {
    resp->success = false;
    resp->message = "启动模型已包含固定工具；运行时附着要求 gripper:=none";
    return;
  }
  if (found) {
    if (!current_attached_tool_.empty()) {
      detachToolFromScene(current_attached_tool_);
    }
    attachToolToScene(req->tool_id);
    current_attached_tool_ = req->tool_id;
  }
  resp->success = found;
  resp->message = found ? ("附着: " + req->tool_id) : ("未知工具: " + req->tool_id);
}

void SceneAttachWorker::onSceneDetach(
  const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
  std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp)
{
  bool found = tool_geometries_.find(req->tool_id) != tool_geometries_.end();
  if (found && current_attached_tool_ == req->tool_id) {
    detachToolFromScene(req->tool_id);
    current_attached_tool_.clear();
  }
  resp->success = found;
  resp->message = found ? ("脱离: " + req->tool_id) : ("未知工具: " + req->tool_id);
}

void SceneAttachWorker::onSetDisplayTool(
  const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
  std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp)
{
  if (modeled_tool_ != "none") {
    resp->success = req->tool_id == modeled_tool_;
    resp->message = resp->success ? "该工具已在启动模型中" :
      "固定模型须停栈后通过 gripper 参数切换";
    return;
  }
  // A display request must never remove or replace the physically attached tool.
  const std::string requested = req->tool_id == "none" ? "" : req->tool_id;
  resp->success = requested == current_attached_tool_;
  resp->message = resp->success ? "工具状态已一致；机器人描述不变" :
    "显示模型由启动参数选择；碰撞附着只随实际工具状态更新";
}

}  // namespace tool_changer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tool_changer::SceneAttachWorker>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
