/**
 * @file scene_attach_worker.cpp
 * @brief 通过 AttachedCollisionObject + 动态 URDF 管理工具碰撞。
 *
 * 两层协作：
 *   1. AttachedCollisionObject (/planning_scene diff) — move_group 感知碰撞、用于规划避障
 *   2. URDF <collision> — RViz 视觉渲染 + robot_state_publisher TF
 *
 * 工具切换时：
 *   脱离：REMOVE AttachedCollisionObject + 更新空工具 URDF
 *   附着：ADD AttachedCollisionObject（网格附着到 kuaihuan_Link）+ 更新含工具 URDF
 */

#include "tool_changer/scene_attach_worker.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <resource_retriever/retriever.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cstring>
#include <map>
#include <sstream>
#include <tuple>

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

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// 构造函数
// ═══════════════════════════════════════════════════════════════════════

SceneAttachWorker::SceneAttachWorker(const rclcpp::NodeOptions& options)
  : rclcpp::Node("scene_attach_worker", options)
{
  // ── Publisher ──────────────────────────────────────────────────────
  planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(10).transient_local());

  robot_description_pub_ = create_publisher<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local());

  // ── Subscription ───────────────────────────────────────────────────
  tool_status_sub_ = create_subscription<ivg_interfaces::msg::ToolChangerStatus>(
      "/tool_changer_status", 10,
      [this](const ivg_interfaces::msg::ToolChangerStatus& msg) { onToolStatus(msg); });

  // ── Services ───────────────────────────────────────────────────────
  using namespace std::placeholders;
  scene_attach_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
      "/scene_attach", std::bind(&SceneAttachWorker::onSceneAttach, this, _1, _2));
  scene_detach_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
      "/scene_detach", std::bind(&SceneAttachWorker::onSceneDetach, this, _1, _2));

  // ── 加载工具配置 ──────────────────────────────────────────────────
  loadToolConfig();

  // ── 预生成 URDF 缓存 ──────────────────────────────────────────────
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "robot_state_publisher");

  const std::string xacro_path =
    ament_index_cpp::get_package_share_directory("aubo_moveit_config") +
    "/config/aubo_e5.urdf.xacro";

  for (const auto& [tid, geom] : tool_geometries_)
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

  RCLCPP_INFO(get_logger(), "就绪 | %zu 工具 | 监听 /tool_changer_status | /scene_attach /scene_detach",
              tool_geometries_.size());
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
    std::string tid = kv.first.as<std::string>();
    const auto& t = kv.second;
    ToolGeometry geom;

    // 加载碰撞网格
    geom.mesh_collision = loadMesh(t["mesh_collision"].as<std::string>());

    // 加载 attach_offset
    const auto& ao = t["attach_offset"];
    geom.attach_offset.position.x = ao["position"]["x"].as<double>();
    geom.attach_offset.position.y = ao["position"]["y"].as<double>();
    geom.attach_offset.position.z = ao["position"]["z"].as<double>();
    geom.attach_offset.orientation.x = ao["orientation"]["x"].as<double>();
    geom.attach_offset.orientation.y = ao["orientation"]["y"].as<double>();
    geom.attach_offset.orientation.z = ao["orientation"]["z"].as<double>();
    geom.attach_offset.orientation.w = ao["orientation"]["w"].as<double>();

    // 加载 touch_links
    if (t["touch_links"])
      for (const auto& link : t["touch_links"])
        geom.touch_links.push_back(link.as<std::string>());

    tool_geometries_[tid] = geom;
  }
  RCLCPP_INFO(get_logger(), "已加载 %zu 个工具配置", tool_geometries_.size());
}

// ═══════════════════════════════════════════════════════════════════════
// loadMesh — 二进制 STL 解析（带顶点去重）
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

    // ASCII STL 回退
    if (res.size >= 5 && std::memcmp(data, "solid", 5) == 0) {
      RCLCPP_WARN(get_logger(), "loadMesh: ASCII STL 不支持: %s", resource_path.c_str());
      return msg;
    }

    uint32_t num_triangles = *reinterpret_cast<const uint32_t*>(data + 80);
    size_t expected_size = 84 + static_cast<size_t>(num_triangles) * 50;
    if (res.size < expected_size) {
      RCLCPP_ERROR(get_logger(),
        "loadMesh: STL 大小不匹配 (expected %zu, got %zu): %s",
        expected_size, res.size, resource_path.c_str());
      return msg;
    }

    // 顶点去重
    using VertexKey = std::tuple<double, double, double>;
    std::map<VertexKey, uint32_t> vertex_map;
    std::vector<geometry_msgs::msg::Point> unique_verts;
    std::vector<shape_msgs::msg::MeshTriangle> triangles;
    triangles.reserve(num_triangles);

    const uint8_t* tri_base = data + 84;

    for (uint32_t t = 0; t < num_triangles; ++t) {
      shape_msgs::msg::MeshTriangle tri;
      const uint8_t* tb = tri_base + static_cast<size_t>(t) * 50;

      for (int v = 0; v < 3; ++v) {
        const uint8_t* vb = tb + 12 + static_cast<size_t>(v) * 12;
        float fx, fy, fz;
        std::memcpy(&fx, vb, 4);
        std::memcpy(&fy, vb + 4, 4);
        std::memcpy(&fz, vb + 8, 4);

        VertexKey key(static_cast<double>(fx),
                       static_cast<double>(fy),
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

    if (!msg.vertices.empty()) {
      double xmin = msg.vertices[0].x, xmax = msg.vertices[0].x;
      double ymin = msg.vertices[0].y, ymax = msg.vertices[0].y;
      double zmin = msg.vertices[0].z, zmax = msg.vertices[0].z;
      for (const auto& v : msg.vertices) {
        if (v.x < xmin) xmin = v.x;
        if (v.x > xmax) xmax = v.x;
        if (v.y < ymin) ymin = v.y;
        if (v.y > ymax) ymax = v.y;
        if (v.z < zmin) zmin = v.z;
        if (v.z > zmax) zmax = v.z;
      }
      RCLCPP_INFO(get_logger(),
        "loadMesh: %s (%zu△→%zu△, %zu 顶点) X[%.4f,%.4f] Y[%.4f,%.4f] Z[%.4f,%.4f]",
        resource_path.c_str(), static_cast<size_t>(num_triangles),
        msg.triangles.size(), msg.vertices.size(),
        xmin, xmax, ymin, ymax, zmin, zmax);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "loadMesh: %s — %s", resource_path.c_str(), e.what());
  }
  return msg;
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

  // 脱离旧工具
  if (!current_attached_tool_.empty())
    detachToolFromScene(current_attached_tool_);

  // 附着新工具
  if (!new_tool.empty())
    attachToolToScene(new_tool);

  // 更新 URDF（仅在附着新工具时）
  updateRobotDescription(new_tool);

  current_attached_tool_ = new_tool;
}

// ═══════════════════════════════════════════════════════════════════════
// AttachedCollisionObject 管理
// ═══════════════════════════════════════════════════════════════════════

void SceneAttachWorker::attachToolToScene(const std::string& tool_id)
{
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) {
    RCLCPP_WARN(get_logger(), "attachTool: 未知工具 %s", tool_id.c_str());
    return;
  }

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;

  auto att = moveit_msgs::msg::AttachedCollisionObject{};
  att.object.id = "attached_tool_" + tool_id;
  att.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  att.link_name = "kuaihuan_Link";
  att.touch_links = it->second.touch_links;

  att.object.meshes.push_back(it->second.mesh_collision);
  auto mesh_pose = geometry_msgs::msg::Pose{};
  mesh_pose.position.x = 0.0;
  mesh_pose.position.y = 0.0;
  mesh_pose.position.z = 0.0;
  mesh_pose.orientation = it->second.attach_offset.orientation;
  att.object.mesh_poses.push_back(mesh_pose);

  scene.robot_state.attached_collision_objects.push_back(att);

  planning_scene_pub_->publish(scene);
  RCLCPP_INFO(get_logger(), "AttachedCollisionObject ADD: %s → kuaihuan_Link", tool_id.c_str());
}

void SceneAttachWorker::detachToolFromScene(const std::string& tool_id)
{
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) return;

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;

  auto att = moveit_msgs::msg::AttachedCollisionObject{};
  att.object.id = "attached_tool_" + tool_id;
  att.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  att.link_name = "kuaihuan_Link";

  scene.robot_state.attached_collision_objects.push_back(att);

  planning_scene_pub_->publish(scene);
  RCLCPP_INFO(get_logger(), "AttachedCollisionObject REMOVE: %s", tool_id.c_str());
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
    if (!current_attached_tool_.empty())
      detachToolFromScene(current_attached_tool_);
    attachToolToScene(req->tool_id);
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
    detachToolFromScene(req->tool_id);
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

  // 发布到 /robot_description → RViz2 重载
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
