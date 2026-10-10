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
  attach_frame_ = declare_parameter<std::string>("attach_frame", "quick_changer_link");

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
  loadUrdfCache();

  // 独立回调组，避免 set_parameters 与 onSetDisplayTool 死锁
  param_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    get_node_base_interface(),
    get_node_topics_interface(),
    get_node_graph_interface(),
    get_node_services_interface(),
    "robot_state_publisher",
    rmw_qos_profile_parameters,
    param_cb_group_);

  // 发布 /robot_description（RSP dynamic reload 话题 + 前端 3D 重载）
  robot_description_pub_ = create_publisher<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(
    get_logger(),
    "就绪 | %zu 工具 | attach_frame=%s | ACO + /robot_description(URDF) | "
    "sub /tool_changer_status | srv /scene_attach /scene_detach /set_display_tool",
    tool_geometries_.size(), attach_frame_.c_str());
}

// ═════════════════════════════════ 配置加载 ═════════════════════════════════

void SceneAttachWorker::loadToolConfig()
{
  std::string config_path;
  try {
    config_path =
      ament_index_cpp::get_package_share_directory("tool_changer") + "/config/tools.yaml";
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

// ═════════════════════════════════ URDF 缓存 ═════════════════════════════════

/// 读取本包 urdf/ 下 vendored 静态 URDF（aubo_e5_<tool>.urdf；空 id = base）
void SceneAttachWorker::loadUrdfCache()
{
  std::string urdf_dir;
  try {
    urdf_dir = ament_index_cpp::get_package_share_directory("tool_changer") + "/urdf";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "tool_changer 包路径: %s", e.what());
    return;
  }

  auto load_file = [&](const std::string & tid) {
      std::string fname = tid.empty() ? "aubo_e5_base.urdf" :
        "aubo_e5_" + tid + ".urdf";
      std::ifstream ifs(urdf_dir + "/" + fname);
      if (!ifs.is_open()) {
        return std::string();
      }
      std::ostringstream oss;
      oss << ifs.rdbuf();
      return oss.str();
    };

  urdf_cache_[""] = load_file("");
  for (const auto & [tid, geom] : tool_geometries_) {
    (void)geom;
    std::string urdf = load_file(tid);
    if (!urdf.empty()) {
      urdf_cache_[tid] = urdf;
      RCLCPP_INFO(get_logger(), "URDF 缓存: %s (%zu bytes)", tid.c_str(), urdf.size());
    } else {
      RCLCPP_WARN(
        get_logger(), "URDF 文件缺失: %s（/set_display_tool 对该工具降级）", tid.c_str());
    }
  }
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
  updateRobotDescription(tool_id);
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
  updateRobotDescription("");
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
  // 双重更新：URDF（/robot_description → 前端 + RViz2）+ ACO（PlanningScene 显示）
  bool found = urdf_cache_.find(req->tool_id) != urdf_cache_.end();
  if (found) {
    if (!current_display_tool_.empty() && current_display_tool_ != current_attached_tool_) {
      moveit_msgs::msg::AttachedCollisionObject att;
      att.object.id = "attached_tool_" + current_display_tool_;
      att.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      att.link_name = attach_frame_;
      attached_object_pub_->publish(att);
      removeWorldToolObject(current_display_tool_);
    }

    if (!req->tool_id.empty() && req->tool_id != current_attached_tool_) {
      auto it = tool_geometries_.find(req->tool_id);
      if (it != tool_geometries_.end()) {
        removeWorldToolObject(req->tool_id);
        moveit_msgs::msg::AttachedCollisionObject att;
        att.object.id = "attached_tool_" + req->tool_id;
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
        removeWorldToolObject(req->tool_id);
      }
    }

    updateRobotDescription(req->tool_id, true);

    current_display_tool_ = req->tool_id;
    RCLCPP_INFO(
      get_logger(), "显示工具: %s (URDF + ACO)",
      req->tool_id.empty() ? "(无工具)" : req->tool_id.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "显示工具失败， URDF 缓存未命中: %s", req->tool_id.c_str());
  }
  resp->success = found;
  resp->message = found ?
    ("已更新: " + (req->tool_id.empty() ? std::string("(无工具)") : req->tool_id)) :
    ("未知工具 ID: " + req->tool_id);
}

// ═════════════════════════════════ robot_description 更新 ═════════════════════════════════

void SceneAttachWorker::updateRobotDescription(const std::string & tool_id, bool sync)
{
  auto it = urdf_cache_.find(tool_id);
  if (it == urdf_cache_.end()) {
    RCLCPP_WARN(get_logger(), "URDF 缓存未命中: '%s'", tool_id.c_str());
    return;
  }

  std_msgs::msg::String msg;
  msg.data = it->second;
  robot_description_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "URDF 已发布到 /robot_description: %s (%zu bytes)",
    tool_id.empty() ? "(default)" : tool_id.c_str(), it->second.size());

  if (!param_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "robot_state_publisher 参数服务未就绪，跳过 set_parameters");
    return;
  }

  if (sync) {
    auto future = param_client_->set_parameters(
      {rclcpp::Parameter("robot_description", it->second)});
    auto status = future.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "robot_state_publisher set_parameters 超时");
      return;
    }
    auto results = future.get();
    for (const auto & r : results) {
      if (!r.successful) {
        RCLCPP_ERROR(
          get_logger(), "robot_state_publisher set_parameters 失败: %s",
          r.reason.c_str());
      }
    }
  } else {
    param_client_->set_parameters(
      {rclcpp::Parameter("robot_description", it->second)},
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        auto results = future.get();
        for (const auto & r : results) {
          if (!r.successful) {
            RCLCPP_ERROR(
              get_logger(), "robot_state_publisher set_parameters 失败: %s",
              r.reason.c_str());
          }
        }
      });
  }
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
