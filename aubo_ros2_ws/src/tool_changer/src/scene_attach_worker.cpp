/**
 * @file scene_attach_worker.cpp
 * @brief 独立节点：订阅 /tool_changer_status，更新 PlanningScene 工具附着显示
 *
 * ======== 这个文件是干什么的？ ========
 * 这是一个 ROS2 节点（Node），负责在 MoveIt 的"规划场景"中显示/隐藏工具的碰撞模型。
 *
 * 背景概念：
 * - ROS2 节点：ROS2 程序的基本运行单元，可以理解为一个独立运行的"小程序"。
 *   一个机器人系统通常由多个节点组成，节点之间通过"话题"和"服务"通信。
 *
 * - MoveIt：ROS 生态中最常用的机械臂运动规划框架。它内部维护一个"规划场景
 *   (PlanningScene)"，记录当前有哪些物体、机械臂在什么位置，用于碰撞检测和轨迹规划。
 *
 * - 碰撞对象(CollisionObject)：MoveIt 中用来表示一个物体的数据结构，包含物体的
 *   形状（网格/Mesh）和位置（Pose），MoveIt 用它来判断机械臂会不会撞到这个物体。
 *
 * 这个节点的核心任务：
 * 1. 启动时从 YAML 配置文件加载所有工具的 3D 碰撞模型
 * 2. 把所有工具的模型发布到 PlanningScene 中（显示在 "dock" 停靠位置）
 * 3. 监听 /tool_changer_status 话题，当工具被装上/卸下时，自动把该工具的碰撞模型
 *    从"世界"移到"机械臂上"或从"机械臂上"移回"世界"
 *
 * 为什么需要这个？—— 机械臂装上一个工具后，工具的模型必须"附着"到机械臂末端
 * 关节（kuaihuan_Link）上，这样 MoveIt 才知道"工具现在跟着手臂动了"，才能正确
 * 做碰撞检测和轨迹规划。
 */

// ============================================================
// 头文件包含（引入需要的库和类型定义）
// ============================================================

#include "tool_changer/scene_attach_worker.h"  // 本类的头文件声明

#include <ament_index_cpp/get_package_share_directory.hpp>  // 用于查找 ROS2 包的安装路径
#include <yaml-cpp/yaml.h>                                   // 解析 YAML 配置文件

#include <resource_retriever/retriever.hpp>  // 解析 package:// 资源路径
#include <cstdio>   // popen, pclose
#include <cstring>   // std::memcpy
#include <map>       // std::map
#include <sstream>   // std::ostringstream
#include <tuple>     // std::tuple
#include <vector>

namespace tool_changer  // 命名空间：把代码归类到 tool_changer 下，避免与其他库的名字冲突
{

// ═══════════════════════════════════════════════════════════════════════
// 构造函数 —— 节点启动时执行一次
// ═══════════════════════════════════════════════════════════════════════

/**
 * 构造函数详解：
 *
 * 每个 ROS2 节点在被创建时，首先调用构造函数进行初始化。
 * 这里主要做四件事：
 *   1. 创建发布者(Publisher)   —— 向 MoveIt 发送场景更新
 *   2. 创建订阅者(Subscription) —— 接收工具状态变化通知
 *   3. 创建服务(Service)        —— 提供手动附着/脱离的接口
 *   4. 加载配置并初始化场景     —— 把工具模型加到世界中
 *
 * 参数 options：ROS2 节点的可选配置（如名称重映射、参数覆盖等），
 *              这里传给父类 rclcpp::Node 的构造函数。
 */
SceneAttachWorker::SceneAttachWorker(const rclcpp::NodeOptions& options)
  : rclcpp::Node("scene_attach_worker", options)  // 调用父类构造函数，节点名称设为 "scene_attach_worker"
{
  // ─── 1. 创建 PlanningScene 发布者 ─────────────────────────────────
  // 发布者(Publisher)：用来向某个"话题"发送消息。
  // 话题(Topic)：ROS2 的通信机制之一，一个节点"发"(publish)，多个节点"收"(subscribe)。
  //
  // 这里创建的是向 "/planning_scene" 话题发送 moveit_msgs::msg::PlanningScene 消息的发布者。
  //
  // QoS(10).transient_local()：
  //   - QoS(Quality of Service)：ROS2 的服务质量策略，控制消息传输的可靠性
  //   - 10：消息队列深度，最多缓存 10 条未发出的消息
  //   - transient_local：短暂本地持久化 —— 意思是新订阅者上线时可以收到
  //     最近一条消息（即使消息是在它订阅之前发的）。这样 MoveIt 重启后
  //     仍然能收到当前场景的完整状态，不会丢失工具模型。
  planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(10).transient_local());

  // 发布 URDF 到 /robot_description 话题（供 RViz2 / move_group 动态重载）
  robot_description_pub_ = create_publisher<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local());

  // ─── 2. 创建工具状态订阅者 ────────────────────────────────────────
  // 订阅者(Subscription)：用来接收某个"话题"的消息。
  // 这里订阅 "/tool_changer_status" 话题，当物理工具快换装置状态变化时
  // （比如装上或卸下一个工具），会收到通知。
  //
  // 第三个参数是一个 Lambda 表达式（匿名函数）：
  //   [this](const ... & msg) { onToolStatus(msg); }
  //   - [this]：捕获当前对象的 this 指针，以便在回调中访问成员函数
  //   - 收到消息后，调用 onToolStatus() 来处理状态变化
  tool_status_sub_ = create_subscription<ivg_interfaces::msg::ToolChangerStatus>(
      "/tool_changer_status", 10,  // 队列深度 10
      [this](const ivg_interfaces::msg::ToolChangerStatus& msg) { onToolStatus(msg); });

  // ─── 3. 创建手动附着服务 ─────────────────────────────────────────
  // 服务(Service)：ROS2 的另一种通信机制，与 Topic 的"单向推送"不同，
  // Service 是"请求-响应"模式：客户端发送请求，服务端处理后返回响应。
  //
  // 这里创建两个服务，允许外部节点手动触发工具的附着/脱离：
  //
  // /scene_attach 服务：手动让某个工具的碰撞模型附着到机械臂上
  //   std::bind(&SceneAttachWorker::onSceneAttach, this, _1, _2)
  //     - std::bind 把成员函数 onSceneAttach 绑定为回调
  //     - _1, _2 是占位符，分别对应 Request 和 Response
  scene_attach_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
      "/scene_attach",
      std::bind(&SceneAttachWorker::onSceneAttach, this, std::placeholders::_1, std::placeholders::_2));

  // /scene_detach 服务：手动让某个工具的碰撞模型脱离机械臂，回到世界 dock
  scene_detach_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
      "/scene_detach",
      std::bind(&SceneAttachWorker::onSceneDetach, this, std::placeholders::_1, std::placeholders::_2));

  // ─── 4. 加载配置 & 初始化场景 ─────────────────────────────────────
  loadToolConfig();       // 从 tools.yaml 读取所有工具的网格模型和位姿信息
  addAllToolsToWorld();   // 把所有工具发布到 PlanningScene 的 world 中（初始状态在 dock 位置）

  RCLCPP_INFO(get_logger(), "就绪 | 监听 /tool_changer_status | /scene_attach /scene_detach | %zu 工具",
              tool_geometries_.size());

  // ─── 5. 预生成每个工具的 URDF 并缓存 ─────────────────────────────
  // 工具切换时直接取缓存，避免重复调用 xacro
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "robot_state_publisher");
  const std::string xacro_path =
    ament_index_cpp::get_package_share_directory("aubo_moveit_config") +
    "/config/aubo_e5.urdf.xacro";
  for (const auto& [tid, geom] : tool_geometries_)
  {
    std::ostringstream cmd;
    cmd << "xacro " << xacro_path << " gripper:=" << tid
        << " use_fake_hardware:=true 2>/dev/null";
    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) continue;
    std::string urdf;
    char buf[4096];
    while (fgets(buf, sizeof(buf), pipe)) urdf += buf;
    pclose(pipe);
    if (!urdf.empty()) {
      urdf_cache_[tid] = urdf;
      RCLCPP_INFO(get_logger(), "URDF 缓存: %s (%zu bytes)", tid.c_str(), urdf.size());
    }
  }
  // 默认 URDF (无夹爪)
  {
    std::ostringstream cmd;
    cmd << "xacro " << xacro_path << " use_fake_hardware:=true 2>/dev/null";
    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (pipe) {
      std::string urdf;
      char buf[4096];
      while (fgets(buf, sizeof(buf), pipe)) urdf += buf;
      pclose(pipe);
      if (!urdf.empty()) urdf_cache_[""] = urdf;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
// 配置加载
// ═══════════════════════════════════════════════════════════════════

/**
 * loadToolConfig —— 从 YAML 配置文件加载所有工具的信息
 *
 * YAML 文件格式示例：
 *   tools:
 *     gripper_a:                          # 工具 ID
 *       mesh_collision: "package://tool_changer/meshes/gripper_a.stl"  # 碰撞网格文件路径
 *       dock_pose:                        # 工具在 dock 上的存放位置（相对 base_link）
 *         position: {x: 0.5, y: 0.0, z: 0.2}
 *       attach_offset:                    # 工具装上后相对 kuaihuan_Link 的偏移
 *         position: {x: 0.0, y: 0.0, z: 0.1}
 *       touch_links: ["link1", "link2"]   # 附着时需要忽略碰撞检测的关节
 *
 * 执行流程：
 *   1. 找到 tool_changer 包的安装路径
 *   2. 用 yaml-cpp 库解析 YAML 文件
 *   3. 遍历每个工具，加载网格，读取位姿，存入 tool_geometries_ 成员变量
 */
void SceneAttachWorker::loadToolConfig()
{
  // ─── 步骤 1：获取配置文件路径 ─────────────────────────────────────
  // ament_index_cpp::get_package_share_directory("tool_changer")
  //   返回 tool_changer 包的 "share" 目录，ROS2 把包的数据文件安装在这里。
  //   例如：/home/user/ros2_ws/install/tool_changer/share/tool_changer/
  std::string config_path;
  try {
    config_path = ament_index_cpp::get_package_share_directory("tool_changer") + "/config/tools.yaml";
  } catch (const std::exception& e) {  // 如果包没找到（没有安装或没 source 环境）
    RCLCPP_ERROR(get_logger(), "tool_changer 包路径: %s", e.what());
    return;  // 加载失败就直接返回，后面的代码不再执行
  }

  // ─── 步骤 2：解析 YAML 文件 ───────────────────────────────────────
  // YAML::LoadFile() 读取并解析文件，返回 YAML::Node（树状结构）
  // 如果文件不存在或格式错误会抛异常
  YAML::Node config;
  try { config = YAML::LoadFile(config_path); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "YAML 加载失败 %s: %s", config_path.c_str(), e.what());
    return;
  }

  // ─── 步骤 3：获取 "tools" 节点 ────────────────────────────────────
  // config["tools"] 获取 YAML 中 tools 键对应的内容（一个 map）
  auto tools = config["tools"];
  if (!tools) {  // 如果 tools 键不存在或类型不对，tools 为"空节点"
    RCLCPP_ERROR(get_logger(), "缺少 'tools' 节点");
    return;
  }

  // ─── 步骤 4：遍历每个工具 ─────────────────────────────────────────
  // YAML 的 tools 是一个 map（键值对集合），kv 的 first 是键(工具ID)，second 是值(配置信息)
  for (const auto& kv : tools)
  {
    // 工具 ID（如 "gripper_a"），作为 map 的 key
    std::string tid = kv.first.as<std::string>();
    const auto& t = kv.second;  // 工具的具体配置（map 的 value）
    ToolGeometry geom;  // 新建一个几何信息结构体，存放这个工具的所有数据

    // 4a. 加载碰撞网格文件
    //     t["mesh_collision"] 取 mesh_collision 键的值
    //     .as<std::string>() 把它转成 C++ 的 string 类型
    //     loadMesh() 调用下面的函数从文件加载 3D 网格数据
    geom.mesh_collision = loadMesh(t["mesh_collision"].as<std::string>());

    // 4b. 加载 dock_pose（工具在 dock 上的位姿）
    //     Pose 包含 position(x, y, z) 和 orientation(四元数旋转)
    //     这里只设置位置，方向用默认的单位四元数 (w=1, x=y=z=0) 表示不旋转
    const auto& dp = t["dock_pose"];              // dp = 配置中的 dock_pose 节点
    geom.dock_pose.position.x = dp["position"]["x"].as<double>();
    geom.dock_pose.position.y = dp["position"]["y"].as<double>();
    geom.dock_pose.position.z = dp["position"]["z"].as<double>();
    geom.dock_pose.orientation.x = dp["orientation"]["x"].as<double>();
    geom.dock_pose.orientation.y = dp["orientation"]["y"].as<double>();
    geom.dock_pose.orientation.z = dp["orientation"]["z"].as<double>();
    geom.dock_pose.orientation.w = dp["orientation"]["w"].as<double>();

    // 4c. 加载 attach_offset（工具附着时相对 kuaihuan_Link 的偏移）
    //     当工具被装到机械臂上时，工具的网格模型需要挂到末端关节上，
    //     但不是直接放在关节原点，而是有一个偏移（比如夹爪的安装点
    //     和夹爪重心之间有距离）
    const auto& ao = t["attach_offset"];          // ao = 配置中的 attach_offset 节点
    geom.attach_offset.position.x = ao["position"]["x"].as<double>();
    geom.attach_offset.position.y = ao["position"]["y"].as<double>();
    geom.attach_offset.position.z = ao["position"]["z"].as<double>();
    geom.attach_offset.orientation.x = ao["orientation"]["x"].as<double>();
    geom.attach_offset.orientation.y = ao["orientation"]["y"].as<double>();
    geom.attach_offset.orientation.z = ao["orientation"]["z"].as<double>();
    geom.attach_offset.orientation.w = ao["orientation"]["w"].as<double>();

    // 4d. 加载 touch_links（"触碰关节"列表）
    //     touch_links 是一个可选列表，列出附着时需要忽略碰撞检测的关节。
    //     比如工具已经装在 kuaihuan_Link 上，工具和 kuaihuan_Link 之间
    //     就不应该被判定为"碰撞"，所以把相关关节列入 touch_links。
    //     MoveIt 会对 touch_links 中的关节跳过工具与手臂之间的碰撞检测。
    if (t["touch_links"])  // 先判断有没有这个键
      for (const auto& tl : t["touch_links"])
        geom.touch_links.push_back(tl.as<std::string>());

    // 4e. 把加载好的工具信息存入 map
    //     tool_geometries_[tid] = geom:
    //       std::map 的 [] 运算符：如果 key(tid) 已存在则覆盖，不存在则插入
    //       之后通过 tool_geometries_["gripper_a"] 就可以拿到该工具的几何信息
    tool_geometries_[tid] = geom;

    // 打印加载成功日志：
    //   %s —— C 风格字符串
    //   %zu —— size_t 类型（三角形数量）
    RCLCPP_INFO(get_logger(), "加载: %s (%zu△)", tid.c_str(), geom.mesh_collision.triangles.size());
  }
}

/**
 * loadMesh —— 从资源路径加载二进制 STL 网格，直接转为 ROS Mesh 消息
 *
 * 绕过 Assimp 的矩阵乘法顶点变换管线（aiMatrix4x4 * vertex），
 * 避免浮点精度漂移导致大型网格（如 26K△ 的夹爪）在后续顶点去重时
 * 产生不正确的三角形索引映射。
 *
 * 格式：二进制 STL（80 字节头 + 4 字节三角数 LE + N×50 字节三角数据）
 * 顶点去重：使用 std::map 精确比较（与 geometric_shapes 的 std::set 行为一致）
 */
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

    // 检测 ASCII STL（以 "solid" 开头），回退到 Assimp 路径
    if (res.size >= 5 && std::memcmp(data, "solid", 5) == 0) {
      RCLCPP_WARN(get_logger(), "loadMesh: ASCII STL 不支持直接解析，回退到 Assimp: %s", resource_path.c_str());
      return msg;  // 返回空网格，调用方会记录日志
    }

    uint32_t num_triangles = *reinterpret_cast<const uint32_t*>(data + 80);
    size_t expected_size = 84 + static_cast<size_t>(num_triangles) * 50;
    if (res.size < expected_size) {
      RCLCPP_ERROR(get_logger(),
        "loadMesh: STL 大小不匹配 (expected %zu, got %zu): %s",
        expected_size, res.size, resource_path.c_str());
      return msg;
    }

    // 顶点去重：float32 → double 提升后精确比较（与 geometric_shapes 的 std::set 行为一致）
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

    // 打印网格范围用于验证（与 STL 源文件对比）
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
    } else {
      RCLCPP_WARN(get_logger(), "loadMesh: 空网格 — %s", resource_path.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "loadMesh: %s — %s", resource_path.c_str(), e.what());
  }
  return msg;
}

// ═══════════════════════════════════════════════════════════════════
// 初始化 & 场景操作
// ═══════════════════════════════════════════════════════════════════

/**
 * addAllToolsToWorld —— 把所有工具的碰撞模型添加到 PlanningScene 的世界中
 *
 * 调用时机：节点启动时，构造函数中调用一次。
 *
 * 这个函数把所有已加载工具的碰撞网格发布到 PlanningScene 的 "world" 中，
 * 位置在各自的 dock_pose（停靠位置）。这样 MoveIt 知道"世界里有这些物体"，
 * 在做碰撞检测时会考虑它们。
 *
 * 每个工具的碰撞对象：
 *   - 坐标系参考：base_link（机械臂基座）
 *   - 位置：工具在 dock 上的位姿
 *   - 形状：工具的三角形网格
 *
 * 为什么每个工具单独发一条消息？
 *   因为 PlanningScene 消息中 collision_objects 的操作(ADD/REMOVE)
 *   通常按对象处理。每个对象单独发布可以独立修改。
 *
 * 关键概念 —— PlanningScene 的 diff 模式：
 *   scene.is_diff = true 表示这是一个"增量更新"而非"全量替换"。
 *   全量替换会把世界清空再重建，增量更新只新增/修改/删除指定对象，
 *   不会影响世界中的其他物体（如桌子、工件等由别的节点管理的物体）。
 */
void SceneAttachWorker::addAllToolsToWorld()
{
  // 遍历所有已加载的工具（C++17 结构绑定语法）
  //   const auto& [tid, geom] :
  //     相当于 auto it = ...; std::string tid = it->first; const auto& geom = it->second;
  for (const auto& [tid, geom] : tool_geometries_)
  {
    // ─── 创建 PlanningScene 消息 ─────────────────────────────────
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;  // 增量更新模式！不会删除已有的物体

    // ─── 创建 CollisionObject（碰撞对象） ─────────────────────────
    moveit_msgs::msg::CollisionObject co;
    co.id = "tool_" + tid;          // 唯一标识符，加 "tool_" 前缀避免与别的物体重名
    co.header.frame_id = "base_link"; // 参考坐标系：机械臂基座
    co.operation = moveit_msgs::msg::CollisionObject::ADD;  // 操作类型：添加（非删除/移动）

    // ─── 设置物体的位置（Pose） ─────────────────────────────────
    // co.pose 是物体在世界中的整体位姿（位置+姿态）
    // dock_pose：工具在 dock 上的停靠位置，从 YAML 配置文件中读取
    co.pose = geom.dock_pose;

    // ─── 设置物体的形状（网格） ─────────────────────────────────
    // CollisionObject 支持多个网格（meshes），每个网格可以有自己相对物体的位姿
    // 这里每个工具只有一个网格，所以 push_back 一次
    co.meshes.push_back(geom.mesh_collision);  // 把三角形网格加入列表

    // mesh_poses[i] 是 meshes[i] 相对 co.pose 的偏移位姿
    // 这里创建一个"零偏移、无旋转"的 Pose（位置全 0，方向为单位四元数）
    co.mesh_poses.push_back(geometry_msgs::msg::Pose{});  // 默认构造，位置和方向都是 0
    co.mesh_poses.back().orientation.w = 1.0;  // w=1 表示四元数 [0,0,0,1] = 无旋转

    // 将碰撞对象加入场景的世界碰撞对象列表
    scene.world.collision_objects.push_back(co);

    // ─── 发布消息 ──────────────────────────────────────────────
    // 把这条 PlanningScene 消息发到 /planning_scene 话题，
    // MoveIt 的 PlanningSceneMonitor 会收到并更新场景
    planning_scene_pub_->publish(scene);
  }
  // 全部发布完成，打印信息日志
  RCLCPP_INFO(get_logger(), "全部 %zu 工具已添加到 world dock", tool_geometries_.size());
}

// ═══════════════════════════════════════════════════════════════════
// /tool_changer_status 订阅回调
// ═══════════════════════════════════════════════════════════════════

/**
 * onToolStatus —— 当工具快换装置状态发生变化时被调用
 *
 * 参数 msg：来自 /tool_changer_status 话题的消息，包含：
 *   - msg.tool_id：当前连接的工具 ID
 *   - msg.is_connected：true=工具已装上，false=工具已卸下
 *
 * 逻辑流程：
 *   1. 根据 is_connected 决定 new_tool：已连接则取 tool_id，断开则为空字符串 ""
 *   2. 如果 new_tool 和当前已附着的工具相同，什么都不做（避免重复操作）
 *   3. 否则：旧工具放回 world dock → 新工具从 world 移除 → 更新 URDF
 *   4. URDF 同时提供平滑视觉渲染 (Assimp) + 碰撞几何 (MoveIt 自动拾取)
 *   5. 不再使用 AttachedCollisionObject，避免 PlanningScene 平面着色叠加
 *
 * 为什么先脱后装？
 *   确保任意时刻只有一个工具的碰撞模型附着在机械臂上，
 *   否则 MoveIt 会认为机械臂同时拿着两个工具，碰撞检测会出问题。
 */
void SceneAttachWorker::onToolStatus(const ivg_interfaces::msg::ToolChangerStatus& msg)
{
  const std::string new_tool = msg.is_connected ? msg.tool_id : "";

  if (new_tool == current_attached_tool_)
    return;

  RCLCPP_INFO(get_logger(), "状态变更: %s → %s (connected=%s)",
              current_attached_tool_.c_str(), new_tool.c_str(),
              msg.is_connected ? "true" : "false");

  // ─── 旧工具放回 world dock ───────────────────────────────────
  // 只改 world CollisionObject 显示，不再操作 AttachedCollisionObject
  // 碰撞检测由 URDF <collision> 几何负责
  if (!current_attached_tool_.empty())
    addToolToWorldDock(current_attached_tool_);

  // ─── 新工具从 world dock 移除 ────────────────────────────────
  if (!new_tool.empty())
    removeToolFromWorld(new_tool);

  // ─── 更新 robot_description 参数 ────────────────────────────
  // URDF 更新同时提供平滑视觉渲染 (Assimp) + 碰撞几何 (MoveIt 自动拾取)
  updateRobotDescription(new_tool);

  current_attached_tool_ = new_tool;
}

// ═══════════════════════════════════════════════════════════════════
// World dock 辅助方法
// ═══════════════════════════════════════════════════════════════════

void SceneAttachWorker::addToolToWorldDock(const std::string& tool_id)
{
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) return;

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  moveit_msgs::msg::CollisionObject co;
  co.id = "tool_" + tool_id;
  co.header.frame_id = "base_link";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  co.pose = it->second.dock_pose;
  co.meshes.push_back(it->second.mesh_collision);
  co.mesh_poses.push_back(geometry_msgs::msg::Pose{});
  co.mesh_poses.back().orientation.w = 1.0;
  scene.world.collision_objects.push_back(co);

  planning_scene_pub_->publish(scene);
}

void SceneAttachWorker::removeToolFromWorld(const std::string& tool_id)
{
  if (tool_geometries_.find(tool_id) == tool_geometries_.end()) return;

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  moveit_msgs::msg::CollisionObject co;
  co.id = "tool_" + tool_id;
  co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene.world.collision_objects.push_back(co);

  planning_scene_pub_->publish(scene);
}

// ═══════════════════════════════════════════════════════════════════
// 服务回调（手动附着/脱离）
// ═══════════════════════════════════════════════════════════════════

/**
 * onSceneAttach —— /scene_attach 服务的回调
 *
 * 当外部节点调用 /scene_attach 服务时触发。
 * 参数 req->tool_id 指定要附着哪个工具。
 *
 * 与 onToolStatus 的区别：
 *   - onToolStatus 是由硬件状态变化自动触发的
 *   - onSceneAttach 是外部节点手动调用的（如用于调试、测试、Web 界面手动控制）
 *
 * 返回值通过 resp 传出：
 *   - resp->success：操作是否成功
 *   - resp->message：成功/失败的描述信息
 */
void SceneAttachWorker::onSceneAttach(
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp)
{
  bool found = tool_geometries_.find(req->tool_id) != tool_geometries_.end();
  if (found)
  {
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
  if (found && current_attached_tool_ == req->tool_id)
  {
    addToolToWorldDock(req->tool_id);
    current_attached_tool_.clear();
    updateRobotDescription("");
  }
  resp->success = found;
  resp->message = found ? ("脱离: " + req->tool_id) : ("未知工具: " + req->tool_id);
}

// ═══════════════════════════════════════════════════════════════════
// robot_description 参数更新
// ═══════════════════════════════════════════════════════════════════

void SceneAttachWorker::updateRobotDescription(const std::string& tool_id)
{
  auto it = urdf_cache_.find(tool_id);
  if (it == urdf_cache_.end()) {
    RCLCPP_WARN(get_logger(), "URDF 缓存未命中: %s", tool_id.c_str());
    return;
  }
  const auto& urdf = it->second;
  if (urdf.empty()) return;

  // 1. 直接发布 URDF 到 /robot_description 话题 → RViz2 立即重载
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = urdf;
  robot_description_pub_->publish(std::move(msg));

  // 2. 设置 robot_description 参数 → robot_state_publisher 重载 TF
  if (!param_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "robot_state_publisher set_parameters 不可达");
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
// 主函数 —— ROS2 程序的入口点
// ═══════════════════════════════════════════════════════════════════════

/**
 * main —— C++ 程序的入口函数，ROS2 节点的启动流程固定写法：
 *
 *   1. rclcpp::init(argc, argv)     —— 初始化 ROS2 客户端库
 *   2. rclcpp::spin(node)           —— 进入事件循环（阻塞等待消息和回调）
 *   3. rclcpp::shutdown()           —— 程序退出前清理资源
 *
 * 说明：
 *   - rclcpp::init 必须在任何 ROS2 对象创建之前调用
 *   - rclcpp::spin 让节点保持运行，处理订阅回调和服务请求
 *   - std::make_shared<T>(args...)：创建 shared_ptr 智能指针，自动管理对象生命周期
 *   - 当用户 Ctrl+C 或节点被杀死时，spin 返回，shutdown 清理
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tool_changer::SceneAttachWorker>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
