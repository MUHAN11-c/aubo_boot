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
#include <geometric_shapes/mesh_operations.h>               // 3D 网格加载相关工具
#include <geometric_shapes/shape_operations.h>               // 形状与消息互转
#include <geometric_shapes/shapes.h>                         // 形状基类
#include <yaml-cpp/yaml.h>                                   // 解析 YAML 配置文件

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

  // ─── 2. 创建工具状态订阅者 ────────────────────────────────────────
  // 订阅者(Subscription)：用来接收某个"话题"的消息。
  // 这里订阅 "/tool_changer_status" 话题，当物理工具快换装置状态变化时
  // （比如装上或卸下一个工具），会收到通知。
  //
  // 第三个参数是一个 Lambda 表达式（匿名函数）：
  //   [this](const ... & msg) { onToolStatus(msg); }
  //   - [this]：捕获当前对象的 this 指针，以便在回调中访问成员函数
  //   - 收到消息后，调用 onToolStatus() 来处理状态变化
  tool_status_sub_ = create_subscription<tool_changer_interface::msg::ToolChangerStatus>(
      "/tool_changer_status", 10,  // 队列深度 10
      [this](const tool_changer_interface::msg::ToolChangerStatus& msg) { onToolStatus(msg); });

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
  scene_attach_srv_ = create_service<tool_changer_interface::srv::ChangeTool>(
      "/scene_attach",
      std::bind(&SceneAttachWorker::onSceneAttach, this, std::placeholders::_1, std::placeholders::_2));

  // /scene_detach 服务：手动让某个工具的碰撞模型脱离机械臂，回到世界 dock
  scene_detach_srv_ = create_service<tool_changer_interface::srv::ChangeTool>(
      "/scene_detach",
      std::bind(&SceneAttachWorker::onSceneDetach, this, std::placeholders::_1, std::placeholders::_2));

  // ─── 4. 加载配置 & 初始化场景 ─────────────────────────────────────
  loadToolConfig();       // 从 tools.yaml 读取所有工具的网格模型和位姿信息
  addAllToolsToWorld();   // 把所有工具发布到 PlanningScene 的 world 中（初始状态在 dock 位置）

  RCLCPP_INFO(get_logger(), "就绪 | 监听 /tool_changer_status | /scene_attach /scene_detach | %zu 工具",
              tool_geometries_.size());  // %zu 是 size_t 的格式占位符，打印工具数量
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
 * loadMesh —— 从资源路径加载 3D 网格，转为 ROS 消息
 *
 * 参数 resource_path：网格文件的资源路径，格式如 "package://tool_changer/meshes/gripper.stl"
 *                     或者 "file:///absolute/path/to/mesh.stl"
 *
 * 返回值 shape_msgs::msg::Mesh：ROS 标准的网格消息，包含：
 *   - triangles：三角形面片列表（每个三角形由 3 个顶点索引组成）
 *   - vertices：顶点列表（每个顶点的 x, y, z 坐标）
 *
 * 为什么用三角形网格？
 *   MoveIt 的碰撞检测引擎（FCL/Bullet）使用三角形网格来精确表示物体形状，
 *   这样在做碰撞检测时可以准确判断机械臂会不会碰到这个工具。
 *
 * 流程：
 *   1. shapes::createMeshFromResource() 从文件读取网格 → 通用 Shape 指针
 *   2. shapes::constructMsgFromShape()    把 Shape 转为 ROS 消息变体
 *   3. 从消息变体中提取 shape_msgs::msg::Mesh 类型的数据
 */
shape_msgs::msg::Mesh SceneAttachWorker::loadMesh(const std::string& resource_path)
{
  shape_msgs::msg::Mesh msg;  // 默认构造一个空网格（0 个三角形，0 个顶点）
  try {
    // 步骤 1：从资源路径创建网格 shape
    //   createMeshFromResource 支持 STL/DAE 等格式，自动检测文件类型
    //   返回的是 shapes::Shape 的共享指针（智能指针，自动管理内存）
    auto shape = shapes::createMeshFromResource(resource_path);
    if (!shape) return msg;  // 加载失败，返回空网格

    // 步骤 2：把 Shape 对象转换为 ROS 消息格式
    //   ShapeMsg 是一个 boost::variant，可以容纳各种类型的网格消息
    //   constructMsgFromShape 根据 Shape 的具体类型（如 Mesh）填充对应的消息
    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(shape, shape_msg)) return msg;

    // 步骤 3：从变体中取出 Mesh 类型的消息
    //   typeid() 检查变体里装的是什么类型 → 如果是 Mesh 类型
    //   boost::get<shape_msgs::msg::Mesh>() 把值取出来赋值给 msg
    //
    //   注：boost::variant 类似 C++17 的 std::variant，
    //       可以存储多种不同类型中的一种
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

/**
 * attachTool —— 把工具的碰撞模型从"世界"移动到"机械臂上"
 *
 * 参数 tool_id：要附着的工具 ID（如 "gripper_a"）
 *
 * 当机械臂装上工具时，需要做两件事：
 *   1. 把工具的碰撞模型从"世界"中删除（因为工具不在世界中了）
 *   2. 把工具的碰撞模型"附着"到机械臂的末端关节（kuaihuan_Link）上
 *      —— 这样工具会跟着关节一起运动，MoveIt 才知道工具和手臂的关系
 *
 * 返回值：成功返回 true，工具 ID 不存在返回 false
 *
 * "世界对象" vs "附着对象"：
 *   - CollisionObject（世界对象）：位置固定，不随机械臂运动
 *   - AttachedCollisionObject（附着对象）：贴在某个关节上，随关节一起运动
 *     它知道自己是"机械臂的一部分"而不是"环境的障碍物"
 */
bool SceneAttachWorker::attachTool(const std::string& tool_id)
{
  // ─── 查找工具 ─────────────────────────────────────────────────
  // tool_geometries_ 是一个 std::map，find() 返回迭代器
  // find 不到时返回 tool_geometries_.end()（指向末尾的迭代器，表示"不存在"）
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) {
    RCLCPP_ERROR(get_logger(), "未知: %s", tool_id.c_str());
    return false;
  }

  // ─── 创建 PlanningScene 消息 ─────────────────────────────────
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;           // 增量更新世界
  scene.robot_state.is_diff = true; // 增量更新机器人状态（只修改附着物体，不动关节位置等）

  // ─── 步骤 1：从世界删除工具的碰撞对象 ─────────────────────────
  // 因为工具已经被装到机械臂上了，世界（dock 位置）不应该还有它的碰撞模型
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = "tool_" + tool_id;  // 与 addAllToolsToWorld 中使用相同的 ID
    co.operation = moveit_msgs::msg::CollisionObject::REMOVE;  // REMOVE 操作：删除该物体
    scene.world.collision_objects.push_back(co);  // 这个 REMOVE 指令发给 MoveIt
  }

  // ─── 步骤 2：把工具附着到机械臂末端关节 ───────────────────────
  {
    moveit_msgs::msg::AttachedCollisionObject aco;

    // link_name：工具附着到哪个关节上
    // kuaihuan_Link 是工具快换装置的接口关节（机械臂末端的法兰/快换盘）
    aco.link_name = "kuaihuan_Link";

    // touch_links："触碰关节"列表，MoveIt 会忽略这些关节与工具之间的碰撞检测
    // 如果不设这个，工具外形和机械臂本身的关节重叠时会被误判为"碰撞"
    aco.touch_links = it->second.touch_links;

    // object：附着物体的具体信息
    aco.object.id = "tool_" + tool_id;          // 与上面 REMOVE 的 ID 对应
    aco.object.header.frame_id = "kuaihuan_Link"; // 参考坐标系改为末端关节
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD; // 添加操作

    // aco.object.pose：附着物体相对 link_name 的位姿
    // attach_offset 从 YAML 配置读取，通常是工具的重心或安装点偏移
    aco.object.pose = it->second.attach_offset;

    // 复制网格数据
    aco.object.meshes.push_back(it->second.mesh_collision);
    aco.object.mesh_poses.push_back(geometry_msgs::msg::Pose{});
    aco.object.mesh_poses.back().orientation.w = 1.0;  // 网格相对物体的位姿：无额外旋转

    // 把附着对象加入机器人状态的附着列表
    scene.robot_state.attached_collision_objects.push_back(aco);
  }

  // ─── 发布 ─────────────────────────────────────────────────────
  planning_scene_pub_->publish(scene);
  RCLCPP_INFO(get_logger(), "附着: %s → kuaihuan_Link", tool_id.c_str());
  return true;
}

/**
 * detachTool —— 把工具的碰撞模型从"机械臂上"移回"世界 dock"
 *
 * 参数 tool_id：要脱离的工具 ID
 *
 * 当机械臂卸下工具时，需要做与 attach 相反的操作：
 *   1. 从机械臂的附着列表中移除工具的碰撞模型
 *   2. 把工具的碰撞模型重新添加到世界中的 dock 位置
 *
 * 返回值：成功返回 true，工具 ID 不存在返回 false
 */
bool SceneAttachWorker::detachTool(const std::string& tool_id)
{
  // ─── 查找工具 ─────────────────────────────────────────────────
  auto it = tool_geometries_.find(tool_id);
  if (it == tool_geometries_.end()) {
    RCLCPP_ERROR(get_logger(), "未知: %s", tool_id.c_str());
    return false;
  }

  // ─── 创建 PlanningScene 消息 ─────────────────────────────────
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;

  // ─── 步骤 1：从机械臂上移除附着对象 ─────────────────────────────
  // 只需要指定对象 ID 和附着关节，操作设 REMOVE 即可
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = "kuaihuan_Link";          // 与 attach 时相同的关节
    aco.object.id = "tool_" + tool_id;        // 与 attach 时相同的 ID
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;  // 移除操作
    scene.robot_state.attached_collision_objects.push_back(aco);
  }

  // ─── 步骤 2：把工具重新添加到世界中的 dock 位置 ────────────────
  // 与 addAllToolsToWorld 中的逻辑完全一致
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = "tool_" + tool_id;
    co.header.frame_id = "base_link";         // 世界参考系
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    co.pose = it->second.dock_pose;           // 放在 dock 停靠位置
    co.meshes.push_back(it->second.mesh_collision);
    co.mesh_poses.push_back(geometry_msgs::msg::Pose{});
    co.mesh_poses.back().orientation.w = 1.0;
    scene.world.collision_objects.push_back(co);
  }

  // ─── 发布 ─────────────────────────────────────────────────────
  planning_scene_pub_->publish(scene);
  RCLCPP_INFO(get_logger(), "脱离: %s → dock", tool_id.c_str());
  return true;
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
 *   3. 否则：先脱离旧工具 (detachTool)，再附着新工具 (attachTool)
 *   4. 更新 current_attached_tool_ 记录当前状态
 *
 * 为什么先脱后装？
 *   确保任意时刻只有一个工具的碰撞模型附着在机械臂上，
 *   否则 MoveIt 会认为机械臂同时拿着两个工具，碰撞检测会出问题。
 */
void SceneAttachWorker::onToolStatus(const tool_changer_interface::msg::ToolChangerStatus& msg)
{
  // 根据连接状态确定"新工具 ID"
  //   msg.is_connected == true  → new_tool = msg.tool_id（如 "gripper_a"）
  //   msg.is_connected == false → new_tool = ""（空字符串表示"没有工具"）
  const std::string new_tool = msg.is_connected ? msg.tool_id : "";

  // 如果和当前状态一样，说明没有变化，直接返回避免重复操作
  if (new_tool == current_attached_tool_)
    return;

  RCLCPP_INFO(get_logger(), "状态变更: %s → %s (connected=%s)",
              current_attached_tool_.c_str(), new_tool.c_str(),
              msg.is_connected ? "true" : "false");

  // ─── 先脱离旧工具 ─────────────────────────────────────────────
  // 如果当前有工具附着（current_attached_tool_ 非空），先把它从机械臂上卸下来
  if (!current_attached_tool_.empty())
    detachTool(current_attached_tool_);

  // ─── 再附着新工具 ─────────────────────────────────────────────
  // 如果新工具有效（非空），把它的模型附着到机械臂上
  if (!new_tool.empty())
    attachTool(new_tool);

  // ─── 更新当前状态 ─────────────────────────────────────────────
  current_attached_tool_ = new_tool;
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
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> req,
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> resp)
{
  resp->success = attachTool(req->tool_id);  // 调用附着函数，结果作为成功/失败标志
  if (resp->success)
    current_attached_tool_ = req->tool_id;   // 更新当前已附着的工具记录
  resp->message = resp->success ? ("附着: " + req->tool_id) : ("失败: " + req->tool_id);
}

/**
 * onSceneDetach —— /scene_detach 服务的回调
 *
 * 当外部节点调用 /scene_detach 服务时触发。
 * 参数 req->tool_id 指定要脱离哪个工具。
 *
 * 注意：只有在 current_attached_tool_ 与请求的 tool_id 一致时才清空状态，
 * 防止误清空（比如请求脱离工具 A，但当前附着的是工具 B）。
 */
void SceneAttachWorker::onSceneDetach(
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> req,
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> resp)
{
  resp->success = detachTool(req->tool_id);   // 调用脱离函数
  if (resp->success && current_attached_tool_ == req->tool_id)
    current_attached_tool_.clear();           // 清空当前附着记录（因为工具已卸下）
  resp->message = resp->success ? ("脱离: " + req->tool_id) : ("失败: " + req->tool_id);
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
