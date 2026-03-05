/*
 * MoveIt C++ 教程：打印获取的当前状态（参考 moveit2_tutorials doc/examples/moveit_cpp）
 * 使用 MoveItCpp + PlanningComponent 获取 start state / current state 并 printStateInfo 输出。
 *
 * 本节点必须带有 robot_description、robot_description_semantic 参数（由 launch 注入）。
 * automatically_declare_parameters_from_overrides(true) 只会声明“启动时传入的覆盖参数”，
 * 不会从文件或其它节点拉取；故不能单独 ros2 run，需用 launch 启动。
 *
 * 单独运行示例：ros2 launch aubo_moveit_config moveit_cpp_tutorial.launch.py
 * 或随 graspnet：ros2 launch graspnet_ros2 graspnet_demo.launch.py use_moveit_cpp_tutorial:=true
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sstream>
#include <iomanip>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/robot_state.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

/** 打印单次状态的详细信息：printStateInfo + 关节名/位置 + 末端位姿 */
static void printStateDetail(const moveit::core::RobotStatePtr& state,
                             const moveit::core::RobotModelConstPtr& robot_model,
                             const std::string& group_name,
                             const std::string& state_label)
{
  if (!state || !robot_model)
    return;
  std::stringstream ss;
  state->printStateInfo(ss);
  RCLCPP_INFO(LOGGER, "[%s] printStateInfo:\n%s", state_label.c_str(), ss.str().c_str());

  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    RCLCPP_WARN(LOGGER, "[%s] no JointModelGroup '%s'", state_label.c_str(), group_name.c_str());
    return;
  }

  std::vector<double> positions;
  state->copyJointGroupPositions(jmg, positions);
  std::vector<std::string> names = jmg->getActiveJointModelNames();
  ss.str("");
  ss << "  Joint positions (rad): ";
  for (size_t i = 0; i < names.size() && i < positions.size(); ++i)
    ss << names[i] << "=" << std::fixed << std::setprecision(4) << positions[i] << " ";
  RCLCPP_INFO(LOGGER, "[%s] %s", state_label.c_str(), ss.str().c_str());

  std::vector<std::string> link_names = jmg->getLinkModelNames();
  if (!link_names.empty())
  {
    std::string tip_link = link_names.back();
    state->update();
    const Eigen::Isometry3d& T = state->getGlobalLinkTransform(tip_link);
    Eigen::Vector3d p = T.translation();
    Eigen::Quaterniond q(T.rotation());
    RCLCPP_INFO(LOGGER,
                "[%s] End effector (%s) pose: xyz=[%.4f, %.4f, %.4f] xyzw=[%.4f, %.4f, %.4f, %.4f]",
                state_label.c_str(), tip_link.c_str(), p.x(), p.y(), p.z(),
                q.x(), q.y(), q.z(), q.w());
  }

  const std::vector<std::string>& var_names = state->getVariableNames();
  const double* var_pos = state->getVariablePositions();
  if (var_pos && !var_names.empty())
  {
    ss.str("");
    ss << "  All variable positions: ";
    for (size_t i = 0; i < var_names.size(); ++i)
      ss << var_names[i] << "=" << std::fixed << std::setprecision(4) << var_pos[i] << " ";
    RCLCPP_INFO(LOGGER, "[%s] %s", state_label.c_str(), ss.str().c_str());
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "manipulator";

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Initialize node (Aubo moveit_cpp_tutorial)");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

  auto planning_components =
      std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  // 参考官方：getStartState() 为规划考虑的起始状态（默认即当前状态）
  auto robot_start_state = planning_components->getStartState();
  printStateDetail(robot_start_state, robot_model_ptr, PLANNING_GROUP, "StartState");

  // 从 /joint_states 获取的当前状态（getCurrentState）
  moveit::core::RobotStatePtr current_state_ptr;
  if (moveit_cpp_ptr->getCurrentState(current_state_ptr, 2.0) && current_state_ptr)
    printStateDetail(current_state_ptr, robot_model_ptr, PLANNING_GROUP, "CurrentState(joint_states)");
  else
    RCLCPP_WARN(LOGGER, "getCurrentState failed or timeout (joint_states may not be published yet)");

  // PlanningScene 中的当前状态（与 joint_states 同步后的场景状态）
  planning_scene_monitor::PlanningSceneMonitorPtr psm = moveit_cpp_ptr->getPlanningSceneMonitorNonConst();
  if (psm)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm);
    if (scene)
    {
      const moveit::core::RobotState& scene_current = scene->getCurrentState();
      moveit::core::RobotStatePtr scene_state = std::make_shared<moveit::core::RobotState>(scene_current);
      printStateDetail(scene_state, robot_model_ptr, PLANNING_GROUP, "PlanningScene::getCurrentState()");
    }
    else
      RCLCPP_WARN(LOGGER, "LockedPlanningSceneRO lock failed");
  }
  else
    RCLCPP_WARN(LOGGER, "PlanningSceneMonitor is null");

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
