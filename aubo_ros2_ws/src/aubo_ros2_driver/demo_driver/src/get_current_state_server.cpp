/*
 * /get_current_state — 获取机械臂关节角+笛卡尔位姿+速度。
 * 数据源: 订阅 joint_states 缓存 + FK 服务/MoveIt 计算笛卡尔位姿。
 */
#include "demo_driver/get_current_state_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/robot_state/robot_state.h>
#include <ivg_interfaces/srv/get_fk.hpp>
#include <Eigen/Geometry>

namespace demo_driver
{

GetCurrentStateServer::GetCurrentStateServer(const rclcpp::NodeOptions& options)
    : Node("get_current_state_server_node", options)
{
    declare_parameter("planning_group_name", "manipulator");
    declare_parameter("base_frame", "base_link");
    planning_group_name_ = get_parameter("planning_group_name").as_string();
    base_frame_          = get_parameter("base_frame").as_string();

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
          current_joint_states_ = *msg; joint_states_received_ = true;
        });

    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    fk_client_ = create_client<ivg_interfaces::srv::GetFK>(
        "/aubo_driver/get_fk", rmw_qos_profile_services_default, client_cb_group_);

    service_ = create_service<ivg_interfaces::srv::GetCurrentState>(
        "/get_current_state",
        [this](const std::shared_ptr<ivg_interfaces::srv::GetCurrentState::Request> /*req*/,
               std::shared_ptr<ivg_interfaces::srv::GetCurrentState::Response> res) {
          getCurrentStateCallback(res);
        });
}

void GetCurrentStateServer::init()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_name_);
    move_group_->setPoseReferenceFrame(base_frame_);
    end_effector_link_ = move_group_->getEndEffectorLink();

    RCLCPP_INFO(get_logger(), "GetCurrentStateServer ready");
}

void GetCurrentStateServer::getCurrentStateCallback(
    std::shared_ptr<ivg_interfaces::srv::GetCurrentState::Response> res)
{
    std::string msg;
    res->success = getCurrentState(res->joint_position_rad, res->cartesian_position,
                                   res->velocity, msg);
    res->message = msg;
}

bool GetCurrentStateServer::getCurrentState(std::vector<double>& joint_rad,
                                            geometry_msgs::msg::Pose& cartesian,
                                            std::vector<double>& vel,
                                            std::string& msg)
{
    // 1. 从缓存的 joint_states 获取关节角 + 速度
    if (joint_states_received_ && current_joint_states_.position.size() >= 6) {
        joint_rad.assign(current_joint_states_.position.begin(),
                         current_joint_states_.position.begin() + 6);
        vel.resize(6, 0.0);
        for (size_t i = 0; i < 6 && i < current_joint_states_.velocity.size(); ++i)
            vel[i] = current_joint_states_.velocity[i];
    }
    // 2. 回退到 MoveIt
    else if (move_group_) {
        try {
            auto state = move_group_->getCurrentState();
            if (!state) { msg = "no current robot state"; return false; }
            const auto* jmg = state->getJointModelGroup(planning_group_name_);
            if (!jmg) { msg = "no joint model group"; return false; }
            std::vector<double> jv;
            state->copyJointGroupPositions(jmg, jv);
            if (jv.size() < 6) { msg = "insufficient joints"; return false; }
            joint_rad.assign(jv.begin(), jv.begin() + 6);
            vel.assign(6, 0.0);
        } catch (const std::exception& e) {
            msg = e.what(); return false;
        }
    } else {
        msg = "no data source"; return false;
    }

    // 3. 计算笛卡尔位姿: FK 服务优先, 回退 MoveIt
    if (computeFK(joint_rad, cartesian)) {
        msg = "ok"; return true;
    }
    msg = "FK computation failed"; return false;
}

bool GetCurrentStateServer::computeFK(const std::vector<double>& joints,
                                      geometry_msgs::msg::Pose& pose)
{
    // 方法 1: FK 服务
    if (fk_client_->service_is_ready()) {
        auto req = std::make_shared<ivg_interfaces::srv::GetFK::Request>();
        for (size_t i = 0; i < 6; ++i) req->joint[i] = static_cast<float>(joints[i]);
        auto future = fk_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto res = future.get();
            if (res->pos.size() >= 3 && res->ori.size() >= 4) {
                pose.position.x = res->pos[0]; pose.position.y = res->pos[1]; pose.position.z = res->pos[2];
                pose.orientation.w = res->ori[0]; pose.orientation.x = res->ori[1];
                pose.orientation.y = res->ori[2]; pose.orientation.z = res->ori[3];
                return true;
            }
        }
    }

    // 方法 2: MoveIt 正运动学
    if (move_group_ && !end_effector_link_.empty()) {
        try {
            auto state = move_group_->getCurrentState();
            if (!state) return false;
            const auto* jmg = state->getJointModelGroup(planning_group_name_);
            if (!jmg) return false;
            state->setJointGroupPositions(jmg, joints);
            state->update();
            const auto& t = state->getGlobalLinkTransform(end_effector_link_);
            pose.position.x = t.translation().x();
            pose.position.y = t.translation().y();
            pose.position.z = t.translation().z();
            Eigen::Quaterniond q(t.rotation());
            pose.orientation.w = q.w(); pose.orientation.x = q.x();
            pose.orientation.y = q.y(); pose.orientation.z = q.z();
            return true;
        } catch (const std::exception&) {}
    }
    return false;
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts; opts.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<demo_driver::GetCurrentStateServer>(opts);
    node->init();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
