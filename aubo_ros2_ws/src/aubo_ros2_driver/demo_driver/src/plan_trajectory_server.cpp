/*
 * /plan_trajectory — 规划从当前位姿到目标的关节轨迹。构造函数内完成初始化。
 * use_joints=true → IK 服务 → setJointValueTarget → plan
 * use_joints=false → setPoseTarget → plan
 */
#include "demo_driver/plan_trajectory_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <ivg_interfaces/srv/get_ik.hpp>
#include <chrono>

namespace demo_driver
{

PlanTrajectoryServer::PlanTrajectoryServer(const rclcpp::NodeOptions& options)
    : Node("plan_trajectory_server_node", options)
{
    declare_parameter("planning_group_name", "manipulator");
    declare_parameter("base_frame", "base_link");
    planning_group_name_ = get_parameter("planning_group_name").as_string();
    base_frame_          = get_parameter("base_frame").as_string();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_name_);
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setPlanningTime(10.0);
    end_effector_link_ = move_group_->getEndEffectorLink();

    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    ik_client_ = create_client<ivg_interfaces::srv::GetIK>(
        "/aubo_driver/get_ik", rmw_qos_profile_services_default, client_cb_group_);

    service_ = create_service<ivg_interfaces::srv::PlanTrajectory>(
        "/plan_trajectory",
        [this](auto req, auto res) { planTrajectoryCallback(req, res); });

    RCLCPP_INFO(get_logger(), "PlanTrajectoryServer ready, end_effector=%s", end_effector_link_.c_str());
}

void PlanTrajectoryServer::planTrajectoryCallback(
    const std::shared_ptr<ivg_interfaces::srv::PlanTrajectory::Request> req,
    std::shared_ptr<ivg_interfaces::srv::PlanTrajectory::Response> res)
{
    if (!move_group_) {
        res->success = false; res->planning_time = 0; res->message = "not initialized"; return;
    }

    float pt = 0;
    std::string msg;
    res->success = planTrajectory(req->target_pose, req->use_joints, res->trajectory, pt, msg);
    res->planning_time = pt;
    res->message = msg;
}

bool PlanTrajectoryServer::planTrajectory(const geometry_msgs::msg::Pose& target_pose,
                                          bool use_joints,
                                          trajectory_msgs::msg::JointTrajectory& trajectory,
                                          float& planning_time, std::string& message)
{
    if (!move_group_) { message = "not initialized"; return false; }

    try {
        // 读取速度因子 (由 launch 注入的本地参数)
        if (has_parameter("moveit_velocity_scaling_factor"))
            move_group_->setMaxVelocityScalingFactor(
                get_parameter("moveit_velocity_scaling_factor").as_double());

        auto t0 = std::chrono::high_resolution_clock::now();

        if (use_joints) {
            // 获取当前关节状态 → IK 服务求目标关节角
            auto current = move_group_->getCurrentState();
            if (!current) {
                message = "failed to get current robot state"; return false;
            }

            const auto* jmg = current->getJointModelGroup(planning_group_name_);
            if (!jmg) {
                message = "failed to get joint model group"; return false;
            }

            std::vector<double> current_joints;
            current->copyJointGroupPositions(jmg, current_joints);

            if (ik_client_->wait_for_service(std::chrono::seconds(2))) {
                auto ik_req = std::make_shared<ivg_interfaces::srv::GetIK::Request>();
                for (size_t i = 0; i < 6; ++i)
                    ik_req->ref_joint[i] = static_cast<float>(i < current_joints.size() ? current_joints[i] : 0.0);
                ik_req->pos[0] = target_pose.position.x;
                ik_req->pos[1] = target_pose.position.y;
                ik_req->pos[2] = target_pose.position.z;
                ik_req->ori[0] = target_pose.orientation.w;
                ik_req->ori[1] = target_pose.orientation.x;
                ik_req->ori[2] = target_pose.orientation.y;
                ik_req->ori[3] = target_pose.orientation.z;

                auto future = ik_client_->async_send_request(ik_req);
                if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
                    auto ik_res = future.get();
                    if (ik_res->joint.size() >= 6) {
                        move_group_->setJointValueTarget(
                            std::vector<double>(ik_res->joint.begin(), ik_res->joint.begin() + 6));
                    } else {
                        message = "IK returned insufficient joints"; return false;
                    }
                } else {
                    message = "IK service timeout"; return false;
                }
            } else {
                // IK 服务不可用, 回退到 pose target
                move_group_->setPoseTarget(target_pose);
            }
        } else {
            move_group_->setPoseTarget(target_pose);
        }

        // 执行规划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_->plan(plan);

        planning_time = std::chrono::duration<float>(
            std::chrono::high_resolution_clock::now() - t0).count();

        if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            message = "planning failed, error_code=" + std::to_string(result.val);
            return false;
        }

        if (plan.trajectory_.joint_trajectory.joint_names.empty()) {
            message = "planned trajectory is empty"; return false;
        }

        trajectory = plan.trajectory_.joint_trajectory;
        message = "ok, " + std::to_string(trajectory.points.size()) + " waypoints";
        return true;
    } catch (const std::exception& e) {
        message = e.what();
        return false;
    }
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts; opts.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<demo_driver::PlanTrajectoryServer>(opts);

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
