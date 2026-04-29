/*
 * 综合示例：演示完整的工作流程
 * 包括规划轨迹、执行轨迹、获取状态、设置IO等操作
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/plan_trajectory.hpp>
#include <demo_interface/srv/execute_trajectory.hpp>
#include <demo_interface/srv/get_current_state.hpp>
#include <demo_interface/srv/set_robot_io.hpp>
#include <demo_interface/srv/read_robot_io.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ComprehensiveExample : public rclcpp::Node
{
public:
    ComprehensiveExample() : Node("comprehensive_example")
    {
        plan_client_ = this->create_client<demo_interface::srv::PlanTrajectory>("/plan_trajectory");
        execute_client_ = this->create_client<demo_interface::srv::ExecuteTrajectory>("/execute_trajectory");
        get_state_client_ = this->create_client<demo_interface::srv::GetCurrentState>("/get_current_state");
        set_io_client_ = this->create_client<demo_interface::srv::SetRobotIO>("/set_robot_io");
        read_io_client_ = this->create_client<demo_interface::srv::ReadRobotIO>("/read_robot_io");
    }

    bool wait_for_services()
    {
        std::vector<std::pair<std::string, rclcpp::ClientBase::SharedPtr>> services = {
            {"/plan_trajectory", plan_client_},
            {"/execute_trajectory", execute_client_},
            {"/get_current_state", get_state_client_},
            {"/set_robot_io", set_io_client_},
            {"/read_robot_io", read_io_client_},
        };

        for (auto& [name, client] : services) {
            RCLCPP_INFO(this->get_logger(), "等待服务 %s 可用...", name.c_str());
            if (!client->wait_for_service(5s)) {
                RCLCPP_ERROR(this->get_logger(), "服务 %s 不可用", name.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务 %s 已就绪", name.c_str());
        }
        return true;
    }

    std::shared_ptr<demo_interface::srv::GetCurrentState::Response> get_current_state()
    {
        auto request = std::make_shared<demo_interface::srv::GetCurrentState::Request>();
        auto future = get_state_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        }
        return nullptr;
    }

    std::shared_ptr<demo_interface::srv::PlanTrajectory::Response> plan_trajectory(
        const geometry_msgs::msg::Pose& target_pose, bool use_joints = false)
    {
        auto request = std::make_shared<demo_interface::srv::PlanTrajectory::Request>();
        request->target_pose = target_pose;
        request->use_joints = use_joints;
        
        auto future = plan_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        }
        return nullptr;
    }

    std::shared_ptr<demo_interface::srv::ExecuteTrajectory::Response> execute_trajectory(
        const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        auto request = std::make_shared<demo_interface::srv::ExecuteTrajectory::Request>();
        request->trajectory = trajectory;
        
        auto future = execute_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        }
        return nullptr;
    }

    std::shared_ptr<demo_interface::srv::SetRobotIO::Response> set_robot_io(
        const std::string& io_type, int32_t io_index, double value)
    {
        auto request = std::make_shared<demo_interface::srv::SetRobotIO::Request>();
        request->io_type = io_type;
        request->io_index = io_index;
        request->value = value;
        
        auto future = set_io_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        }
        return nullptr;
    }

    std::shared_ptr<demo_interface::srv::ReadRobotIO::Response> read_robot_io(
        const std::string& io_type, int32_t io_index)
    {
        auto request = std::make_shared<demo_interface::srv::ReadRobotIO::Request>();
        request->io_type = io_type;
        request->io_index = io_index;
        
        auto future = read_io_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        }
        return nullptr;
    }

private:
    rclcpp::Client<demo_interface::srv::PlanTrajectory>::SharedPtr plan_client_;
    rclcpp::Client<demo_interface::srv::ExecuteTrajectory>::SharedPtr execute_client_;
    rclcpp::Client<demo_interface::srv::GetCurrentState>::SharedPtr get_state_client_;
    rclcpp::Client<demo_interface::srv::SetRobotIO>::SharedPtr set_io_client_;
    rclcpp::Client<demo_interface::srv::ReadRobotIO>::SharedPtr read_io_client_;
};

geometry_msgs::msg::Pose create_pose(double x, double y, double z,
                                     double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto example = std::make_shared<ComprehensiveExample>();

    if (!example->wait_for_services()) {
        RCLCPP_ERROR(example->get_logger(), "无法连接到所有服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(example->get_logger(), "\n============================================================");
    RCLCPP_INFO(example->get_logger(), "综合示例：完整工作流程演示");
    RCLCPP_INFO(example->get_logger(), "============================================================\n");

    // 步骤1: 获取当前状态
    RCLCPP_INFO(example->get_logger(), "步骤1: 获取机器人当前状态");
    RCLCPP_INFO(example->get_logger(), "------------------------------------------------------------");
    auto state = example->get_current_state();
    if (state && state->success) {
        RCLCPP_INFO(example->get_logger(), "✓ 成功获取状态: %s", state->message.c_str());
        const auto& pos = state->cartesian_position.position;
        RCLCPP_INFO(example->get_logger(), "  当前位置: x=%.3f, y=%.3f, z=%.3f", pos.x, pos.y, pos.z);
    } else {
        RCLCPP_ERROR(example->get_logger(), "✗ 获取状态失败");
    }
    std::this_thread::sleep_for(1s);

    // 步骤2: 读取IO状态
    RCLCPP_INFO(example->get_logger(), "\n步骤2: 读取数字输入 #0");
    RCLCPP_INFO(example->get_logger(), "------------------------------------------------------------");
    auto io_response = example->read_robot_io("digital_input", 0);
    if (io_response && io_response->success) {
        RCLCPP_INFO(example->get_logger(), "✓ 数字输入 #0 = %s",
                   io_response->value > 0.5 ? "HIGH" : "LOW");
    } else {
        RCLCPP_ERROR(example->get_logger(), "✗ 读取IO失败");
    }
    std::this_thread::sleep_for(1s);

    // 步骤3: 设置IO
    RCLCPP_INFO(example->get_logger(), "\n步骤3: 设置数字输出 #0 为 HIGH");
    RCLCPP_INFO(example->get_logger(), "------------------------------------------------------------");
    auto set_io_response = example->set_robot_io("digital_output", 0, 1.0);
    if (set_io_response && set_io_response->success) {
        RCLCPP_INFO(example->get_logger(), "✓ %s", set_io_response->message.c_str());
    } else {
        RCLCPP_ERROR(example->get_logger(), "✗ 设置IO失败");
    }
    std::this_thread::sleep_for(1s);

    // 步骤4: 规划轨迹
    RCLCPP_INFO(example->get_logger(), "\n步骤4: 规划轨迹到目标位置");
    RCLCPP_INFO(example->get_logger(), "------------------------------------------------------------");
    auto target_pose = create_pose(-0.07401805371046066, -0.20905423164367676, 0.9532700777053833,
                                     0.7025718092918396, -0.00014736213779542595, -0.0008002749527804554, 0.711612343788147);
    auto plan_response = example->plan_trajectory(target_pose, false);
    if (plan_response && plan_response->success) {
        RCLCPP_INFO(example->get_logger(), "✓ 规划成功: %s", plan_response->message.c_str());
        RCLCPP_INFO(example->get_logger(), "  规划时间: %.3f 秒", plan_response->planning_time);
        RCLCPP_INFO(example->get_logger(), "  轨迹点数: %zu", plan_response->trajectory.points.size());

        // 步骤5: 执行轨迹
        RCLCPP_INFO(example->get_logger(), "\n步骤5: 执行规划好的轨迹");
        RCLCPP_INFO(example->get_logger(), "------------------------------------------------------------");
        auto execute_response = example->execute_trajectory(plan_response->trajectory);
        if (execute_response && execute_response->success) {
            RCLCPP_INFO(example->get_logger(), "✓ 执行成功: %s", execute_response->message.c_str());
        } else {
            RCLCPP_ERROR(example->get_logger(), "✗ 执行失败");
        }
    } else {
        RCLCPP_ERROR(example->get_logger(), "✗ 规划失败");
    }
    std::this_thread::sleep_for(1s);

    // 步骤6: 再次获取状态
    RCLCPP_INFO(example->get_logger(), "\n步骤6: 获取执行后的状态");
    RCLCPP_INFO(example->get_logger(), "------------------------------------------------------------");
    auto state2 = example->get_current_state();
    if (state2 && state2->success) {
        RCLCPP_INFO(example->get_logger(), "✓ 成功获取状态: %s", state2->message.c_str());
        const auto& pos = state2->cartesian_position.position;
        RCLCPP_INFO(example->get_logger(), "  当前位置: x=%.3f, y=%.3f, z=%.3f", pos.x, pos.y, pos.z);
    }

    RCLCPP_INFO(example->get_logger(), "\n============================================================");
    RCLCPP_INFO(example->get_logger(), "综合示例完成");
    RCLCPP_INFO(example->get_logger(), "============================================================\n");

    rclcpp::shutdown();
    return 0;
}

