/*
 * 执行轨迹服务调用示例
 * 演示如何调用 /execute_trajectory 服务
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/execute_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ExecuteTrajectoryClient : public rclcpp::Node
{
public:
    ExecuteTrajectoryClient() : Node("execute_trajectory_client")
    {
        client_ = this->create_client<demo_interface::srv::ExecuteTrajectory>("/execute_trajectory");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "等待服务 /execute_trajectory 可用...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "服务 /execute_trajectory 不可用");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务 /execute_trajectory 已就绪");
        return true;
    }

    std::shared_ptr<demo_interface::srv::ExecuteTrajectory::Response> call_service(
        const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        auto request = std::make_shared<demo_interface::srv::ExecuteTrajectory::Request>();
        request->trajectory = trajectory;

        RCLCPP_INFO(this->get_logger(), "发送执行请求: 轨迹包含 %zu 个点", trajectory.points.size());
        std::string joint_names_str;
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
            if (i > 0) joint_names_str += ", ";
            joint_names_str += trajectory.joint_names[i];
        }
        RCLCPP_INFO(this->get_logger(), "关节名称: [%s]", joint_names_str.c_str());

        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✓ 执行成功: %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ 执行失败 (错误码: %d): %s",
                            response->error_code, response->message.c_str());
            }
            return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
            return nullptr;
        }
    }

private:
    rclcpp::Client<demo_interface::srv::ExecuteTrajectory>::SharedPtr client_;
};

trajectory_msgs::msg::JointTrajectory create_simple_trajectory()
{
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {
        "shoulder_joint",
        "upperArm_joint",
        "foreArm_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint"
    };

    // 点1: 初始位置
    trajectory_msgs::msg::JointTrajectoryPoint point1;
    point1.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point1.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point1.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point1.time_from_start.sec = 0;
    point1.time_from_start.nanosec = 0;
    trajectory.points.push_back(point1);

    // 点2: 中间位置
    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.positions = {0.5, 0.3, 0.2, 0.0, 0.0, 0.0};
    point2.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point2.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point2.time_from_start.sec = 2;
    point2.time_from_start.nanosec = 0;
    trajectory.points.push_back(point2);

    // 点3: 目标位置
    trajectory_msgs::msg::JointTrajectoryPoint point3;
    point3.positions = {1.0, 0.5, 0.3, 0.0, 0.0, 0.0};
    point3.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point3.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point3.time_from_start.sec = 4;
    point3.time_from_start.nanosec = 0;
    trajectory.points.push_back(point3);

    return trajectory;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<ExecuteTrajectoryClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 示例: 执行一个简单的轨迹
    RCLCPP_INFO(client->get_logger(), "\n=== 示例: 执行轨迹 ===");
    auto trajectory = create_simple_trajectory();
    client->call_service(trajectory);

    rclcpp::shutdown();
    return 0;
}

