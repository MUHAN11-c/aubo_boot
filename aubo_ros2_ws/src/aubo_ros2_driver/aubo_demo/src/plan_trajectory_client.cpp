/*
 * 规划轨迹服务调用示例
 * 演示如何调用 /plan_trajectory 服务
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/plan_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class PlanTrajectoryClient : public rclcpp::Node
{
public:
    PlanTrajectoryClient() : Node("plan_trajectory_client")
    {
        client_ = this->create_client<demo_interface::srv::PlanTrajectory>("/plan_trajectory");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "等待服务 /plan_trajectory 可用...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "服务 /plan_trajectory 不可用");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务 /plan_trajectory 已就绪");
        return true;
    }

    std::shared_ptr<demo_interface::srv::PlanTrajectory::Response> call_service(
        const geometry_msgs::msg::Pose& target_pose,
        bool use_joints = false)
    {
        auto request = std::make_shared<demo_interface::srv::PlanTrajectory::Request>();
        request->target_pose = target_pose;
        request->use_joints = use_joints;

        RCLCPP_INFO(this->get_logger(), "发送规划请求: 目标位姿 x=%.3f, y=%.3f, z=%.3f",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "使用关节空间: %s", use_joints ? "true" : "false");

        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✓ 规划成功: %s", response->message.c_str());
                RCLCPP_INFO(this->get_logger(), "  规划时间: %.3f 秒", response->planning_time);
                RCLCPP_INFO(this->get_logger(), "  轨迹点数: %zu", response->trajectory.points.size());
                if (!response->trajectory.points.empty()) {
                    RCLCPP_INFO(this->get_logger(), "  关节数: %zu",
                               response->trajectory.points[0].positions.size());
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ 规划失败: %s", response->message.c_str());
            }
            return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
            return nullptr;
        }
    }

private:
    rclcpp::Client<demo_interface::srv::PlanTrajectory>::SharedPtr client_;
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

    auto client = std::make_shared<PlanTrajectoryClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 示例1: 笛卡尔空间规划
    RCLCPP_INFO(client->get_logger(), "\n=== 示例1: 笛卡尔空间轨迹规划 ===");
    auto target_pose1 = create_pose(-0.07401805371046066, -0.20905423164367676, 0.9532700777053833,
                                     0.7025718092918396, -0.00014736213779542595, -0.0008002749527804554, 0.711612343788147);
    auto response1 = client->call_service(target_pose1, false);

    if (response1 && response1->success) {
        RCLCPP_INFO(client->get_logger(), "规划得到的轨迹有 %zu 个点",
                   response1->trajectory.points.size());
    }

    // 示例2: 关节空间规划
    RCLCPP_INFO(client->get_logger(), "\n=== 示例2: 关节空间轨迹规划 ===");
    auto target_pose2 = create_pose(-0.07401805371046066, -0.20905423164367676, 0.9532700777053833,
                                     0.7025718092918396, -0.00014736213779542595, -0.0008002749527804554, 0.711612343788147);
    auto response2 = client->call_service(target_pose2, true);

    if (response2 && response2->success) {
        RCLCPP_INFO(client->get_logger(), "规划得到的轨迹有 %zu 个点",
                   response2->trajectory.points.size());
    }

    rclcpp::shutdown();
    return 0;
}

