/*
 * 移动到目标位姿服务调用示例
 * 演示如何调用 /move_to_pose 服务
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/move_to_pose.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class MoveToPoseClient : public rclcpp::Node
{
public:
    MoveToPoseClient() : Node("move_to_pose_client")
    {
        client_ = this->create_client<demo_interface::srv::MoveToPose>("/move_to_pose");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "等待服务 /move_to_pose 可用...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "服务 /move_to_pose 不可用");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务 /move_to_pose 已就绪");
        return true;
    }

    std::shared_ptr<demo_interface::srv::MoveToPose::Response> call_service(
        const geometry_msgs::msg::Pose& target_pose,
        bool use_joints = false,
        float velocity_factor = 0.5f,
        float acceleration_factor = 0.5f)
    {
        auto request = std::make_shared<demo_interface::srv::MoveToPose::Request>();
        request->target_pose = target_pose;
        request->use_joints = use_joints;
        request->velocity_factor = velocity_factor;
        request->acceleration_factor = acceleration_factor;

        RCLCPP_INFO(this->get_logger(), "发送请求: 目标位姿 x=%.3f, y=%.3f, z=%.3f",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "使用关节空间: %s, 速度因子: %.2f, 加速度因子: %.2f",
                    use_joints ? "true" : "false", velocity_factor, acceleration_factor);

        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✓ 成功: %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ 失败 (错误码: %d): %s",
                            response->error_code, response->message.c_str());
            }
            return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
            return nullptr;
        }
    }

private:
    rclcpp::Client<demo_interface::srv::MoveToPose>::SharedPtr client_;
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

    auto client = std::make_shared<MoveToPoseClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 示例1: 移动到指定笛卡尔位置（笛卡尔空间规划）
    RCLCPP_INFO(client->get_logger(), "\n=== 示例1: 笛卡尔空间移动到目标位姿 ===");
    auto target_pose1 = create_pose(-0.07401805371046066, -0.20905423164367676, 0.9532700777053833,
                                     0.7025718092918396, -0.00014736213779542595, -0.0008002749527804554, 0.711612343788147);
    client->call_service(target_pose1, false, 0.5f, 0.5f);

    // 示例2: 使用关节空间规划
    RCLCPP_INFO(client->get_logger(), "\n=== 示例2: 关节空间移动到目标位姿 ===");
    auto target_pose2 = create_pose(-0.07401805371046066, -0.20905423164367676, 0.9532700777053833,
                                     0.7025718092918396, -0.00014736213779542595, -0.0008002749527804554, 0.711612343788147);
    client->call_service(target_pose2, true, 0.3f, 0.3f);

    // 示例3: 高速移动到目标位姿
    RCLCPP_INFO(client->get_logger(), "\n=== 示例3: 高速移动到目标位姿 ===");
    auto target_pose3 = create_pose(-0.07401805371046066, -0.20905423164367676, 0.9532700777053833,
                                     0.7025718092918396, -0.00014736213779542595, -0.0008002749527804554, 0.711612343788147);
    client->call_service(target_pose3, false, 0.8f, 0.8f);

    rclcpp::shutdown();
    return 0;
}

